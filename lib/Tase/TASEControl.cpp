#include <stdio.h>
#include <fcntl.h>
#include <sys/sem.h>
#include <cstdlib>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <iostream>
#include <sys/prctl.h>
#include <time.h>
#include "klee/CVAssignment.h"
#include "klee/Internal/ADT/KTest.h"

#include "../Core/Executor.h"
#include "klee/Internal/System/Time.h"
#include "klee/CommandLine.h"

using namespace llvm;
using namespace klee;

extern FILE * prev_stdout_log;
extern std::stringstream worker_ID_stream;
extern std::string prev_worker_ID;

extern void deserializeAssignments ( void * buf, int bufSize, klee::Executor * exec, CVAssignment * cv);
extern void reset_run_timers();
extern void print_run_timers();
 void cycleTASELogs(bool isReplay);

extern double run_start_time;
extern double run_interp_time;
extern double run_fork_time;
extern double interp_enter_time;

extern bool taseDebug;

double last_message_verification_time = 0;

typedef struct RoundRecord {
  uint16_t RoundNumber;  //Index of message, starting with 0
  uint64_t RoundRealTime;  //Time to verify message in microseconds  
  uint16_t SocketEventType;  //0 for c2s, 1 for s2c
  int SocketEventSize;  //Size of message in bytes
  struct timeval SocketEventTimestamp;
  
} RoundRecord;

std::vector<RoundRecord> s2c_records; //Populated with logging info on first pass
std::vector<RoundRecord> s2c_records_replay; //Populated from first pass and used during replay
extern KTestObjectVector ktov;


int peekCtr = 0;
bool guess_path_for_parent = false; 
enum nextMessageType{C2S, S2C, NONE};
void guessParentPath(int * childNum, int * parentNum);

//Perf debugging
extern double target_start_time;
extern uint64_t total_interp_returns;

typedef struct WorkerInfo {
  int pid;
  int branches;
  int round;
  int pass;
  int parent;
} WorkerInfo;



int round_count = 0;
int pass_count = 0; //Pass ctr for current round of verification
int run_count = 0;
int msg_count = 0;
int tase_branches = 0;
int multipassAssignmentSize = 20000;  //Totally arbitrary. Size of mmap'd multipass assignment buffer.
void * MPAPtr;  //Ptr to serialized multipass info for current round of verification
int * replayPIDPtr; //Ptr to the pid storing the replay PID for the current round of verification
int * replayLock ; //Lock to request replay.  1 is available, 0 unavailable.

int * latestRoundPtr; //Pointer to furthest round found so far in verification
int managerRoundCtr = 0;

CVAssignment  prevMPA;


const int QR_OFF = 512;
const int QA_OFF = 1536;
const int RECORD_OFF = 15360;
extern int QR_MAX_WORKERS;
const int QA_MAX_WORKERS = 495;
const int MAX_ROUND_RECORDS = 5000;
void * ms_base;
int ms_size = 320000; //Size in bytes of shared manager structures buffer.

void * ms_QR_base;
int * ms_QR_size_ptr; //Pointer to num of workers in QR, not size in bytes

void * ms_QA_base;
int * ms_QA_size_ptr; //Pointer to num of workers in QA, not size in bytes

void * ms_Records_base;
int * ms_Records_count_ptr;  //Pointer to num of records in record list, not size in bytes

int * target_started_ptr;
int * target_ended_ptr;

int * last_msg_count_ptr; //Value holds last msg count at end of verification.  Should
//equal total number of S2C and C2S messages.


int semID; //Global semaphore ID for sync

struct sembuf sem_lock = {0, -1, 0 | SEM_UNDO}; // {sem index, inc/dec, flags}
struct sembuf sem_unlock = {0, 1, 0 | SEM_UNDO};// SEM_UNDO added to release
//lock if process dies.

#ifndef TASE_OPENSSL
int * total_workers;
int * total_branches;
int * explorer_get_next_pid;    //Should be 1 or 0 to indicate if it's time to get the next PID
void * explorer_pid_stack_base; //For DFS
void * explorer_pid_queue_base; //For BFS
int * BFS_Front_Ptr;  //Todo -- Need to shift the entire queue down periodically, or use another data structure.
int * BFS_Back_Ptr;
uint64_t MAX_QUEUE_SIZE = 4000;
int * DFS_Stack_Ptr;
uint64_t MAX_STACK_SIZE = 4000;
const int EXP_WORKER_QUEUE_OFF = 20000;
const int EXP_WORKER_STACK_OFF = 30000;

int BFS_enqueue();
int DFS_pop();
void BFS_enqueue(int pid);
void DFS_push(int pid);

void manage_exploration_workers();
int tase_explorer_fork(int parentPID, uint64_t rip);
#endif

extern int orig_stdout_fd;

void get_sem_lock () {
  int res =  semop(semID, &sem_lock, 1);
  if (res == 0) {
    return;
  }
  else {
    printf("Error getting sem lock \n");
    std::cout.flush();
    perror("Error in get_sem_lock");
    std::exit(EXIT_FAILURE);
  }
}

void release_sem_lock () {
  int res = semop(semID, &sem_unlock,1);
  if (res == 0)
    return;
  else {
    perror("Error in release_sem_lock");
    std::exit(EXIT_FAILURE);
  }
}

int initialize_semaphore(int semKey) {
  semID = semget(semKey, 1, IPC_CREAT |IPC_EXCL | 0660 );
  if ( semID == -1) {
    perror("Error creating semaphore ");
    std::exit(EXIT_FAILURE);
  }
  //Todo -- Double check to see if we need to initialize
  //semaphore explicitly to 1.
 int res = semctl(semID, 0, SETVAL, 1);
 if (res == -1) {
   perror("Error initializing semaphore \n");
   std::exit(EXIT_FAILURE);
 }
 return semID;
}

static void printWorkerInfo(WorkerInfo * wi) {
  printf("<pid %d> <branches %d> <round %d> <pass %d> <parent %d> \n", wi->pid, wi->branches, wi->round, wi->pass, wi->parent);
}

static void printQ( int *base_size_ptr, void * basePtr) {
  int size = *base_size_ptr;
  if (size < 0) {
    printf("Error in printQ: size less than zero \n");
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }
  printf("Printing queue: \n");
  if (size == 0)
    printf("Nothing in queue! \n");
  while (size > 0) {
    size--;
    WorkerInfo * worker = (WorkerInfo *) ( (uint64_t) basePtr + (uint64_t) (size * sizeof(WorkerInfo)));
    printWorkerInfo(worker);
  }
}

static void printQR() {
  printf("Printing QR: \n");
  printQ(ms_QR_size_ptr, ms_QR_base);
}

static void printQA() {
  printf("Printing QA: \n");
  printQ(ms_QA_size_ptr, ms_QA_base);
}

WorkerInfo * PidInQ(int pid, int * QSizePtr, void * basePtr) {
  int Qsize = *QSizePtr;
  for (int i = 0; i < Qsize; i++) {
    WorkerInfo * wi = (WorkerInfo *) ((uint64_t) basePtr + (uint64_t) (i * sizeof(WorkerInfo)) );
    if (wi->pid == pid)
      return wi;
  }
  return NULL;  
}

WorkerInfo * PidInQR(int pid) {
  return PidInQ(pid, ms_QR_size_ptr, ms_QR_base);
}

WorkerInfo * PidInQA(int pid) {
  return PidInQ(pid, ms_QA_size_ptr, ms_QA_base);
}

//Return pointer to worker in queue with earliest round, pass, branch.
WorkerInfo * getEarliestWorker(void * basePtr, int Qsize ) {
  
  //Initialization case
  WorkerInfo * earliest = (WorkerInfo *) (basePtr);
  if (basePtr == NULL || Qsize == 0) {
    fprintf(stderr,"getEarliestWorker called on empty/null queue \n");
    return NULL;
  }
  
  for (int i = 0; i < Qsize; i++) {
    WorkerInfo * currWorker = (WorkerInfo *) ((uint64_t) basePtr + (uint64_t)( i *(sizeof(WorkerInfo))));
    if (currWorker->round < earliest->round) {
      earliest = currWorker; //New earliest
    } else if (currWorker->round == earliest->round) { //Tie on round
      if (currWorker->pass < earliest->pass) { 
	earliest = currWorker; //New earliest
      } /* else if (currWorker->pass == earliest->pass) { //Tie on pass
	   if (currWorker->branches < earliest->branches) 
	   earliest = currWorker; //New earliest
	*/
    }
  }
  return earliest;
}


WorkerInfo * getLatestWorker(void * basePtr, int Qsize) {
  if (taseDebug) {
    printf("Manager calling getLatestWorker \n");
    fflush(stdout);
  }

  //Initialization case
  WorkerInfo * latest = (WorkerInfo *) (basePtr);
  if (basePtr == NULL || Qsize == 0) {
    fprintf(stderr,"getLatestWorker called on empty/null queue \n");
    return NULL;
  }

  for (int i = 0; i < Qsize; i++) {
    WorkerInfo * currWorker = (WorkerInfo *) ((uint64_t) basePtr + (uint64_t)( i *(sizeof(WorkerInfo))));
    if (currWorker->round > latest->round) {
      latest = currWorker; //New latest
    } else if (currWorker->round == latest->round) { //Tie on round
      if (currWorker->pass > latest->pass) {
	latest = currWorker; //New latest
      } /* else if (currWorker->pass == latest->pass) { //Tie on pass
	   if (currWorker->branches > latest->branches)
	   latest = currWorker; //New latest
	*/
    }
  }
  return latest;
}

void removeFromQ(WorkerInfo * worker, int * sizePtr, void * basePtr) {
  
  int size = *sizePtr;
  WorkerInfo * found = PidInQ(worker->pid, sizePtr, basePtr);
  if (found == NULL) {
    fprintf(stderr,"Error: Trying to remove pid %d not in Q \n", worker->pid);
    std::exit(EXIT_FAILURE);
  }
  memset((void *) found, 0, sizeof(WorkerInfo));
  //Slide values down
  WorkerInfo * edgePtr = (WorkerInfo *) ((uint64_t) basePtr + (size -1) * sizeof(WorkerInfo));
  WorkerInfo * searchPtr = found;
  WorkerInfo * tmpPtr;
  while (searchPtr != edgePtr) {
    tmpPtr = searchPtr;
    searchPtr++;
    memcpy ((void *) (tmpPtr), searchPtr, sizeof(WorkerInfo));
  }
  memset((void *) searchPtr, 0, sizeof(WorkerInfo));
  *sizePtr = *sizePtr -1;
}

void removeFromQR(WorkerInfo * worker) {

  if (worker == NULL) {
    fprintf(stderr,"ERROR: calling removeFromQR on null ptr \n");
    fflush(stdout);
    std::exit(EXIT_FAILURE);
  }
  removeFromQ(worker, ms_QR_size_ptr, ms_QR_base);

}

void removeFromQA(WorkerInfo * worker) {
  if (worker == NULL) {
    fprintf(stderr,"ERROR: calling removeFromQA on null ptr \n");
    fflush(stdout);
    std::exit(EXIT_FAILURE);
  }
  removeFromQ(worker, ms_QA_size_ptr, ms_QA_base);
}

void addToQ(WorkerInfo * worker, int * sizePtr, void * basePtr) {
  void * dst = (void *) ((uint64_t) basePtr + (*sizePtr) * sizeof(WorkerInfo));
  void * src = (void *) worker;
  memcpy(dst ,  src , sizeof(WorkerInfo));
  *sizePtr = *sizePtr +1;
}

void addToQR(WorkerInfo * worker) {
  addToQ(worker, ms_QR_size_ptr, ms_QR_base);
}

void addToQA(WorkerInfo * worker) {
  addToQ(worker, ms_QA_size_ptr, ms_QA_base);
}

void QRtoQA(WorkerInfo * worker) {
  WorkerInfo tmp;  
  memcpy ((void *) &tmp, (void *) worker, sizeof(WorkerInfo));
  removeFromQR (worker);
  addToQA(&tmp);
}

void QAtoQR(WorkerInfo * worker) {
  WorkerInfo tmp;
  memcpy((void *) &tmp, (void *) worker, sizeof(WorkerInfo));
  removeFromQA (worker);
  addToQR(&tmp);
}

void writeRecordsToFile(FILE * f, std::vector<RoundRecord> & records, double lastMessageVerTime) {


  //Alloc buffer to hold enough space for the records,  last message time (double), and number of records (int);
  int totalBytes = sizeof(RoundRecord) * records.size();
  totalBytes += sizeof(int) + sizeof(double);
  char buf [totalBytes];
  char * itrPtr = buf;

  //Handle metadata first
  memcpy((void *) itrPtr, (void *) &lastMessageVerTime, sizeof(double));
  itrPtr += sizeof(double);
  
  int rsize = records.size();
  memcpy((void *) itrPtr, (void *) &(rsize), sizeof(int));
  itrPtr += sizeof(int);
  
  //Add actual round records  
  RoundRecord * directRecordsPtr = records.data();
  
  for (int i = 0 ; i < records.size(); i++) {
    memcpy ((void *)itrPtr, &(directRecordsPtr[i]), sizeof(RoundRecord));
    itrPtr = itrPtr + sizeof(RoundRecord);
  }
  fwrite((void *) buf, 1, totalBytes, f);
  fclose(f);
}

std::vector<RoundRecord> readRoundRecordsFromFile(FILE * f, double * lastMessageVerTime) {

  //Read metadata first
  char metaDataBuf [ sizeof(double) + sizeof(int)];
  fread(metaDataBuf, 1,  sizeof(double) + sizeof(int), f);
  double tmpD;
  int numRecords;
  memcpy (&tmpD,  metaDataBuf, sizeof(double));
  *lastMessageVerTime = tmpD;

  memcpy (&numRecords, metaDataBuf + sizeof(double), sizeof(int));


  //Grab the RoundRecords
  char recordsBuf [numRecords * sizeof(RoundRecord)];
  fread(recordsBuf, 1, numRecords * sizeof(RoundRecord), f);

  char * itrPtr = recordsBuf;
  std::vector<RoundRecord> vec;
  for (int i = 0; i < numRecords; i++) {
    RoundRecord r;
    memcpy (&r, (void *) itrPtr, sizeof(RoundRecord));
    itrPtr += sizeof(RoundRecord);
    vec.push_back(r);
  }
  return vec;
}


void addRoundRecord(RoundRecord r) {
  void * dst = (void *) ((uint64_t) ms_Records_base + *ms_Records_count_ptr * sizeof(RoundRecord));
  void * src = (void *) &r;
  memcpy (dst,src, sizeof(RoundRecord));
  *ms_Records_count_ptr = *ms_Records_count_ptr + 1;

  if (*ms_Records_count_ptr > MAX_ROUND_RECORDS) {
    fprintf(stderr, "FATAL ERROR: Too many records \n");
    std::exit(EXIT_FAILURE);
  }
}
  
void select_workers () {

  if (*ms_QR_size_ptr == QR_MAX_WORKERS) {

    //Get earliest worker by round, pass, branch

    WorkerInfo * earliestWorkerQR = getEarliestWorker(ms_QR_base, *ms_QR_size_ptr);

    if (earliestWorkerQR->round < managerRoundCtr ) {
      int res = kill(earliestWorkerQR->pid, SIGSTOP);
      if (res == -1) {
	perror("Error sigstopping in select_workers \n");
	fprintf(stderr, "Issue delivering SIGSTOP to pid %d \n", earliestWorkerQR->pid);
      } else {
	printf("Manager kicking pid from QR %d; pid is in round %d when latest round is %d \n", earliestWorkerQR->pid, earliestWorkerQR->round, managerRoundCtr);
      }
      QRtoQA(earliestWorkerQR);
    }
    
  
  }

  if (*ms_QR_size_ptr < QR_MAX_WORKERS && *ms_QA_size_ptr != 0) {

    if (taseDebug) {
      printQA();
      printQR();
    }
    WorkerInfo * latestWorker = getLatestWorker(ms_QA_base, *ms_QA_size_ptr);

    if (latestWorker->round < managerRoundCtr && *ms_QR_size_ptr > 0) {
      if (taseDebug) {
	printf("No workers in QA in latest round %d.  Not moving to QR. \n", managerRoundCtr);
      }
    } else {
      int res = kill(latestWorker->pid, SIGCONT);
      if (res == -1){
	perror("Error during kill sigcont \n");
	printf("Error during kill sigcont \n");
	fflush(stdout);
      } 

      QAtoQR(latestWorker);

    }
  }
}

void print_eval_header(FILE * f) {
  

  fprintf(f, "RoundNumber,RoundUserTime,RoundRealTime,RoundRealTimePassOne,RoundSysTime,MergedStates,MergerTime,SearcherTime,ForkTime,InstructionCount,InstructionCountPassOne,RecvInstructionCount,RebuildTime,BindingsSolveTime,ExecTreeTime,EditDistTime,EditDistBuildTime,EditDistHintTime,EditDistStatTime,StageCount,StateCloneCount,StateRemoveCount,SocketEventType,SocketEventSize,SocketEventTimestamp,ValidPathInstructionCount,ValidPathInstructionCountPassOne,SymbolicVariableCount,PassCount,EditDist,EditDistK,EditDistMedoidCount,EditDistSelfFirstMedoid,EditDistSelfLastMedoid,EditDistSelfSocketEvent,EditDistSocketEventFirstMedoid,EditDistSocketEventLastMedoid,BackTrackCount,RewriteTime,SimplifyExprTime,SimplifyExprTimeV2,SimplifyExprTimeV3,SimplifyExprTimeV4,SolverTime,QueryTime,CexCacheTime,QueryConstructTime,ResolveTime,Queries,QueriesInvalid,QueriesValid,QueryCacheHits,QueryCacheMisses,QueriesConstructs");

  fprintf(f,"\n");
  
}

void print_eval_record(FILE * f, RoundRecord r) {

  //Column 1 
  fprintf(f, "%d,", r.RoundNumber);
  //Columns 2-3
  fprintf(f, "0,%lu,", r.RoundRealTime);
  //Columns 4-22
  fprintf(f, "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,");
  //Column 23
  fprintf(f,"%d,",r.SocketEventType);
  //Column 24
  fprintf(f,"%d,", r.SocketEventSize);
  //Column 25
  //Formatting from cliver
  uint64_t timestamp =
    (1000000)*((uint64_t)r.SocketEventTimestamp.tv_sec)
    + (uint64_t)r.SocketEventTimestamp.tv_usec;
  fprintf(f,"%lu,", timestamp);
  //Columns 26-53
  for (int i = 0; i < 28; i++)
    fprintf(f, "0,");
  //Column 54
  fprintf(f,"0");

  fprintf(f,"\n");
  
}

void manage_workers () {

#ifndef TASE_OPENSSL
  manage_exploration_workers();
#else
  get_sem_lock();
  
  //Check to see if analysis completed
  if (*target_ended_ptr == 1) {
    double ct = util::getWallTime();
    fprintf(stderr, "Verification complete at time %lf \n", ct - target_start_time);

    //Kludge to fake a cliver-compatible log
    FILE * f = fopen("TASE_RESULTS.csv", "w+");
    print_eval_header(f);
    RoundRecord * rPtr = (RoundRecord *) ms_Records_base;
    
    for (int i = 0 ; i < *ms_Records_count_ptr; i++) {
      RoundRecord r = *rPtr;
      print_eval_record(f, r);
      rPtr += 1;
    }

    //R scripts expect 2 last rows to be ignored, so populate them with bogus late values.
    RoundRecord rTmp1;
    struct timeval t;
    t.tv_usec = 49380481;
    t.tv_sec = 14143968;
    rTmp1.RoundNumber = *last_msg_count_ptr ;
    rTmp1.SocketEventSize = 0;
    rTmp1.SocketEventType = 0;
    rTmp1.RoundRealTime = 0;
    rTmp1.SocketEventTimestamp = t;
    print_eval_record(f,rTmp1);
    RoundRecord rTmp2;
    rTmp2 = rTmp1;
    rTmp2.RoundNumber = *last_msg_count_ptr +1;
    
    print_eval_record(f,rTmp2);
    
    fflush(stdout);
    std::exit(EXIT_SUCCESS);
  }
  bool sawNewRound = false;
  double diff;
  if (managerRoundCtr < *latestRoundPtr) {
    sawNewRound = true;
    managerRoundCtr = *latestRoundPtr;
    diff = util::getWallTime() - target_start_time;
    
  }

  //Exit case
  if (*ms_QA_size_ptr == 0 && *ms_QR_size_ptr == 0 && *target_started_ptr == 1) {

    if (testType == VERIFICATION) {
      fprintf(stderr,"Manager found empty QA and QR.  Latest round reached was %d \n", managerRoundCtr);
      if (*target_ended_ptr != 1) {
	fprintf(stderr,"Verification failed at time %lf. \n", util::getWallTime() - target_start_time);
      } else {
	fprintf(stderr,"All messages verified. \n");
      }
    } else {
      fprintf(stderr, "Manager found emptied QA and QR.\n");
    }
    
    std::exit(EXIT_SUCCESS);
  }
  
  if (*ms_QR_size_ptr > QR_MAX_WORKERS) {
    fprintf(stderr,"FATAL ERROR: found more workers than expected in manage_workers \n");
    std::exit(EXIT_FAILURE);
  }

  select_workers();
  release_sem_lock();
  if (sawNewRound && !noLog) {
    fprintf(stderr,"Manager sees new round %d starting at time %lf \n", managerRoundCtr, diff);
  }
  #endif
}

//Todo -- Any more cleanup needed?
void worker_exit() {
 #ifdef TASE_OPENSSL

  if (taseManager != true) {
    printf("WARNING: worker_exit called without taseManager \n");
    std::cout.flush();
    std::exit(EXIT_SUCCESS);
  } else {
    int p = getpid();
    get_sem_lock();
    removeFromQR(PidInQR(p));
    release_sem_lock();
    printf("Worker %d attempting to exit and remove self from QR \n", getpid());
    std::cout.flush();
    std::exit(EXIT_SUCCESS);
  }
    #else

  get_sem_lock();
  *total_workers = *total_workers -1 ;
  release_sem_lock();

  if (taseManager != true) {
    printf("WARNING: worker_exit called without taseManager \n");
    std::cout.flush();
    std::exit(EXIT_SUCCESS);
  } else {
    int p = getpid();
    printf("Worker pid %d attempting to exit \n", getpid());
    std::cout.flush();
    std::exit(EXIT_SUCCESS);
  }

#endif
}

void worker_self_term() {

  int p = getpid();
  get_sem_lock();
  removeFromQR(PidInQR(p));
  release_sem_lock();
  if (taseDebug) {
    printf("Worker %d is in round %d when latest round is %d. Worker exiting. \n", getpid(), round_count, *latestRoundPtr );
    fflush(stdout);
  }
  std::exit(EXIT_SUCCESS);
}

int tase_explorer_fork(int parentPID, uint64_t rip) {
  get_sem_lock(); //System V semaphore adjustments aren't inherited across fork call

  *total_branches = *total_branches + 1;
  *total_workers = *total_workers + 1;

  //Fork off a child process for the "TRUE" branch, and add it to the queue/stack. ----------

  int trueChildPID = ::fork();

  if (trueChildPID == -1) {
    printf("FATAL ERROR during forking \n");
    fflush(stdout);
    fprintf(stderr, "ERROR during fork; pid is %d \n", getpid());
    fflush(stderr);
    perror("Fork error \n");
  }

  if (trueChildPID != 0) {  //PARENT
    if (!noLog) {
      printf("Parent PID %d forked off child %d at rip 0x%lx for TRUE branch \n", parentPID, trueChildPID, rip);
      fflush(stdout);
    }

    //Block until child has sigstop'd.
    while (true) {
      int status;
      int res = waitpid(trueChildPID, &status, WUNTRACED);
      if (WIFSTOPPED(status))
	break;
    }
    if (explorationType == BFS) {
      BFS_enqueue(trueChildPID);
    } else if (explorationType == DFS) {
      DFS_push(trueChildPID);
    } else {
      printf("Error: no exploration type defined \n");
      return -1;
    }
  } else  {                //CHILD
    raise(SIGSTOP);
    cycleTASELogs(false);
    return 1; // Defaults to 0 // Go back to path exploration
  }
  //End true process setup-------------------------------------------------------------------


  //Fork off a child process for the "FALSE" branch, and add it to the queue/stack.----------
  int falseChildPID = ::fork();

  if (falseChildPID != 0) {  //PARENT
    if (!noLog) {
      printf("Parent PID %d forked off child %d at rip 0x%lx for FALSE branch \n", parentPID, falseChildPID, rip);
      fflush(stdout);
    }

    //Block until child has sigstop'd.
    while (true) {
      int status;
      int res = waitpid(falseChildPID, &status, WUNTRACED);
      if (WIFSTOPPED(status))
	break;
    }

    if (explorationType == BFS) {
      BFS_enqueue(trueChildPID);
    } else if (explorationType == DFS) {
      DFS_push(trueChildPID);
    } else {
      printf("Error: no exploration type defined \n");
      return -1;
    }

  } else  {                //CHILD
    raise(SIGSTOP);
    cycleTASELogs(false);
    return 0;
  }
  //End False process setup-------------------------------------------------------


  *explorer_get_next_pid = 1;

  release_sem_lock();

  std::exit(EXIT_SUCCESS);

}


  

int tase_fork(int parentPID, uint64_t rip) {
#ifndef TASE_OPENSSL
  return tase_explorer_fork(parentPID, rip);
#else

  tase_branches++;
  double curr_time = util::getWallTime();
  if (!noLog) {
    printf("PID %d entering tase_fork at rip 0x%lx %lf seconds after analysis started \n", parentPID, rip, curr_time - target_start_time);
  }
  if (dontFork) {
    printf("Forking is disabled.  Shutting down \n");
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }
  if (taseManager) {
    if (taseDebug) {
      printf("TASE FORKING! \n");
    }
   
    
    //Should actually place check to latestRoundPtr in semaphore
    if (round_count < *latestRoundPtr && workerSelfTerminate)  {
      worker_self_term();
    }

    
    WorkerInfo wi;
    wi.round = round_count;
    wi.pass  = pass_count;
    wi.branches = tase_branches;
    wi.parent = parentPID;

    if(!noLog) {
      fflush(stdout);
    }

    int childReturn = 0;
    int parentReturn = 1;

    if (guess_path_for_parent)
      guessParentPath(&childReturn, &parentReturn);

    fflush(stdout);
    fflush(stderr);
    
    double T1 =  util::getWallTime();    
    int trueChildPID = ::fork();
    if (trueChildPID == -1) {
      printf("FATAL ERROR during forking \n");
      fflush(stdout);
      fprintf(stderr, "ERROR during fork; pid is %d \n", getpid());
      fflush(stderr);
      perror("Fork error \n");
    }
    double T2 = util::getWallTime();
    
    if (trueChildPID != 0) {  //PARENT
      if (!noLog) {
	printf("Parent PID %d forked off child %d at rip 0x%lx for TRUE branch \n", parentPID, trueChildPID, rip);
	fflush(stdout);
      }       
      run_fork_time += T2-T1;
      if (!noLog) {
	printf("Took time %lf just to fork \n", T2-T1);
	fflush(stdout);
      }
      
      //Block until child has sigstop'd.
      while (true) {
	int status;
	int res = waitpid(trueChildPID, &status, WUNTRACED);
	if (WIFSTOPPED(status))
	  break;
      }	
      get_sem_lock();

      wi.pid = trueChildPID;     
      addToQA(&wi);
      
    } else  {                //CHILD
      raise(SIGSTOP);
      if (taseDebug) {
	get_sem_lock();
	if (PidInQR(getpid()) == NULL)
	  fprintf(stderr,"Error: Fork pid %d running but not in QR ", getpid());
	release_sem_lock();
      }
      cycleTASELogs(false);
      return childReturn; // Defaults to 0 // Go back to path exploration
    } 

    curr_time = util::getWallTime();
    if (modelDebug)
      printf("Parent returning to path exploration %lf seconds into analysis \n", curr_time - target_start_time);
    
    //Make self the True branch
    WorkerInfo * myInfo = PidInQR(getpid());
    myInfo->branches++;
    release_sem_lock();
    print_run_timers();
    cycleTASELogs(false);
    return parentReturn;  //Defaults to 1

  } else {
    int pid = ::fork();
    return pid;
  }
#endif
}

void tase_exit() {
  if (taseManager) {
    printf("PID %d calling tase_exit \n", getpid());  
    std::cout.flush();
    get_sem_lock();
    removeFromQR(PidInQR(getpid()));
    release_sem_lock();
    std::exit(EXIT_SUCCESS); 
  } else {
    printf("PID %d calling tase_exit \n", getpid());
    std::cout.flush();
    std::exit(EXIT_SUCCESS);
  }
}

void initManagerStructures() {
   
   initialize_semaphore(getpid());
   
   ms_base = mmap(NULL, ms_size, PROT_READ|PROT_WRITE, MAP_ANON|MAP_SHARED, -1, 0);

   //--------------------------------------------
   //Random shared vars--------------------------
   //-------------------------------------------
   target_started_ptr = ((int *) ms_base) + 8;
   *target_started_ptr = 0; //Switches to 1 when we execute target

   target_ended_ptr  = ((int *) ms_base) + 16;
   *target_ended_ptr = 0;  //First worker to complete verification flips to 1
   
   latestRoundPtr = ((int *) ms_base) + 24;
   *latestRoundPtr = 0;

   last_msg_count_ptr = ((int *) ms_base) + 32; //Number of C2S and S2C messages 
   *last_msg_count_ptr = 0;
   
   //--------------------------------------------
   //Space for queues and records----------------
   //--------------------------------------------
   ms_QR_base =  ms_base + QR_OFF;
   ms_QR_size_ptr = (int *)(ms_QR_base -4);
   *ms_QR_size_ptr = 0;
      
   ms_QA_base =  ms_base + QA_OFF;
   ms_QA_size_ptr = (int *)(ms_QA_base -4);
   *ms_QA_size_ptr = 0;

   ms_Records_base = (ms_base) + RECORD_OFF;
   ms_Records_count_ptr = (int *) (ms_Records_base -4);
   *ms_Records_count_ptr = 0;

   #ifndef TASE_OPENSSL
   total_workers = (int*) ( (ms_base) + EXP_WORKER_STACK_OFF - 192);
   *total_workers = 1;
   total_branches = (int*) ( (ms_base) + EXP_WORKER_STACK_OFF - 128);
   *total_branches = 1;
   explorer_get_next_pid   = (int*) ( (ms_base) + EXP_WORKER_STACK_OFF - 64);
   *explorer_get_next_pid = 0;
   explorer_pid_stack_base =  (ms_base) + EXP_WORKER_STACK_OFF;
   explorer_pid_queue_base =  (ms_base) + EXP_WORKER_QUEUE_OFF;

   BFS_Front_Ptr = (int *) explorer_pid_queue_base;
   BFS_Back_Ptr  = (int *) explorer_pid_queue_base;
   DFS_Stack_Ptr = (int *) explorer_pid_stack_base;

   #endif

   //-------------------------------------------
   //Boundary checks----------------------------
   //-------------------------------------------
   //Note that 20 bytes of "wiggle room" is added
   
   //Is QR too big?
   if ( (uint64_t) ms_QR_base + (sizeof(WorkerInfo) * QR_MAX_WORKERS) + 20 > (uint64_t) ms_QA_base) {
     fprintf(stderr, "FATAL MANAGER ERROR: Not enough space for QR \n");
     std::exit(EXIT_FAILURE);
   }

   //Is QA too big?
   if ( (uint64_t) ms_QA_base + (sizeof(WorkerInfo) * QA_MAX_WORKERS) + 20 > (uint64_t) ms_Records_base ) {
     fprintf(stderr, "FATAL MANAGER ERROR: Not enough space for QA \n");
     std::exit(EXIT_FAILURE);
   }

   //Is the records buffer too big?
   if ( (uint64_t) ms_Records_base + (sizeof(RoundRecord) * MAX_ROUND_RECORDS) + 20 > (uint64_t) ms_base + (uint64_t) ms_size) {
     fprintf(stderr, "FATAL MANAGER ERROR: Not enough space for record info \n");
     fprintf(stderr, "Need at least %lu bytes just for records \n", (sizeof(RoundRecord) * MAX_ROUND_RECORDS) + 20);
     std::exit(EXIT_FAILURE);
   }
   
   int res = prctl(PR_SET_CHILD_SUBREAPER, 1);
   if (res == -1) {
     perror("Subreaper err ");
     fprintf(stderr, "Exiting due to reaper error in initManagerStructures \n");
     std::exit(EXIT_FAILURE);
   }
 }


//Fork off a child, and schedule it to explore the current round.

//Child will learn assignments and use the parent as the replay process.
//GCM in OpenSSL for TLS 1.2 requires only two passes, but we can
//generalize to have more.  Ideally, the final pass should be conducted
//by the parent to avoid deepening the process tree across successive rounds
//of verification, but that's an optimization.
void multipass_start_round (klee::Executor * theExecutor, bool isReplay) {
  if (modelDebug) {
    printf("Hit top of multipass_start_round \n");
    std::cout.flush();
  }

  

  if (round_count < *latestRoundPtr  && workerSelfTerminate)  {
    get_sem_lock(); //DBG -- Make sure this is OK
    removeFromQR(PidInQR(getpid()));
   
    release_sem_lock(); //is this safe?
    if (taseDebug){
      printf("Worker %d is in round %d when latest round is %d. Worker exiting. \n", getpid(), round_count, *latestRoundPtr );
      fflush(stdout);
    }
    std::exit(EXIT_SUCCESS);
  } else {
    if (taseDebug) {
      printf("Worker sees latest round as %d; updating to %d \n", *latestRoundPtr, round_count);
    }

    get_sem_lock();
    //Make record for eval later
    if (round_count != 0 && round_count > *latestRoundPtr) {
      
      //ABH Added      
      printf("Adding %d s2c records at end of round %d \n", s2c_records_replay.size(), round_count);
      for (std::vector<RoundRecord>::iterator itr = s2c_records_replay.begin(); itr < s2c_records_replay.end(); itr++) {
	addRoundRecord(*itr);
      }
            
      s2c_records.clear();
      s2c_records_replay.clear();
      
      
      if(taseDebug) {
	printf("Added S2C records \n");
	fflush(stdout);
      }
      
      
      *latestRoundPtr = round_count;
      double currTime = util::getWallTime();
      double RT = currTime - last_message_verification_time;
      last_message_verification_time = currTime;
      
      KTestObject * kto = &(ktov.objects[ktov.playback_index -1]);
      int eventType = 0;
      if (strcmp(kto->name ,"c2s") == 0) {
	eventType = 0;
      } else if (strcmp(kto->name,"s2c") == 0) {
	eventType = 1;
      } else {
	printf(" ERROR: unrecognized ktest object type \n");
	fflush(stdout);
	std::exit(EXIT_FAILURE);
      }
      
      RoundRecord r;
      r.RoundNumber = msg_count;
      //r.RoundNumber = round_count -1;
      r.RoundRealTime = RT * 1000000.; 
      r.SocketEventType = eventType;
      r.SocketEventSize = kto->numBytes;
      r.SocketEventTimestamp = kto->timestamp ;
      addRoundRecord(r);
      
    }
    release_sem_lock();
    
  }


  int parentPID = getpid();
  *replayPIDPtr = getpid();

  fflush(stdout);
  fflush(stderr);
  
  double T1 = util::getWallTime();
  int childPID = ::fork();
  if (childPID == -1) {
    printf("Error during forking \n");
    fprintf(stderr, "ERROR during forking; pid is %d \n", getpid());
    perror("Fork error \n");
  }
  
  double T2 = util::getWallTime();


   
  
  if (childPID != 0) { //PARENT
    if (!noLog) {
      printf("Parent PID %d forking off child pid %d for starting round %d \n", parentPID,
	     childPID, round_count);
      fflush(stdout);
    }
    run_fork_time +=  T2-T1;
    //Block until child has sigstop'd
    while (true) {
      int status;
      int res = waitpid(childPID, &status, WUNTRACED);
      if (res == -1)
	  perror("Waitpid error in multipass_start_round()");
      if (WIFSTOPPED(status))
	break;
    }
    //printf("In parent path \n");
    
    get_sem_lock();

    //Swap parent for child into QR
    
    WorkerInfo * myInfo = PidInQR(getpid());
    if (myInfo != NULL) {
      myInfo->pid = childPID;
    } else {
      printf("ERROR: could not find self in QR \n");
      fflush(stdout);
      std::exit(EXIT_FAILURE);
    }

    release_sem_lock();

    //TEST
    print_run_timers();

    //END TEST

    
    int res = kill( childPID, SIGCONT);
    if (res == -1){
	perror("Error during kill sigcont \n");
	printf("Error during kill sigcont \n");
	fflush(stdout);
    } 

    
    raise(SIGSTOP);

    //Replay path
    //-------------------------------------

    
    int replayingFromPID;
    get_sem_lock();
    
    
    //Pick up round_count and pass_count
    WorkerInfo * replayWI = PidInQR(getpid());
    if (myInfo != NULL) {
      round_count = replayWI->round;
      pass_count = replayWI->pass;
      replayingFromPID = replayWI->parent;
    } else {
      printf("ERROR: could not find self in QR \n");
      fflush(stdout);
      std::exit(EXIT_FAILURE);
    }
    
    //Pickup assignment info if necessary
    if (*(uint8_t *) MPAPtr != 0) {
      if (taseDebug) {
	printf("Attempting to deserialize MP Assignments \n");
	std::cout.flush();
      }
      if (*replayLock != 0)
	printf("IMPORTANT: Error: control debug - replay lock has value %d in multipass_start_round \n", *replayLock);
      //std::cout.flush();
      prevMPA.clear();
      deserializeAssignments(MPAPtr, multipassAssignmentSize, theExecutor, &prevMPA);
      memset(MPAPtr, 0, multipassAssignmentSize); //Wipe out the multipass assignment to be safe
      if (taseDebug) {
	printf("Printing assignments AFTER deserialization \n");
	prevMPA.printAllAssignments(NULL);
      }
      *replayLock = 1;
      
    } else {
      printf( "IMPORTANT: ERROR: control debug: deserializing from empty buf 0x%lx for replay pid %d  \n", (uint64_t) MPAPtr,  getpid());
      std::cout.flush();
    }
    release_sem_lock();

    //ABH DBG
    cycleTASELogs(true);

    //END DBG

    char buf [10];
    sprintf(buf,"%d",replayingFromPID);
    std::string fName = std::string("ReplayS2CRecords.") + std::string(buf);
    FILE * s2cFile = fopen(fName.c_str(), "r+");

    if (s2cFile == NULL) {
	printf("FATAL ERROR: Couldn't open replay S2C records file during replay \n");
	fflush(stdout);
    }
    
    s2c_records_replay = readRoundRecordsFromFile(s2cFile, &last_message_verification_time);
    fclose(s2cFile);
    remove(fName.c_str()); //Delete the replay records file

  } else {    //CHILD
    raise(SIGSTOP);

    //ABH DBG
    //kill(parentPID, SIGSTOP);  //Shouldn't we just let parent raise(SIGSTOP)?  Race condition?

    cycleTASELogs(false); 

  }    
}

void multipass_replay_round (void * assignmentBufferPtr, CVAssignment * mpa)  {
  //printf("Entering multipass_replay_round \n");
  print_run_timers();
  
  while(true) {//Is this actually needed?  get_sem_lock() should block until semaphore is available
 
    get_sem_lock();
    if ((round_count < *latestRoundPtr && workerSelfTerminate) || *replayPIDPtr == -1)  {
      if (*replayPIDPtr == -1) {
	printf("replayPIDPtr is -1 \n");
      }
      removeFromQR(PidInQR(getpid()));
      release_sem_lock();
      if (!noLog) {
	printf("Worker %d is in round %d when latest round is %d. Worker exiting. \n", getpid(), round_count, *latestRoundPtr );
	fflush(stdout);
      }
      std::exit(EXIT_SUCCESS);
    }

    
    if (*replayLock != 1  || (PidInQA(*replayPIDPtr) != NULL) || (PidInQR(*replayPIDPtr) != NULL) ||  *((uint8_t *) assignmentBufferPtr) != 0)  {
      release_sem_lock();  //Spin and try again after pending replay fully executes

    } else {
      
      if (*replayLock != 1) {
	printf("IMPORTANT: control debug: Error - replayLock has unexpected value %d \n", *replayLock);
      } else {
	*replayLock = 0;
	if (PidInQA(*replayPIDPtr) != NULL)
	  printf("ERROR: control debug: replay pid is somehow already in QA \n");	
      }


      //Kills worker if latest round is further than the worker's.  This is because
      //we only set up one replay PID, and when the replay for the worker's round happens,
      //we zero-out the buffer.
      if (*replayPIDPtr == 0) {
	release_sem_lock();
	worker_self_term();  
      }
      
      WorkerInfo wi;
      wi.pid = *replayPIDPtr;
      wi.round = round_count;
      wi.pass = pass_count + 1;
      wi.branches = tase_branches;
      wi.parent = getpid();
      addToQA(&wi);  //Move replay child pid into QA
      if ( *((uint8_t *) assignmentBufferPtr) != 0) { //Move latest assignments into shared mem
	printf("ERROR: control debug: see data in assignment buffer when trying to serialize new constraints \n");
	std::cout.flush();
      }
      if (taseDebug){
	printf("Printing assignments BEFORE serialization \n");
	mpa->printAllAssignments(NULL);
      }

      mpa->serializeAssignments(assignmentBufferPtr, multipassAssignmentSize);

      //Write down S2C records here
      char buf [10];
      sprintf(buf,"%d",getpid());
      std::string fName = std::string("ReplayS2CRecords.") + std::string(buf);
      FILE * recvRecordsFile = fopen(fName.c_str(), "w+");
      if (recvRecordsFile == NULL) {
	printf("FATAL ERROR: Couldn't open replay S2C records file \n");
	fflush(stdout);
      }
      writeRecordsToFile(recvRecordsFile, s2c_records, last_message_verification_time);
      
      
      removeFromQR(PidInQR(getpid()));

      *replayPIDPtr= -1;//ABH DBG
      
      release_sem_lock();

      std::exit(EXIT_SUCCESS);
    }
  }
}


void  multipass_reset_round(bool isFirstCall) {

  pass_count = 0;
  if (!isFirstCall) {
    round_count++;
  }
  void * tmp = mmap(NULL, multipassAssignmentSize, PROT_READ|PROT_WRITE, MAP_ANON|MAP_SHARED, -1, 0);
  get_sem_lock();
  MPAPtr = tmp;
  replayPIDPtr = (int *) ((uint64_t) MPAPtr + multipassAssignmentSize - 8);

  *replayPIDPtr = 0;
  replayLock = replayPIDPtr + 1;
  //replayLock = (int *) mmap(NULL, sizeof(int), PROT_READ|PROT_WRITE, MAP_ANON|MAP_SHARED, -1, 0);
  *replayLock = 1;
  release_sem_lock();
}

nextMessageType getNextNetworkMsgType () {
  KTestObject * kto;
  
  //Find next ktest s2c or c2s message
  int index = ktov.playback_index;
  while (true) {
    kto = &(ktov.objects[index]);
    if (strcmp(kto->name, "c2s") == 0)
      return C2S;
    else if (strcmp(kto->name, "s2c") == 0)
      return S2C;
    else 
      index++;
    
    if (index >= ktov.size)
      return NONE;
  }

  //Should never reach this point
  printf("FATAL ERROR in getNextNetworkMsgType \n");
  fflush(stdout);
  std::exit(EXIT_FAILURE);
}
//Function to determine if parent should be assigned true or false path at
//branch during fork.
void guessParentPath (int * childNum, int * parentNum) {

  nextMessageType mt = getNextNetworkMsgType();    
  printf("peekCtr is %d \n", peekCtr);
  if (mt == C2S) {
    //Do nothing
  } else if (mt == S2C) {
    peekCtr++;
    if (peekCtr %3 != 0) {
      printf("Parent stays on True branch \n");
      fflush(stdout);
      *childNum =0;
      *parentNum = 1;
    } else {
      printf("Parent gets False branch \n");
      fflush(stdout);
      *childNum = 1;
      *parentNum = 0;
    }
  }
}


int BFS_dequeue() {
  int tmp  = *BFS_Front_Ptr;
  BFS_Front_Ptr++;
  return tmp;
}

void BFS_enqueue(int pid) {

  if ((uint64_t) BFS_Back_Ptr - (uint64_t) explorer_pid_queue_base > MAX_QUEUE_SIZE) {
    release_sem_lock();
    printf("BFS Queue too large.  Exiting. \n");
    worker_exit();
  }

  *BFS_Back_Ptr = pid;
  BFS_Back_Ptr++;
  
}

int DFS_pop() {
  DFS_Stack_Ptr--;
  return *DFS_Stack_Ptr;
}

void DFS_push(int pid) {
  if ((uint64_t) DFS_Stack_Ptr - (uint64_t) explorer_pid_stack_base > MAX_STACK_SIZE) {
    release_sem_lock();
    printf("DFS stack too large.  Exiting. \n");
    worker_exit();
  }
  
  *DFS_Stack_Ptr= pid;
  DFS_Stack_Ptr++;
}

//Only call this fn when holding the lock!
int get_next_explorer_pid () {
  if (explorationType == BFS) {
    return BFS_dequeue();
  } else if (explorationType == DFS) {
    return DFS_pop();
  } else {
    printf("Error: no exploration type defined \n");
    return 0;
  }
}

void manage_exploration_workers() {
  get_sem_lock();

  if (*explorer_get_next_pid == 1) {
    int PID = get_next_explorer_pid();//Get next pid and remove it from either the queue or stack
    int res = kill(PID, SIGCONT);
    if (res == -1){
      perror("Error during kill sigcont \n");
      printf("Error during kill sigcont \n");
      fflush(stdout);
    }

    *explorer_get_next_pid = 0;
  }

  if (*total_workers == 0 ) {
    release_sem_lock();

    stdout = fdopen(STDOUT_FILENO, "w");
    stdout = fdopen(orig_stdout_fd, "w") ;
    
    printf("\nTASE: Successfully exiting with %d paths explored \n", *total_branches);
    std::exit(EXIT_SUCCESS);
  } else {

    release_sem_lock();
  }
}
