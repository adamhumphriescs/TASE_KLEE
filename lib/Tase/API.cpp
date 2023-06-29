#include "API.h"
#include <cstdlib>
#include <sys/mman.h>
#include <signal.h>
#include <stdio.h>
#include <cstdint>
#include <sys/sem.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <iostream>
#include <sys/prctl.h>

#include "klee/CommandLine.h"

// not actually using this here
// struct WorkerInfo {
//   int pid;
//   int branches;
//   int round;
//   int pass;
//   int parent;
// };
// in addToQA under tase_verification_fork
// do tase_explorer_fork first, matches the other API better

// Actually 2 APIs here -> Explorer and Verifier. Separate them!

struct WorkerGroup {
  pid_t* base;
  pid_t* front_ptr;
  pid_t* back_ptr;
};

extern struct WorkerGroup * Stopped;
extern struct WorkerGroup * Running;

bool* success;

void* ms_base;
size_t ms_size;

int64_t MAX_GROUP_SIZE;


int * total_branches;
int * explorer_get_next_pid;    //Should be 1 or 0 to indicate if it's time to get the next PID

extern int orig_stdout_fd;

int semID; //Global semaphore ID for sync                                                                                      
struct sembuf sem_lock = {0, -1, 0 | SEM_UNDO}; // {sem index, inc/dec, flags}                                                 
struct sembuf sem_unlock = {0, 1, 0 | SEM_UNDO};// SEM_UNDO added to release


extern "C" void cycleTASELogs(bool);
extern bool noLog;

void wait_stopped(pid_t pid) {
  while (true) {
    int status;
    waitpid(pid, &status, WUNTRACED);
    if (WIFSTOPPED(status))
      break;
  }
}

void wait_started(pid_t pid) {
  while (true) {
    int status;
    waitpid(pid, &status, WUNTRACED);
    if (WIFCONTINUED(status))
      break;
  }
}


void wait_killed(pid_t pid) {
  while( true ){
    int status;
    waitpid(pid, &status, WUNTRACED);
    if( WIFEXITED(status) || ( WIFSIGNALED(status) && WTERMSIG(status) == SIGTERM ) )
      break;
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


void get_sem_lock () {
  int res =  semop(semID, &sem_lock, 1);
  if (res == 0) {
    return;
  } else {
    printf("Error getting sem lock \n");
    std::cout.flush();
    perror("Error in get_sem_lock");
    std::exit(EXIT_FAILURE);
  }
}


void release_sem_lock () {
  int res = semop(semID, &sem_unlock, 1);
  if (res == 0) {
    return;
  } else {
    perror("Error in release_sem_lock");
    std::exit(EXIT_FAILURE);
  }
}


bool add_worker(pid_t child, struct WorkerGroup * grp) {
  printf("adding worker\n");
  fflush(stdout);
  if( (grp->back_ptr - grp->front_ptr) > MAX_GROUP_SIZE ) {
    return false;
  }
  
  if( klee::explorationType == BFS ) {
    *grp->back_ptr = child;
    grp->back_ptr++;
  } else {
    *grp->back_ptr = child;
    grp->back_ptr++;
  }

  return true;
}


bool remove_worker(pid_t* worker, struct WorkerGroup* grp) {
  if( grp->back_ptr == grp->front_ptr )
    return false;
     
  if( klee::explorationType == BFS ) {
    *worker = *grp->front_ptr;
    grp->front_ptr++;
  } else {
    *worker = *grp->back_ptr;
    grp->back_ptr--;
  }

  return true;
}


void release_worker(pid_t pid, struct WorkerGroup *grp) {
  for( pid_t* x = grp->front_ptr; x != grp->back_ptr; x++ ) {
    if( *x == pid ) {
      if( x == grp->back_ptr - 1 ) {
	grp->back_ptr--;
      } else {
	memcpy(x, x+1, grp->back_ptr - x - 1);
      }
    }
  }
}



int schedule_worker(struct WorkerGroup * Stopped, struct WorkerGroup * Running) {
  pid_t x;
  if( remove_worker(&x, Stopped) ) {
    if ( add_worker(x, Running) ) {
      printf("Scheduling worker %d\n", (int)x);
      kill(x, SIGCONT);
      //      wait_started(x);
      return x;
    } else {
      if( !add_worker(x, Stopped) ) {
	get_sem_lock();
	destroy_structures(&Stopped, &Running);
	release_sem_lock();
	
	printf("Error scheduling worker.\n");
        fflush(stdout);
	std::exit(EXIT_FAILURE);
      }
    }
  }
  return 0;
}


pid_t fork_child() {
  int child = ::fork();

  if( child == -1 ) {
    printf("FATAL ERROR during forking \n");
    fflush(stdout);
    fprintf(stderr, "ERROR during fork; pid is %d \n", getpid());
    fflush(stderr);
    perror("Fork error \n");
  }

  return child;
}


pid_t do_fork() {
  int child = fork_child();
  setpgid(0, 0);

  if( child != 0 ) {
    if( !noLog ) {
      printf("Parent PID %d forked off child %d\n", getpid(), child);
      fflush(stdout);
    }

    wait_stopped(child);
  } else {
    cycleTASELogs(false);
    raise(SIGSTOP);
  }
  return child;  
}




// this still works fine in Executor.cpp -> Executor::fork, Executor::forkOnPossibleRIPValues
pid_t worker_fork(struct WorkerGroup * Stopped, struct WorkerGroup * Running) {
  get_sem_lock();
  
  *total_branches = *total_branches + 1;
  
  pid_t child = do_fork();

  if( child == 0 ) { // true child
    if( add_worker(child, Stopped) )
      return 1;
    
    destroy_structures(&Stopped, &Running);  // failed to add
    release_sem_lock();
    std::exit(EXIT_FAILURE);
  }

  child = do_fork();

  if( child == 0 ) { // false child
    if( add_worker(child, Stopped) ) 
      return 0;

    destroy_structures(&Stopped, &Running); // failed to add
    release_sem_lock();
    std::exit(EXIT_FAILURE);    
  }
  
  *explorer_get_next_pid = 1;
  release_sem_lock();
  
  std::exit(EXIT_SUCCESS);
}



pid_t initial_fork(struct WorkerGroup * Stopped, struct WorkerGroup * Running) {
  pid_t child = do_fork();
  
  if( child != 0 ) {
    printf("Initial worker forked: %d\n", child);
    fflush(stdout);
    *explorer_get_next_pid = 1;
    add_worker(child, Stopped);
  }
  
  return child;
}

void manage_workers(struct WorkerGroup * Stopped, struct WorkerGroup * Running) {
  while( true ) {
    printf("Stopped: %ld, Running: %ld\n", Stopped->back_ptr - Stopped->front_ptr, Running->back_ptr - Running->front_ptr);
    fflush(stdout);
    get_sem_lock();

    if (*explorer_get_next_pid == 1) {
      if( schedule_worker(Stopped, Running) ) // else?
	*explorer_get_next_pid = 0;
    }

    if ( *success || Stopped->back_ptr == Stopped->front_ptr && Running->back_ptr == Running->front_ptr ) {
      stdout = fdopen(STDOUT_FILENO, "w");
      stdout = fdopen(orig_stdout_fd, "w");

      if( *success ) {
	printf("\nTASE: Successfully exiting with %d paths explored \n", *total_branches);
	destroy_structures(&Stopped, &Running);
	munmap(ms_base, ms_size);
	release_sem_lock();
	std::exit(EXIT_SUCCESS);
      }
    }

    release_sem_lock();
  }
}



void release_child(int signal, siginfo_t *info, void *context) {
  printf("Releasing worker %d\n", info->si_pid);
  fflush(stdout);
  int status;
  waitpid(info->si_pid, &status, 0);
  get_sem_lock();
  release_worker(info->si_pid, Running);
  release_sem_lock();
}


void init_structures(struct WorkerGroup ** Stopped, struct WorkerGroup ** Running) {
  MAX_GROUP_SIZE = 4000;
  initialize_semaphore(getpid());
  
  // shared mem for:
  // total_workers, total_branches, explorer_get_next_pid, Stopped, Running, Stopped/Running contents, success
  ms_size = sizeof(struct WorkerGroup) * 2 + sizeof(int*)*2 + sizeof(pid_t) * MAX_GROUP_SIZE * 2 + sizeof(bool);
  ms_base = mmap(NULL, ms_size, PROT_READ|PROT_WRITE, MAP_ANON|MAP_SHARED, -1, 0);

  *Stopped = (struct WorkerGroup*) ms_base;
  *Running = ((struct WorkerGroup*) ms_base)+1;

  total_branches = (int*) ((char*)ms_base + sizeof(struct WorkerGroup)*2 );
  explorer_get_next_pid = total_branches+1;

  (*Running)->base = (pid_t*)((char*)ms_base + sizeof(struct WorkerGroup)*2 + sizeof(int*)*2);
  (*Stopped)->base = (*Running)->base + MAX_GROUP_SIZE;

  success = (bool*)((*Stopped)->base + MAX_GROUP_SIZE);
  
  (*Running)->front_ptr = (*Running)->base;
  (*Running)->back_ptr = (*Running)->base;
  (*Stopped)->front_ptr = (*Stopped)->base;
  (*Stopped)->back_ptr = (*Stopped)->base;

  *total_branches = 1;
  *explorer_get_next_pid = 0;

  int res = prctl(PR_SET_CHILD_SUBREAPER, 1);
   if (res == -1) {
     perror("Subreaper err ");
     fprintf(stderr, "Exiting due to reaper error in initManagerStructures \n");
     std::exit(EXIT_FAILURE);
   }

  struct sigaction sigact;
  sigact.sa_sigaction = &release_child;
  sigaction(SIGCHLD, &sigact, NULL);  
}


void destroy_structures(struct WorkerGroup ** Stopped, struct WorkerGroup ** Running) {
  printf("Destroying structures\n");
  fflush(stdout);
  
  pid_t* x;
  if( (*Running)->back_ptr != (*Running)->front_ptr ) {
    for( x = (*Running)->back_ptr - 1; x != (*Running)->front_ptr - 1; x-- ) {
      kill(*x, SIGTERM);
    }
  }

  if( (*Stopped)->back_ptr != (*Stopped)->front_ptr ) {
    for( x = (*Stopped)->back_ptr - 1; x != (*Stopped)->front_ptr - 1; x-- ) {
      kill(*x, SIGTERM);
    }
  }
}


void worker_success(struct WorkerGroup * Stopped, struct WorkerGroup * Running) {
  get_sem_lock();
  *success = true;
  release_sem_lock();
  
  std::exit(EXIT_SUCCESS);
}

