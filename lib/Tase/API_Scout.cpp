#include <deque>
#include <algorithm>
#include <memory>
#include <cstring>
#include <sys/prctl.h>

#include "API_Scout.h"
#include "klee/CommandLine.h"

// worker_fork is intended to be called by workers
// all other items from the API are just for the managing process to call

// originator is the process that was cloned, but we use CLONE_PARENT in clone3 to reparent to the manager process
struct WorkerInfo {
  pid_t pid;
  pid_t originator;
  size_t branches;
  bool scout;  // useful for debug
};


// standard signal types
enum class SIGNALS {
		    ABORT,
		    SUCCESS
};


void worker_cleanup(std::deque<WorkerInfo>& workers);

struct WorkerGroup {
  typedef void(*finish_t)(std::deque<WorkerInfo>&);
  
  std::deque<WorkerInfo> workers;
  finish_t cleanup;

  
  WorkerInfo popf() {
    auto x = workers.front();
    workers.pop_front();
    return x;
  }
  
  WorkerInfo popb() {
    auto x = workers.back();
    workers.pop_back();
    return x;
  }

  // void push(WorkerInfo&& w) {
  //   workers.push_back(std::move(w));
  // }

  void push(const WorkerInfo& w) {
    workers.push_back(w);
  }

  bool find(WorkerInfo * r, pid_t pid) {
    for(auto x = workers.begin(); x != workers.end(); x++) {
      if( x->pid == pid ) {
        *r = *x;
	return true;
      }
    }
    return false;
  }
  
  // destructive find
  bool get(WorkerInfo * r, pid_t pid) {
    for(auto x = workers.begin(); x != workers.end(); x++) {
      if( x->pid == pid ) {
	*r = *x;
	workers.erase(x);
	return true;
      }
    }

    return false;
  }

  size_t size() {
    return workers.size();
  }

  bool empty() {
    return workers.empty();
  }

  std::deque<WorkerInfo>::iterator begin() {
    return workers.begin();
  }

  std::deque<WorkerInfo>::iterator end() {
    return workers.end();
  }

  void clear() {
    workers.clear();
  }
					   
  
  WorkerGroup(finish_t cleanup = worker_cleanup)
    : workers()
    , cleanup(cleanup)
  {}

  ~WorkerGroup() {
    cleanup(workers);
  }
};

size_t MAX_RUNNING_WORKERS = 32; // need to add options to klee
size_t MAX_STOPPED_WORKERS = 1024;
WorkerGroup * Stopped;
WorkerGroup * Running;
pid_t manager_pid;
pid_t backup = 0;
pid_t scout = -1;
int sfd;
struct signalfd_siginfo signals[MAX_EVENTS];
int num_signals;

// wait could be interrupted and fail, so keep in the loop
void wait_stopped(pid_t pid) {
  while( true ) {
    int status;
    waitpid(pid, &status, WUNTRACED | __WCLONE );
    if( WIFSTOPPED(status) )
      break;
  }
}

void wait_killed(pid_t pid) {
  int status;
  while( true ) {
    waitpid(pid, &status, WUNTRACED | __WCLONE );
    if( WIFEXITED(status) )
      break;
  }
}

void deathsig() {
  if( getpid() != manager_pid ) {

    if( backup ) {
      sigval y;
      y.sival_int = -1;
      sigqueue(backup, SIGSTD, y);    // scout signaling the backup on exit
    }
    
    sigval x;    
    x.sival_int = static_cast<int>(SIGNALS::ABORT);
    sigqueue(manager_pid, SIGSTD, x); // abort signal to manager
  }
}



void init_structures(WorkerGroup ** Stopped, WorkerGroup ** Running) {
  *Stopped = new WorkerGroup();
  *Running = new WorkerGroup();
  
  if( MAX_RUNNING_WORKERS % 2 != 0 ) // need even number for scout processes
    MAX_RUNNING_WORKERS+=1;
  
  manager_pid = getpid();

  prctl(PR_SET_CHILD_SUBREAPER, 1);
  
  sigset_t mask;
  sigemptyset(&mask);

  //  sigaddset(&mask, SIGCHLD);  // ignore sigchld
  sigaddset(&mask, SIGSTD);   // standard signals
  sigaddset(&mask, SIGSCOUT); // scout signal

  if( sigprocmask(SIG_BLOCK, &mask, NULL) == - 1 ) {
    printf("Could not set sigprocmask\n");
    fflush(stdout);
    exit(1);
  }

  sigemptyset(&mask);
  sigaddset(&mask, SIGSTD); // standard signals
  sigaddset(&mask, SIGSCOUT); // scout signal       

  sfd = signalfd(-1, &mask, 0);
  if( sfd == -1 ) {
    printf("Error creating signalfd\n");
    fflush(stdout);
    exit(1);
  }

  atexit(deathsig);
}
				    

void destroy_structures(WorkerGroup ** Stopped, WorkerGroup ** Running) {
  auto destroy = [](WorkerInfo& x){ kill(x.pid, SIGKILL); wait_killed(x.pid);};
  std::for_each((*Stopped)->begin(), (*Stopped)->end(), destroy);
  std::for_each((*Running)->begin(), (*Running)->end(), destroy);
  (*Stopped)->clear();
  (*Running)->clear();

  delete *Stopped;
  delete *Running;
}


// kill the worker processess and wait on them
void worker_cleanup(std::deque<WorkerInfo>& workers) {
  std::for_each(workers.begin(), workers.end(), [](WorkerInfo& w){
						  kill(w.pid, SIGKILL);
						  wait_killed(w.pid);
						});
}



int schedule_worker(WorkerGroup * Stopped, WorkerGroup * Running) {
  if ( Running->size() < MAX_RUNNING_WORKERS ) {
    WorkerInfo tmp = klee::explorationType == BFS ? Stopped->popf() : Stopped->popb();
    Running->push( tmp );
    kill(tmp.pid, SIGCONT);
    return 1;
  }
  
  return 0;
}


// fork replacement but with reparenting to calling process's parent to allow manager to wait
pid_t call_fork() {
  struct clone_args args = {};
  args.flags = CLONE_PARENT;
  return (pid_t) syscall(SYS_clone3, &args, sizeof(struct clone_args));
}


// We don't use Stopped/Running here since they are only actually available in
// the manager process and not shared mem for this implementation
// but the API has them here
pid_t worker_fork(WorkerGroup * Stopped, WorkerGroup * Running) {
  pid_t child = call_fork();
  if( child == -1 ) {
    printf("Fork failed\n");
    fflush(stdout);
    destroy_structures(&Stopped, &Running);
    exit(1);
  }

  if( child != 0 ) {
    sigval x;
    x.sival_int = *reinterpret_cast<int*>(&child);  // pid_t isn't int but same size
    sigqueue(manager_pid, SIGSTD, x);
  }

  kill(getpid(), SIGSTOP);

  return child;
}


pid_t initial_fork(WorkerGroup * Stopped, WorkerGroup * Running) {
  struct clone_args args = {};
  pid_t pid = (pid_t) syscall(SYS_clone3, &args, sizeof(struct clone_args));;

  if( pid == -1 ) {
    printf("Fork failed\n");
    fflush(stdout);
    destroy_structures(&Stopped, &Running);
    exit(1);
  }
  
  if( pid != 0 ) {
    Stopped->push(WorkerInfo{pid, getpid(), 0, false}); // add initial worker
    wait_stopped(pid);
  } else {    
    kill(getpid(), SIGSTOP);
  }

  return pid;
}



void fetch_signals(WorkerGroup * Stopped, WorkerGroup * Running) {
  int bytes = read( sfd, signals, sizeof(struct signalfd_siginfo) * MAX_EVENTS );
  if ( bytes == 0 ) {
    num_signals = 0;
    return;
  } else if ( bytes == -1 ) {
    printf("Error reading events\n");
    fflush(stdout);
    destroy_structures(&Stopped, &Running);
    exit(1);
  }

  num_signals = bytes / sizeof(struct signalfd_siginfo);
}


void clear_signals() {
  memset(&signals[0], '\0', MAX_EVENTS*sizeof(struct signalfd_siginfo));
}


// where the magic happens
void handle_signals(WorkerGroup * Stopped, WorkerGroup * Running) {
  fetch_signals(Stopped, Running);

  for( int i = 0; i < num_signals; i++ ) {
    WorkerInfo tmp;
    if( signals[i].ssi_signo == SIGSTD ) { // standard signals

      switch( SIGNALS(signals[i].ssi_int) ) {
      case SIGNALS::ABORT:  // scout abort signal also sent to backup on exit, no special case here. Handled in tase/src/tase/common_scout.c
	Running->get(&tmp, signals[i].ssi_pid) || Stopped->get(&tmp, signals[i].ssi_pid);
	
	wait_killed(signals[i].ssi_pid);
	break;
	
      case SIGNALS::SUCCESS: // print something here...
	destroy_structures(&Stopped, &Running);
	break;
	
      default: // fork
	if( Stopped->size() == MAX_STOPPED_WORKERS ) {
	  printf("MAX_STOPPED_WORKERS Exceeded. Execution failed\n");
	  fflush(stdout);
	  destroy_structures(&Stopped, &Running);
	  exit(1);
	}
	
	Running->find(&tmp, signals[i].ssi_pid) || Stopped->find(&tmp, signals[i].ssi_pid);
	Stopped->push({*reinterpret_cast<pid_t*>(&signals[i].ssi_int), (pid_t)signals[i].ssi_pid, tmp.branches+1, false});

	wait_stopped(tmp.pid);
	wait_stopped(*reinterpret_cast<pid_t*>(&signals[i].ssi_int));
	break;
      }
      
    } else if ( signals[i].ssi_signo == SIGSCOUT ) {
      // scout fork. Scout goes into Running. Sent by originator == new scout.
      // corresponding code for the forking is in tase/src/tase/common_scout.c
      // should set 'pid_t scout' to be the scout process ID in the new backup process,
      // and 0 in the new scout process.
      
      pid_t scout_pid = signals[i].ssi_pid;
      pid_t backup_pid = *reinterpret_cast<pid_t*>(&signals[i].ssi_int);
      size_t branches = 0;
      
      //      wait_stopped(backup_pid); moved below so the scout can go ahead when applicable instead of waiting on the backup to start
      //      wait_stopped(scout_pid);
      
      Running->get(&tmp, scout_pid) || Stopped->get(&tmp, scout_pid);

      tmp.scout = true; // backup becomes new scout
      branches = tmp.branches;

      
      if( Running->size()+1 < MAX_RUNNING_WORKERS ){
	Running->push(tmp);
	Running->push({backup_pid, scout_pid, branches, false});

	wait_stopped(scout_pid);
	kill(scout_pid, SIGCONT);
	
	wait_stopped(backup_pid);
	kill(backup_pid, SIGCONT);
	
      } else if( Stopped->size()+1 < MAX_STOPPED_WORKERS ) {
        Stopped->push(tmp);
	Stopped->push({backup_pid, scout_pid, branches, false});
	wait_stopped(scout_pid);
	wait_stopped(backup_pid);
	
      } else {
	printf("%s Exceeded. Execution failed.\n", Running->size()+1 >= MAX_RUNNING_WORKERS ? "MAX_RUNNING_WORKERS" : "MAX_STOPPED_WORKERS");
	fflush(stdout);
	destroy_structures(&Stopped, &Running);
	exit(1);
      }
    }
  }
  
  clear_signals();
}


void manage_workers(WorkerGroup * Stopped, WorkerGroup * Running) {
  while( !Stopped->empty() || !Running->empty() ) {

    while( !Stopped->empty() && schedule_worker(Stopped, Running) ){}

    handle_signals(Stopped, Running);
  }
}



void worker_success(WorkerGroup * Stopped, WorkerGroup * Running) {
  sigval x;
  x.sival_int = static_cast<int>(SIGNALS::SUCCESS);
  sigqueue(manager_pid, SIGSTD, x);
}
