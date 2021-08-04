#ifndef TASE_CONTROL_H
#define TASE_CONTROL_H

void get_sem_lock();
void release_sem_lock();
int initialize_semaphore(int semkey);
void initManagerStructures();
void manage_workers();
int tase_fork(int parentPID, uint64_t initRIP);
void tase_exit();

extern void * ms_base;
extern int ms_size;
extern int * ms_QR_base;
extern int * ms_QR_size_ptr;
extern int * ms_QA_base;
extern int * ms_QA_size_ptr;

#ifndef TASE_OPENSSL
void DFS_push (int pid);
void BFS_enqueue (int pid);
#endif

#endif
