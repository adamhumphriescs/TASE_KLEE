//===-- KTest.cpp ---------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Internal/ADT/KTest.h"
#ifdef TASE_OPENSSL
#include "../../../openssl/e_os.h"
#endif
#include "../../../test/proj_defs.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>
#include <sys/socket.h>
#include <sys/time.h>

//Changed for TASE
//Ugly kludge to pass in these two bools derived from command line args.
//Ideally we should just put these in include/klee/CommandLine.h but
//that would require changing the cmake structure of lib/Basic to have
//llvm dependencies.
extern bool taseDebug;
extern bool dropS2C;

#define KTEST_VERSION 4
#define KTEST_MAGIC_SIZE 5
#define KTEST_MAGIC "KTEST"
#include <cstdlib>
#include <iostream>
#define KTEST_DEBUG 0

extern void worker_exit();

extern "C" int RAND_pseudo_bytes (unsigned char *buf, int num);

extern "C" int RAND_bytes (unsigned char *buf, int num);


extern bool enableMultipass;
extern uint64_t interpCtr;

extern int round_count;

// for compatibility reasons
#define BOUT_MAGIC "BOUT\n"

/***/

// override inline assembly version of FD_ZERO from                               
// /usr/include/x86_64-linux-gnu/bits/select.h                                    
#ifdef FD_ZERO
#undef FD_ZERO
#endif
#define FD_ZERO(p)        memset((char *)(p), 0, sizeof(*(p)))

static int read_uint32(FILE *f, unsigned *value_out) {
  unsigned char data[4];
  if (fread(data, 4, 1, f)!=1)
    return 0;
  *value_out = (((((data[0]<<8) + data[1])<<8) + data[2])<<8) + data[3];
  return 1;
}

static int write_uint32(FILE *f, unsigned value) {
  unsigned char data[4];
  data[0] = value>>24;
  data[1] = value>>16;
  data[2] = value>> 8;
  data[3] = value>> 0;
  return fwrite(data, 1, 4, f)==4;
}

static int read_uint64(FILE *f, uint64_t *value_out) {
  unsigned char data[8];
  if (fread(data, 8, 1, f)!=1)
    return 0;
  *value_out = (((((((((((( (data[0]<<8) + data[1])<<8) + data[2])<<8) + data[3])<<8) + data[4])<<8) + data[5])<<8) + data[6])<<8) + data[7];
  return 1;
}

static int write_uint64(FILE *f, uint64_t value) {
  unsigned char data[8];
  data[0] = value>>56;
  data[1] = value>>48;
  data[2] = value>>40;
  data[3] = value>>32;
  data[4] = value>>24;
  data[5] = value>>16;
  data[6] = value>> 8;
  data[7] = value>> 0;
  return fwrite(data, 1, 8, f)==8;
}

static int read_string(FILE *f, char **value_out) {
  unsigned len;
  if (!read_uint32(f, &len))
    return 0;
  *value_out = (char*) malloc(len+1);
  if (!*value_out)
    return 0;
  if (fread(*value_out, len, 1, f)!=1)
    return 0;
  (*value_out)[len] = 0;
  return 1;
}

static int write_string(FILE *f, const char *value) {
  unsigned len = strlen(value);
  if (!write_uint32(f, len))
    return 0;
  if (fwrite(value, len, 1, f)!=1)
    return 0;
  return 1;
}

/***/


unsigned kTest_getCurrentVersion() {
  return KTEST_VERSION;
}


static int kTest_checkHeader(FILE *f) {
  char header[KTEST_MAGIC_SIZE];
  if (fread(header, KTEST_MAGIC_SIZE, 1, f)!=1)
    return 0;
  if (memcmp(header, KTEST_MAGIC, KTEST_MAGIC_SIZE) &&
      memcmp(header, BOUT_MAGIC, KTEST_MAGIC_SIZE))
    return 0;
  return 1;
}

int kTest_isKTestFile(const char *path) {
  FILE *f = fopen(path, "rb");
  int res;

  if (!f)
    return 0;
  res = kTest_checkHeader(f);
  fclose(f);
  
  return res;
}

KTest *kTest_fromFile(const char *path) {
  FILE *f = fopen(path, "rb");
  KTest *res = 0;
  unsigned i, version;

  if (!f) 
    goto error;
  if (!kTest_checkHeader(f)) 
    goto error;

  res = (KTest*) calloc(1, sizeof(*res));
  if (!res) 
    goto error;

  if (!read_uint32(f, &version)) 
    goto error;
  
  if (version > kTest_getCurrentVersion())
    goto error;

  res->version = version;

  if (!read_uint32(f, &res->numArgs)) 
    goto error;
  res->args = (char**) calloc(res->numArgs, sizeof(*res->args));
  if (!res->args) 
    goto error;
  
  for (i=0; i<res->numArgs; i++)
    if (!read_string(f, &res->args[i]))
      goto error;

  if (version >= 2) {
    if (!read_uint32(f, &res->symArgvs)) 
      goto error;
    if (!read_uint32(f, &res->symArgvLen)) 
      goto error;
  }

  if (!read_uint32(f, &res->numObjects))
    goto error;
  res->objects = (KTestObject*) calloc(res->numObjects, sizeof(*res->objects));
  if (!res->objects)
    goto error;
  for (i=0; i<res->numObjects; i++) {
    KTestObject *o = &res->objects[i];
    if (!read_string(f, &o->name))
      goto error;
    if (res->version >= 4) { // Cliver-specific version 4
      if (!read_uint64(f, (uint64_t*)&o->timestamp.tv_sec))
	goto error;
      if (!read_uint64(f, (uint64_t*)&o->timestamp.tv_usec))
	goto error;
    }
    if (!read_uint32(f, &o->numBytes))
      goto error;
    o->bytes = (unsigned char*) malloc(o->numBytes);
    if (o->numBytes && fread(o->bytes, o->numBytes, 1, f)!=1)
      goto error;
  }

  fclose(f);

  return res;
 error:
  if (res) {
    if (res->args) {
      for (i=0; i<res->numArgs; i++)
        if (res->args[i])
          free(res->args[i]);
      free(res->args);
    }
    if (res->objects) {
      for (i=0; i<res->numObjects; i++) {
        KTestObject *bo = &res->objects[i];
        if (bo->name)
          free(bo->name);
        if (bo->bytes)
          free(bo->bytes);
      }
      free(res->objects);
    }
    free(res);
  }

  if (f) fclose(f);

  return 0;
}


int kTest_toFile(KTest *bo, const char *path) {
  FILE *f = fopen(path, "wb");
  unsigned i;

  if (!f) 
    goto error;
  if (fwrite(KTEST_MAGIC, strlen(KTEST_MAGIC), 1, f)!=1)
    goto error;
  if (!write_uint32(f, KTEST_VERSION))
    goto error;
      
  if (!write_uint32(f, bo->numArgs))
    goto error;
  for (i=0; i<bo->numArgs; i++) {
    if (!write_string(f, bo->args[i]))
      goto error;
  }

  if (!write_uint32(f, bo->symArgvs))
    goto error;
  if (!write_uint32(f, bo->symArgvLen))
    goto error;
  
  if (!write_uint32(f, bo->numObjects))
    goto error;
  for (i=0; i<bo->numObjects; i++) {
    KTestObject *o = &bo->objects[i];
    if (!write_string(f, o->name))
      goto error;
    if (!write_uint32(f, o->numBytes))
      goto error;
    if (fwrite(o->bytes, o->numBytes, 1, f)!=1)
      goto error;
  }

  fclose(f);

  return 1;
 error:
  if (f) fclose(f);
  
  return 0;
}

unsigned kTest_numBytes(KTest *bo) {
  unsigned i, res = 0;
  for (i=0; i<bo->numObjects; i++)
    res += bo->objects[i].numBytes;
  return res;
}

void kTest_free(KTest *bo) {
  unsigned i;
  for (i=0; i<bo->numArgs; i++)
    free(bo->args[i]);
  free(bo->args);
  for (i=0; i<bo->numObjects; i++) {
    free(bo->objects[i].name);
    free(bo->objects[i].bytes);
  }
  free(bo->objects);
  free(bo);
}


static void print_fd_set(int nfds, fd_set *fds) {
  int i;
  for (i = 0; i < nfds; i++) {
    printf(" %d", FD_ISSET(i, fds));
  }
  printf("\n");
  fflush(stdout);
}

enum { CLIENT_TO_SERVER=0, SERVER_TO_CLIENT, RNG, PRNG, TIME, STDIN, SELECT,
       MASTER_SECRET };
char* ktest_object_names[] = {
  "c2s", "s2c", "rng", "prng", "time", "stdin", "select", "master_secret"
};


KTestObjectVector ktov;
static enum kTestMode ktest_mode = KTEST_NONE;
static const char *ktest_output_file = "s_client.ktest";
static const char *ktest_network_file = "s_client.net.ktest";
static int ktest_sockfd = -1; // descriptor of the socket we're capturing


// KTOV = "KTestObjectVector"
static void KTOV_init(KTestObjectVector *ov) {
  memset(ov, 0, sizeof(*ov));
}

static void KTOV_done(KTestObjectVector *ov) {
  if (ov && (ov->objects)) {
    int i;
    for (i = 0; i < ov->size; i++) {
      free(ov->objects[i].name);
      if (ov->objects[i].bytes != NULL) {
	free(ov->objects[i].bytes);
      }
    }
    free(ov->objects);
  }
  memset(ov, 0, sizeof(*ov));
}

static void KTOV_check_mem(KTestObjectVector *ov) {
  if (ov->size + 1 > ov->capacity) {
    size_t new_capacity = (ov->size + 1)*2;
    ov->objects = (KTestObject*) realloc(ov->objects,
					 sizeof(KTestObject) * new_capacity);
    if (!ov->objects) {
      perror("KTOV_check_mem error");
      exit(1);
    }
    ov->capacity = new_capacity;
  }
}

static void timeval2str(char *out, int outlen, const struct timeval *tv) {
  time_t nowtime;
  struct tm *nowtm;
  char tmbuf[64];

  nowtime = tv->tv_sec;
  nowtm = localtime(&nowtime);
  strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
  snprintf(out, outlen, "%s.%06ld", tmbuf, tv->tv_usec);
}

// Print hex and ascii side-by-side
static void KTO_print(FILE *f, const KTestObject *o) {
  unsigned int i, j;
  const unsigned int WIDTH = 16;
  char timebuf[64];

  timeval2str(timebuf, sizeof(timebuf), &o->timestamp);
  fprintf(f, "%s | ", timebuf);
  fprintf(f, "%s [%u]\n", o->name, o->numBytes);
  for (i = 0; WIDTH*i <  o->numBytes; i++) {
    for (j = 0; j < 16 && WIDTH*i+j < o->numBytes; j++) {
      fprintf(f, " %2.2x", o->bytes[WIDTH*i+j]);
    }
    for (; j < 17; j++) {
      fprintf(f, "   ");
    }
    for (j = 0; j < 16 && WIDTH*i+j < o->numBytes; j++) {
      unsigned char c = o->bytes[WIDTH*i+j];
      fprintf(f, "%c", isprint(c)?c:'.');
    }
    fprintf(f, "\n");
  }
  fprintf(f, "\n");
}


// Deep copy of KTestObject
static void KTO_deepcopy(KTestObject *dest, KTestObject *src) {
  dest->name = strdup(src->name);
  dest->timestamp = src->timestamp;
  dest->numBytes = src->numBytes;
  dest->bytes = (unsigned char*)malloc(sizeof(unsigned char)*src->numBytes);
  memcpy(dest->bytes, src->bytes, src->numBytes);
}

static void KTOV_print(FILE *f, const KTestObjectVector *ov) {
  int i;
  fprintf(f, "KTestObjectVector of size %d and capacity %d:\n\n",
	  ov->size, ov->capacity);
  for (i = 0; i < ov->size; i++) {
    fprintf(f, "#%d: ", i);
    KTO_print(f, &ov->objects[i]);
  }
}

static void KTOV_append(KTestObjectVector *ov,
			const char *name,
			int num_bytes,
			const void *bytes)
{
  int i;
  assert(ov != NULL);
  assert(name != NULL);
  assert(num_bytes == 0 || bytes != NULL);
  i = ov->size;
  KTOV_check_mem(ov); // allocate more memory if necessary
  ov->objects[i].name = strdup(name);
  ov->objects[i].numBytes = num_bytes;
  ov->objects[i].bytes = NULL;
  gettimeofday(&ov->objects[i].timestamp, NULL);
  if (num_bytes > 0) {
      ov->objects[i].bytes =
          (unsigned char*)malloc(sizeof(unsigned char)*num_bytes);
      memcpy(ov->objects[i].bytes, bytes, num_bytes);
  }
  ov->size++;
  // KTO_print(stdout, &ov->objects[i]);
}

KTestObject* KTOV_next_object(KTestObjectVector *ov, const char *name)
{

  
  if (ov->playback_index >= ov->size) {
    printf("ERROR: ktest playback -- no more recorded events \n");
    fflush(stdout);
    fprintf(stderr, "ERROR: ktest playback %s - no more recorded events", name);
    printf("IMPORTANT:control debug: Worker %d exiting due to no more log entries \n",getpid());
    
    fflush(stdout);
    exit(2);
  }

  KTestObject *o = &ov->objects[ov->playback_index];

  /*Hack to make us skip to the next c2s or s2c record for the old gmail traces
    for which we may not have .net.ktest files readily available */

  if (enableMultipass) {

    while (true) {
      
      if (strcmp(o->name,"c2s") != 0 &&  (strcmp (o->name, "s2c") != 0 || (dropS2C && round_count >= 3) ) ) {
	if (taseDebug) {
	  printf("skipping record of type %s at index %d \n", o->name, ov->playback_index);
	}
	ov->playback_index++;
	o = &ov->objects[ov->playback_index];
      } else
	break;
    }
  }
  if (strcmp(o->name, name) != 0) {
    printf("ERROR: ktest playback needed '%s', but recording had '%s'\n",
	   name, o->name);
    fflush(stdout);
    printf("IMPORTANT:control debug: Worker %d exiting due to mismatch in log object \n",getpid());
    
    fflush(stdout);
    worker_exit();
    exit(2);
  }

  ov->playback_index++;
  if (taseDebug) {
    printf("Returning from ktov_next_object \n");
    fflush(stdout);
  }
  return o;
}


 int ktest_connect_tase(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{

  if(taseDebug) {
    printf("Entering ktest_connect at interpCtr %lu with sockfd %d \n", interpCtr, sockfd);
    fflush(stdout);
  }
  if (ktest_mode == KTEST_NONE) { // passthrough
    return connect(sockfd, addr, addrlen);
  }
  else if (ktest_mode == KTEST_RECORD) {
    int ret;
    ktest_sockfd = sockfd; // record the socket descriptor of interest
    ret = connect(sockfd, addr, addrlen);
    if (KTEST_DEBUG) {
      printf("connect() called on socket for TLS traffic (%d)\n", sockfd);
    }
    return ret;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    ktest_sockfd = sockfd; // record the socket descriptor of interest
    if (KTEST_DEBUG) {
      printf("connect() called on socket for TLS traffic (%d)\n", sockfd);
    }
    return 0; // assume success
  }
  else {
    perror("ktest_connect error - should never get here");
    exit(4);
  }
}

/**
 * Note that ktest_select playback is slightly brittle in that it
 * assumes that the only socket descriptors we care about are
 * stdin(0), stdout(1), stderr(2), and ktest_sockfd (this is allowed
 * to differ between recording and playback).  If this assumption is
 * not true, then playback could break.
 */
 int ktest_select_tase(int nfds, fd_set *readfds, fd_set *writefds,
		 fd_set *exceptfds, struct timeval *timeout)
{
  printf("Entering ktest_select at interpCtr %lu \n", interpCtr);
  if (ktest_mode == KTEST_NONE) { // passthrough
    return select(nfds, readfds, writefds, exceptfds, timeout);
  }
  else if (ktest_mode == KTEST_RECORD) {
    // select input/output is stored as ASCII text in the format:
    // "sockfd 3 nfds 4 inR 1001 inW 0000 ret 1 outR 1000 outW 0000"
    //    sockfd - just for reference, not part of the select call
    //    nfds - number of active fds in fdset, usually sockfd+1
    //    inR - readfds input value
    //    inW - writefds input value
    //    ret - return value from select()
    //    outR - readfds output value
    //    outW - writefds output value

    int ret, i;
    unsigned int size = 4*nfds + 40 /*text*/ + 3*4 /*3 fd's*/ + 1 /*null*/;
    char *record = (char *)calloc(size, sizeof(char));
    unsigned int pos = 0;

    if (KTEST_DEBUG) {
      printf("\n");
      printf("IN readfds  = ");
      print_fd_set(nfds, readfds);
      printf("IN writefds = ");
      print_fd_set(nfds, writefds);
      fflush(stdout);
    }

    pos += snprintf(&record[pos], size-pos,
		    "sockfd %d nfds %d inR ", ktest_sockfd, nfds);
    for (i = 0; i < nfds; i++) {
      pos += snprintf(&record[pos], size-pos, "%d", FD_ISSET(i, readfds));
    }
    pos += snprintf(&record[pos], size-pos, " inW ");
    for (i = 0; i < nfds; i++) {
      pos += snprintf(&record[pos], size-pos, "%d", FD_ISSET(i, writefds));
    }

    ret = select(nfds, readfds, writefds, exceptfds, timeout);

    if (KTEST_DEBUG) {
      printf("Select returned %d (sockfd = %d)\n", ret, ktest_sockfd);
      printf("OUT readfds   = ");
      print_fd_set(nfds, readfds);
      printf("OUT writefds  = ");
      print_fd_set(nfds, writefds);
      printf("\n");
      fflush(stdout);
    }

    pos += snprintf(&record[pos], size-pos, " ret %d outR ", ret);
    for (i = 0; i < nfds; i++) {
      pos += snprintf(&record[pos], size-pos, "%d", FD_ISSET(i, readfds));
    }
    pos += snprintf(&record[pos], size-pos, " outW ");
    for (i = 0; i < nfds; i++) {
      pos += snprintf(&record[pos], size-pos, "%d", FD_ISSET(i, writefds));
    }

    record[size-1] = '\0'; // just in case we ran out of room.
    KTOV_append(&ktov, ktest_object_names[SELECT], strlen(record)+1, record);
    free(record);
    return ret;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {

    if (KTEST_DEBUG) {
      printf("\n KTEST: Input readfds and writefds are:  \n");
      printf("IN readfds  = ");
      print_fd_set(nfds, readfds);
      printf("IN writefds = ");
      print_fd_set(nfds, writefds);
      fflush(stdout);
    }

    
    KTestObject *o = KTOV_next_object(&ktov, ktest_object_names[SELECT]);
    
    // Make sure we have included the socket for TLS traffic
    assert(ktest_sockfd < nfds);

    // Parse the recorded select input/output.
    char *recorded_select = strdup((const char*)o->bytes);
    char *item;
    fd_set in_readfds, in_writefds, out_readfds, out_writefds;
    int i, ret, recorded_sockfd, recorded_nfds;
    
    FD_ZERO(&in_readfds);  // input to select
    FD_ZERO(&in_writefds); // input to select
    FD_ZERO(&out_readfds); // output of select
    FD_ZERO(&out_writefds);// output of select
    
    assert(strcmp(strtok(recorded_select, " "), "sockfd") == 0);
    strtok(recorded_select, " ");
    
    recorded_sockfd = atoi(strtok(NULL, " ")); // socket for TLS traffic

    assert(strcmp(strtok(NULL, " "), "nfds") == 0);
    strtok(NULL, " ");
    
    recorded_nfds = atoi(strtok(NULL, " "));

    assert(strcmp(strtok(NULL, " "), "inR") == 0);
    strtok(NULL, " ");

    
    item = strtok(NULL, " ");

    assert(strlen(item) == recorded_nfds);
    
    for (i = 0; i < recorded_nfds; i++) {
      if (item[i] == '1') {
	FD_SET(i, &in_readfds);
      }
    }

    
    assert(strcmp(strtok(NULL, " "), "inW") == 0);
    strtok(NULL, " ");
    

    item = strtok(NULL, " ");

    assert(strlen(item) == recorded_nfds);
    
    for (i = 0; i < recorded_nfds; i++) {
      if (item[i] == '1') {
	FD_SET(i, &in_writefds);
      }
    }

    
    assert(strcmp(strtok(NULL, " "), "ret") == 0);
    strtok(NULL, " ");

    
    ret = atoi(strtok(NULL, " "));
    
    assert(strcmp(strtok(NULL, " "), "outR") == 0);
    strtok(NULL, " "); //Replace for assert

    
    item = strtok(NULL, " ");

    
    assert(strlen(item) == recorded_nfds);
    for (i = 0; i < recorded_nfds; i++) {
      if (item[i] == '1') {
	FD_SET(i, &out_readfds);
      }
    }
    
    
    assert(strcmp(strtok(NULL, " "), "outW") == 0);
    strtok(NULL, " ");

    item = strtok(NULL, " ");
    assert(strlen(item) == recorded_nfds);
    for (i = 0; i < recorded_nfds; i++) {
      if (item[i] == '1') {
	FD_SET(i, &out_writefds);
      }
    }
    
    free(recorded_select);
    
    if (KTEST_DEBUG) {
      printf("SELECT playback (recorded_nfds = %d, actual_nfds = %d):\n",
	     recorded_nfds, nfds);
      printf("  inR: ");
      print_fd_set(recorded_nfds, &in_readfds);
      printf("  inW: ");
      print_fd_set(recorded_nfds, &in_writefds);
      printf("  outR:");
      print_fd_set(recorded_nfds, &out_readfds);
      printf("  outW:");
      print_fd_set(recorded_nfds, &out_writefds);
      printf("  ret = %d\n", ret);

    }

    // Copy recorded data to the final output fd_sets.
    FD_ZERO(readfds);
    FD_ZERO(writefds);
    int active_fd_count = 0;
    // stdin(0), stdout(1), stderr(2)
    
    for (i = 0; i < 3; i++) {
      if (FD_ISSET(i, &out_readfds)) {
	FD_SET(i, readfds);
	active_fd_count++;
      }
      if (FD_ISSET(i, &out_writefds)) {
	FD_SET(i, writefds);
	active_fd_count++;
      }
    }
    
    // TLS socket (nfds-1)
    if (FD_ISSET(recorded_sockfd, &out_readfds)) {
      FD_SET(ktest_sockfd, readfds);
      active_fd_count++;
    }
    if (FD_ISSET(recorded_sockfd, &out_writefds)) {
      FD_SET(ktest_sockfd, writefds);
      active_fd_count++;
    }
    assert(active_fd_count == ret); // Did we miss anything?
    if (active_fd_count != ret) {
      printf( "ERROR: active_fd_count %d != ret %d in ktest_select \n", active_fd_count, ret);
    }

    if (KTEST_DEBUG) {
      printf("\n KTEST: At end of ktest_select final values are: \n");
      printf("OUT readfds  = ");
      print_fd_set(nfds, readfds);
      printf("OUT writefds = ");
      print_fd_set(nfds, writefds);
      fflush(stdout);
    }
    
    return ret;
  }
  else {
    perror("ktest_select error - should never get here");
    exit(4);
  }
}

ssize_t ktest_writesocket_tase(int fd, const void *buf, size_t count)
{

  #ifdef TASE_OPENSSL
  if (ktest_mode == KTEST_NONE) { // passthrough
    return writesocket(fd, buf, count);
  }
  else if (ktest_mode == KTEST_RECORD) {
    ssize_t num_bytes = writesocket(fd, buf, count);
    if (num_bytes > 0) {
      KTOV_append(&ktov, ktest_object_names[CLIENT_TO_SERVER], num_bytes, buf);
    } else if (num_bytes < 0) {
      perror("ktest_writesocket error");
      exit(1);
    }
    return num_bytes;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {

    
    
    KTestObject *o = KTOV_next_object(&ktov,
				      ktest_object_names[CLIENT_TO_SERVER]);

    if (KTEST_DEBUG) {
      int i;
      printf("Attempting to write playback [%d]", count);
      for (i = 0; i < count; i++) {
	printf(" %2.2x", ((unsigned char*)buf)[i]);
      }
      printf("\n");
    }
    
    if (KTEST_DEBUG) {
      int i;
      printf("writesocket playback [%d]", o->numBytes);
      for (i = 0; i < o->numBytes; i++) {
	printf(" %2.2x", ((unsigned char*) o->bytes)[i]);
      }
      printf("\n");
    }
    
    if (o->numBytes > count) {
      fprintf(stderr,
	            "ktest_writesocket playback error: %zu bytes of input, "
	      "%d bytes recorded", count, o->numBytes);

      printf("ktest_writesocket playback error: %zu bytes of input, %d bytes recorded \n ", count, o->numBytes);
      fflush(stdout);
      std::exit(EXIT_FAILURE);
      // exit(2);
    }
    // Since this is a write, compare for equality.
    
    if (o->numBytes > 0 && memcmp(buf, o->bytes, o->numBytes) != 0) {
      fprintf(stderr, "WARNING: ktest_writesocket playback - data mismatch\n");
      printf("WARNING: ktet_writesocket playback - data mismatch \n");
      fflush(stdout);
      std::exit(EXIT_FAILURE);
      
    }
    if (KTEST_DEBUG) {
      int i;
      printf("writesocket playback [%d]", o->numBytes);
      for (i = 0; i < o->numBytes; i++) {
	printf(" %2.2x", ((unsigned char*)buf)[i]);
      }
      printf("\n");
    }
    
    
    return o->numBytes; //Todo -- make sure this is ok
  }
  else {
    perror("ktest_writesocket coding error - should never get here");
    exit(4);
  }

  #else
  printf("OpenSSL writesocket behavior disabled \n");
  return 0;
  #endif
  
}

 ssize_t ktest_readsocket_tase(int fd, void *buf, size_t count)
{

  printf("Entering ktest_readsocket \n");

  #ifdef TASE_OPENSSL
  if (ktest_mode == KTEST_NONE) { // passthrough
    return readsocket(fd, buf, count);
  }
  else if (ktest_mode == KTEST_RECORD) {
    ssize_t num_bytes = readsocket(fd, buf, count);
    if (num_bytes > 0) {
      KTOV_append(&ktov, ktest_object_names[SERVER_TO_CLIENT], num_bytes, buf);
    } else if (num_bytes < 0) {
      perror("ktest_readsocket error");
      exit(1);
    }
    return num_bytes;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    //printf("Entered ktest_playback branch \n");
    //fflush(stdout);
    KTestObject *o = KTOV_next_object(&ktov,
				      ktest_object_names[SERVER_TO_CLIENT]);
    //printf("Called KTOV_next_object \n");
    //fflush(stdout);
    if (o->numBytes > count) {
      printf("Error in ktest_readsocket size \n");
      fflush(stdout);
      fprintf(stderr,
	            "ktest_readsocket playback error: %zu byte destination buffer, "
	      "%d bytes recorded", count, o->numBytes);
      exit(2);
    }
    // Read recorded data into buffer
    memcpy(buf, o->bytes, o->numBytes);
    if (KTEST_DEBUG) {
      int i;
      printf("readsocket playback [%d]", o->numBytes);
      for (i = 0; i < o->numBytes; i++) {
	printf(" %2.2x", ((unsigned char*)buf)[i]);
      }
      printf("\n");
    }
    //fflush(stdout);
    return o->numBytes;
  }
  else {
    perror("ktest_readsocket coding error - should never get here");
    exit(4);
  }
  #else
  printf("OpenSSL readsocket behavior disabled \n");
  return 0;
  #endif
}

 int ktest_raw_read_stdin_tase(void *buf,int siz)
{

  
  if (ktest_mode == KTEST_NONE) {
    return read(fileno(stdin), buf, siz);
  }
  else if (ktest_mode == KTEST_RECORD) {
    int ret;
    ret = read(fileno(stdin), buf, siz); // might return 0 (EOF)
    KTOV_append(&ktov, ktest_object_names[STDIN], ret, buf);
    return ret;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    KTestObject *o = KTOV_next_object(&ktov, ktest_object_names[STDIN]);
    if (o->numBytes > siz) {
      fprintf(stderr,
	            "ktest_raw_read_stdin playback error: "
	      "%d byte destination buffer, %d bytes recorded",
	      siz, o->numBytes);
      exit(2);
    }
    // Read recorded data into buffer
    memcpy(buf, o->bytes, o->numBytes);
    if (KTEST_DEBUG) {
      int i;
      printf("raw_read_stdin playback [%d]", o->numBytes);
      for (i = 0; i < o->numBytes; i++) {
	printf(" %2.2x", ((unsigned char*)buf)[i]);
      }
      printf("\n");
    }
    return o->numBytes;
  }
  else {
    perror("ktest_raw_read_stdin coding error - should never get here");
    exit(4);
  }
}

 time_t ktest_time_tase(time_t *t)
{
  if (ktest_mode == KTEST_NONE) {
    return time(t);
  }
  else if (ktest_mode == KTEST_RECORD) {
    time_t ret = time(t);
    KTOV_append(&ktov, ktest_object_names[TIME], sizeof(ret), &ret);
    return ret;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    perror("ktest_time playback not implemented yet");
    exit(2);
  }
  else {
    perror("ktest_time coding error - should never get here");
    exit(4);
  }
}

 int ktest_RAND_bytes_tase(unsigned char *buf, int num)
{
  #ifdef TASE_OPENSSL
  
  if (ktest_mode == KTEST_NONE) {
    return RAND_bytes(buf, num);
  }
  else if (ktest_mode == KTEST_RECORD) {
    int ret = RAND_bytes(buf, num);
    if (KTEST_DEBUG) {
      printf("RAND_bytes returned %d\n", ret);
    }
    KTOV_append(&ktov, ktest_object_names[RNG], num, buf);
    return ret;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    KTestObject *o = KTOV_next_object(&ktov, ktest_object_names[RNG]);
    if (o->numBytes != num) {
      fprintf(stderr,
	            "ktest_RAND_bytes playback error: %d bytes requested, "
	      "%d bytes recorded", num, o->numBytes);
      exit(2);
    }
    memcpy(buf, o->bytes, num);
    if (KTEST_DEBUG) {
      int i;
      printf("RAND_bytes playback [%d]", num);
      for (i = 0; i < num; i++) {
	printf(" %2.2x", buf[i]);
      }
      printf("\n");
    }
    return 1; // success
  }
  else {
    perror("ktest_RAND_bytes coding error - should never get here");
    exit(4);
  }
  #endif
  return 0;
}

int ktest_RAND_pseudo_bytes_tase(unsigned char *buf, int num)
{

  #ifdef TASE_OPENSSL
  if (ktest_mode == KTEST_NONE) {
    return RAND_pseudo_bytes(buf, num);
  }
  else if (ktest_mode == KTEST_RECORD) {
    int ret = RAND_pseudo_bytes(buf, num);
    KTOV_append(&ktov, ktest_object_names[PRNG], num, buf);
    if (KTEST_DEBUG) {
      printf("RAND_pseudo_bytes returned %d\n", ret);
    }
    return ret;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    KTestObject *o = KTOV_next_object(&ktov, ktest_object_names[PRNG]);
    if (o->numBytes != num) {
      fprintf(stderr,
	            "ktest_RAND_pseudo_bytes playback error: %d bytes requested, "
	      "%d bytes recorded", num, o->numBytes);
      exit(2);
    }
    memcpy(buf, o->bytes, num);
    if (KTEST_DEBUG) {
      int i;
      printf("RAND_pseudo_bytes playback [%d]", num);
      for (i = 0; i < num; i++) {
	printf(" %2.2x", buf[i]);
      }
      printf("\n");
    }
    return 1; // 1 = success. WARNING: might return 0 if not crypto-strong
  }
  else {
    perror("ktest_RAND_pseudo_bytes coding error - should never get here");
    exit(4);
  }
  #endif  
  return 0;
}

 void ktest_master_secret_tase(unsigned char *ms, int len) {

  
  if (ktest_mode == KTEST_NONE) {
    return;
  }
  else if (ktest_mode == KTEST_RECORD) {
    KTOV_append(&ktov, ktest_object_names[MASTER_SECRET], len, ms);
    if (KTEST_DEBUG) {
      int i;
      printf("master_secret recorded [%d]", len);
      for (i = 0; i < len; i++) {
	printf(" %2.2x", ms[i]);
      }
      printf("\n");
    }
    return;
  }
  else if (ktest_mode == KTEST_PLAYBACK) {
    KTestObject *o = KTOV_next_object(&ktov, ktest_object_names[MASTER_SECRET]);
    if (o->numBytes != len) {
      fprintf(stderr,
	            "ktest_master_secret playback error: %d bytes requested, "
	      "%d bytes recorded", len, o->numBytes);
      printf( "ERROR: mismatch in length of ktest_master_secret during playback \n");
      fflush(stdout);
      exit(2);
    }
    
    if (o->numBytes > 0 && memcmp(ms, o->bytes, len) != 0) {
      fprintf(stderr, "WARNING: ktest_master_secret playback data mismatch\n");
      printf( "ERROR: Mismatch in master_secret playback \n");
      fflush(stdout);
      
    } else {
      printf( "master_secret generation MATCHES ktest_ playback record \n");
      fflush(stdout);
    }
    memcpy(ms, o->bytes, len);
    if (KTEST_DEBUG) {
      int i;
      printf("master_secret playback [%d]", len);
      for (i = 0; i < len; i++) {
	printf(" %2.2x", ms[i]);
      }
      printf("\n");
    }
    return;
  }
  else {
    perror("ktest_master_secret coding error - should never get here");
    exit(4);
  }
}



void ktest_start_tase(const char *filename, enum kTestMode mode) {
  
  KTOV_init(&ktov);
  ktest_mode = mode;

  // set ktest output filename and ktest network-only filename
  if (filename != NULL) {
    char *network_file = NULL;
    const char *suffix = ".net.ktest";
    const char *ext = ".ktest";
    int n_ext = strlen(ext);
    int n_f = strlen(filename);
    int n_suf = strlen(suffix);
    ktest_output_file = filename;
    network_file = (char *)malloc(sizeof(char) * (n_f + n_suf + 1));
    strcpy(network_file, filename);
    if (n_f > n_ext && strcmp(&filename[n_f-n_ext], ext) == 0) {
      strcpy(&network_file[n_f-n_ext], suffix);
    } else {
      strcat(network_file, suffix);
    }
    ktest_network_file = network_file;
  }

  // Load capture from file if playback mode
  if (ktest_mode == KTEST_PLAYBACK) {
    KTest *ktest;
    ktest = kTest_fromFile(filename);
    if (!ktest) {
      fprintf(stderr, "Error reading ktest file: %s\n", filename);
      exit(1);
    }
    ktov.size = ktov.capacity = ktest->numObjects;
    printf("Found %d ktov entries \n", ktov.size);
    ktov.objects = (KTestObject*)malloc(sizeof(KTestObject) * ktov.size);
    int i;
    for (i = 0; i < ktov.size; i++) {
      KTO_deepcopy(&ktov.objects[i], &ktest->objects[i]);
    }
    kTest_free(ktest);
  }
}

void ktest_finish_tase() {

  KTest ktest;

  if (ktest_mode == KTEST_NONE) {
    return;
  }

  else if (ktest_mode == KTEST_RECORD) {
    memset(&ktest, 0, sizeof(KTest));
    ktest.numObjects = ktov.size;
    ktest.objects = ktov.objects;

    KTOV_print(stdout, &ktov);

    int result = kTest_toFile(&ktest, ktest_output_file);
    if (!result) {
      perror("ktest_finish error");
      exit(1);
    }
    printf("KTest full capture written to %s.\n", ktest_output_file);

    // Sort network events to the front and write as separate file.
    int i, filtered_i;
    for (i = 0, filtered_i = 0; i < ktest.numObjects; i++) {
      if (strcmp(ktest.objects[i].name, "s2c") == 0 ||
	  strcmp(ktest.objects[i].name, "c2s") == 0) {
	KTestObject temp;
	temp = ktest.objects[filtered_i];
	ktest.objects[filtered_i] = ktest.objects[i];
	ktest.objects[i] = temp;
	filtered_i++;
      }
    }
    ktest.numObjects = filtered_i;

    result = kTest_toFile(&ktest, ktest_network_file);
    if (!result) {
      perror("ktest_finish error");
      exit(1);
    }
    printf("KTest network capture written to %s.\n", ktest_network_file);

    KTOV_done(&ktov);
  }

  else if (ktest_mode == KTEST_PLAYBACK) {
    // TODO: nothing except maybe cleanup?
  }
}

KTestObject * peekNextKTestObject () {
  if (ktov.playback_index >= ktov.size){
    printf("Tried to peek for nonexistent KTestObject \n");
    std::exit(EXIT_FAILURE);
  }else {
    return &(ktov.objects[ktov.playback_index]);
  }
}
