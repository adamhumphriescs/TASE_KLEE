//===-- KTest.h --------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef __COMMON_KTEST_H__
#define __COMMON_KTEST_H__

#include <time.h>

#ifdef __cplusplus
//extern "C" {
#endif

  // Capture mode
  enum kTestMode {KTEST_NONE, KTEST_RECORD, KTEST_PLAYBACK};
  
  typedef struct KTestObject KTestObject;
  struct KTestObject {
    char *name;
    struct timeval timestamp;
    unsigned numBytes;
    unsigned char *bytes;
  };

  typedef struct KTestObjectVector {
    KTestObject *objects;
    int size;
    int capacity; // capacity >= size
    int playback_index; // starts at 0
  } KTestObjectVector;

  KTestObject* KTOV_next_object(KTestObjectVector *ov, const char *name);
  
  
  
  typedef struct KTest KTest;
  struct KTest {
    /* file format version */
    unsigned version; 
    
    unsigned numArgs;
    char **args;

    unsigned symArgvs;
    unsigned symArgvLen;

    unsigned numObjects;
    KTestObject *objects;
  };

  //Added to tase to peek at next kto when predicting stdin length
  KTestObject* peekNextKTestObject ();

  /* returns the current .ktest file format version */
  unsigned kTest_getCurrentVersion();
  
  /* return true iff file at path matches KTest header */
  int   kTest_isKTestFile(const char *path);

  /* returns NULL on (unspecified) error */
  KTest* kTest_fromFile(const char *path);

  /* returns 1 on success, 0 on (unspecified) error */
  int   kTest_toFile(KTest *, const char *path);
  
  /* returns total number of object bytes */
  unsigned kTest_numBytes(KTest *);

  void  kTest_free(KTest *);

  void ktest_start_tase(const char *filename, enum kTestMode mode);
  void ktest_finish_tase();     // write capture to file
#ifdef __cplusplus
//}

 
#endif

#endif
