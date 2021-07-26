//===-- CVAssignment.cpp ----------------------------------------*- C++ -*-===//
//
// <insert license>
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

//Modified for TASE

#ifndef CV_ASSIGNMENT_H
#define CV_ASSIGNMENT_H


#include "klee/util/Assignment.h"
#include "klee/Solver.h"
#include "../../lib/Core/Executor.h"
#include "klee/ExecutionState.h"

#include <ostream>

//ABH: Added klee namespace for TASE
using namespace klee;

class CVAssignment : public klee::Assignment {
 public:
  typedef std::map<std::string, const klee::Array*> name_ty;

  name_ty name_bindings;

  CVAssignment() : klee::Assignment(false) {}

  CVAssignment(klee::ref<klee::Expr> &e);

  CVAssignment(std::vector<const klee::Array*> &objects, 
               std::vector< std::vector<unsigned char> > &values);

  void serializeAssignments(void * buf, int bufSize);

  void solveForBindings(klee::Solver* solver, 
                        klee::ref<klee::Expr> &expr,
			klee::ExecutionState * ExecStatePtr);

  void addBindings(std::vector<const klee::Array*> &objects, 
                   std::vector< std::vector<unsigned char> > &values);

  void deserializeAssignments ( void * buf, int bufSize, Executor * exec, CVAssignment * cv ) ;
  
  const klee::Array* getArray(std::string &name) {
    if (name_bindings.find(name) != name_bindings.end()) {
      return name_bindings[name];
    }
    return NULL;
  }

  std::vector<unsigned char>* getBindings(std::string &name) {
    if (name_bindings.find(name) != name_bindings.end()) {
      return &(bindings[name_bindings[name]]);
    }
    return NULL;
  }

  bool has(std::string &name) {
    return name_bindings.find(name) != name_bindings.end();
  }

  void clear() {
    bindings.clear();
    name_bindings.clear();
  }

  void printAllAssignments(FILE * f);
  
  size_t size() {
    return name_bindings.size();
  }

  void print(std::ostream& os) const;
};

typedef struct {
  uint64_t messageNumber; // Number of message M in ktest log currently being verified
  uint64_t passCount;    // Number of times N we've executed verification for current message M
  CVAssignment currMultipassAssignment;  //information on assignments learned up to and including current pass N
  CVAssignment prevMultipassAssignment; //information on assignments learned up to and including previous pass N-1
  uint64_t roundRootPID; // Root pid that we use as our anchor for verification of M.  We return to this
  //pid's state after successive passes generate additional info on symbolic variable assignments.
} multipassRecord;

#endif
