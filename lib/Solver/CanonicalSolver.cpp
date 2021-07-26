//===-- CanonicalSolver.cpp -----------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//Taken from Cliver

#include "klee/Solver.h"

#include "klee/Expr.h"
#include "klee/Constraints.h"
#include "klee/SolverImpl.h"
#include "klee/util/ExprVisitor.h"
#include "klee/util/ExprUtil.h"

#include "llvm/ADT/StringExtras.h"

#include <map>
#include <vector>

#include "klee/Internal/System/Time.h"

using namespace klee;
using namespace llvm;

typedef std::set<const Array*> ArraySet;
typedef std::vector<const Array*> ArrayVector;
typedef std::map<const Array*, unsigned> ArrayIndexMap;
typedef std::map<size_t, ArrayVector> SizeArrayVectorMap;
typedef std::map<size_t, int> SizeIndexMap;

// Robby's brain: Added this canonical solver for Xpilot

//ABH added to work around latest klee's dependency on the array cache
#include "../Core/Executor.h"
extern klee::Executor * GlobalExecutorPtr;

class CanonicalVisitor : public ExprVisitor {
 private:

  ArrayIndexMap&      arrayIndexMap;
  SizeIndexMap&       sizeLRUMap;
  SizeArrayVectorMap& canonicalArrayMap;
  ArraySet&           canonicalArraySet;

 public:

  CanonicalVisitor(ArrayIndexMap &arrayIndexMap, 
                   SizeIndexMap &sizeLRUMap,
                   SizeArrayVectorMap &canonicalArrayMap,
                   ArraySet &canonicalArraySet)
    : ExprVisitor(true), 
      arrayIndexMap(arrayIndexMap),
      sizeLRUMap(sizeLRUMap),
      canonicalArrayMap(canonicalArrayMap),
      canonicalArraySet(canonicalArraySet) {}

  Action visitRead(const ReadExpr &e) {
    if (e.updates.root != NULL) {

      if (canonicalArraySet.count(e.updates.root) != 0) {
        return Action::doChildren();
      }

      const Array *array = e.updates.root;

      if (!array->isConstantArray()) {
        size_t sz = e.updates.root->size;

        if (arrayIndexMap.find(e.updates.root) == arrayIndexMap.end()) {

          if (sizeLRUMap.find(sz) == sizeLRUMap.end())
            sizeLRUMap[sz] = -1;

          int LRUIndex = sizeLRUMap[sz];

          // Check if an array of this size has been created but not yet used
          if ((LRUIndex + 1) < canonicalArrayMap[sz].size()) {
            array = canonicalArrayMap[sz][LRUIndex + 1];

          } else {
            // Create new array
            std::string arrayName = "c_" + llvm::utostr(sz) 
                + "_" + llvm::utostr(canonicalArrayMap[sz].size() + 1);
            array = GlobalExecutorPtr->getArrayCache()->CreateArray(arrayName, sz);

            // Insert into data structures
            canonicalArrayMap[sz].push_back(array);
            canonicalArraySet.insert(array);
          }

          sizeLRUMap[sz] = arrayIndexMap[e.updates.root] = LRUIndex + 1;

        } else {
          array = canonicalArrayMap[sz][arrayIndexMap[e.updates.root]];
        }
      }

      // Because extend() pushes a new UpdateNode onto the list, we need to walk
      // the list in reverse to rebuild it in the same order.
      std::vector< const UpdateNode*> updateList;
      for (const UpdateNode *un = e.updates.head; un; un = un->next) {
        updateList.push_back(un);
      }

      // walk list in reverse
      UpdateList updates(array, NULL);
      for (std::vector<const UpdateNode*>::reverse_iterator it = updateList.rbegin(),
           ie = updateList.rend(); it != ie; ++it) {
        updates.extend(visit((*it)->index), visit((*it)->value));
      }

      return Action::changeTo(ReadExpr::create(updates, visit(e.index)));
    }

    return Action::doChildren();
  }
};

class CanonicalSolver : public klee::SolverImpl {
 private:
  
  klee::Solver *solver;
  SizeArrayVectorMap canonicalArrayMap;
  ArraySet canonicalArraySet;

  void canonicalize(klee::ref<klee::Expr> &expr,
                    klee::ConstraintManager &constraints) {
    ArrayIndexMap arrayIndexMap;
    SizeIndexMap sizeLruMap;

    CanonicalVisitor visitor(arrayIndexMap, sizeLruMap, 
                             canonicalArrayMap, canonicalArraySet);
    expr = visitor.visit(expr);

    for (klee::ConstraintManager::constraints_ty::iterator
          it = constraints.begin(), ie = constraints.end(); 
          it != ie; ++it) {
      *it = visitor.visit(*it);
    }
  }

 public:
  CanonicalSolver(klee::Solver *s) : solver(s) {}

  ~CanonicalSolver() { delete solver; }

  bool computeValidity(const klee::Query& query, 
                       klee::Solver::Validity &isValid) {
    klee::ref<klee::Expr> expr = query.expr;
    klee::ConstraintManager constraints(query.constraints);
    canonicalize(expr, constraints);
    klee::Query canonicalQuery(constraints, expr);
    return solver->impl->computeValidity(canonicalQuery, isValid);
  }

  bool computeTruth(const klee::Query& query, bool &result) {
    klee::ref<klee::Expr> expr = query.expr;
    klee::ConstraintManager constraints(query.constraints);
    canonicalize(expr, constraints);
    klee::Query canonicalQuery(constraints, expr);
    return solver->impl->computeTruth(canonicalQuery, result);
  }

  bool computeValue(const klee::Query& query, 
                    klee::ref<klee::Expr> &result) {
    klee::ref<klee::Expr> expr = query.expr;
    klee::ConstraintManager constraints(query.constraints);
    canonicalize(expr, constraints);
    klee::Query canonicalQuery(constraints, expr);
    return solver->impl->computeValue(canonicalQuery, result);
  }

  bool computeInitialValues(const klee::Query& query,
                            const std::vector<const klee::Array*> &objects,
                            std::vector< std::vector<unsigned char> > &values,
                            bool &hasSolution) {
    return solver->impl->computeInitialValues(query, objects, values, 
                                              hasSolution);
  }

  SolverRunStatus getOperationStatusCode() {
    return solver->impl->getOperationStatusCode();
  }

  char *getConstraintLog(const klee::Query& query) {
    return solver->impl->getConstraintLog(query);
  }

  void setCoreSolverTimeout(double timeout) {
    solver->impl->setCoreSolverTimeout(timeout);
  }

};

Solver *klee::createCanonicalSolver(Solver *s) {
  return new Solver(new CanonicalSolver(s));
}

