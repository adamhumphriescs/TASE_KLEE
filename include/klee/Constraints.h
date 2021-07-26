//===-- Constraints.h -------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef KLEE_CONSTRAINTS_H
#define KLEE_CONSTRAINTS_H

#include "klee/Expr.h"
#include "klee/util/ExprHashMap.h"

#include <unordered_map>



// FIXME: Currently we use ConstraintManager for two things: to pass
// sets of constraints around, and to optimize constraints. We should
// move the first usage into a separate data structure
// (ConstraintSet?) which ConstraintManager could embed if it likes.
namespace klee {

class ExprVisitor;
  
class ConstraintManager {
public:
  typedef std::vector< ref<Expr> > constraints_ty;
  typedef constraints_ty::iterator iterator;
  typedef constraints_ty::const_iterator const_iterator;

  ConstraintManager() {}

  // create from constraints with no optimization
  explicit
  ConstraintManager(const std::vector< ref<Expr> > &_constraints) :
    constraints(_constraints) {}
  
  ConstraintManager(const ConstraintManager &cs) : constraints(cs.constraints) {}
  

  typedef std::vector< ref<Expr> >::const_iterator constraint_iterator;
  typedef std::vector< ref<Expr> >::iterator nonconst_constraint_iterator;
  
  // given a constraint which is known to be valid, attempt to 
  // simplify the existing constraint set
  void simplifyForValidConstraint(ref<Expr> e);

  ref<Expr> simplifyExpr(ref<Expr> e) const;
  ref<Expr> simplifyWithXorOptimization(ref<Expr> e) const;

  //Added option for caller to provide list of symbolic variable names to avoid finding them twice.
  void addConstraint(ref<Expr> e, std::vector<const klee::Array * > * symNames = NULL);

  void DoXorOptimization();
  
  bool empty() const {
    return constraints.empty();
  }
  ref<Expr> back() const {
    return constraints.back();
  }
  constraint_iterator begin() const {
    return constraints.begin();
  }
  constraint_iterator end() const {
    return constraints.end();
  }
  
  nonconst_constraint_iterator begin() {
    return constraints.begin();
  }
  nonconst_constraint_iterator end() {
    return constraints.end();
  }
  
  
  size_t size() const {
    return constraints.size();
  }

  void clear() {
    constraints.clear();
  }
  
  
  bool operator==(const ConstraintManager &other) const {
    return constraints == other.constraints;
  }
  
private:
  std::vector< ref<Expr> > constraints;
  //std::map< ref<Expr>, ref<Expr> > equalities_map;
  //std::unordered_map< unsigned, std::pair< ref<Expr>, ref<Expr> > > equalities_hashval_map;
  
  // returns true iff the constraints were modified
  bool rewriteConstraints(ExprVisitor &visitor);

  //Added extra option to pass in list of symbolic variable names if the caller has already done the work.
  void addConstraintInternal(ref<Expr> e, std::vector<const klee::Array * > * symNames = NULL);
  //void addToEqualitiesMap(ref<Expr> e);
};

}

#endif /* KLEE_CONSTRAINTS_H */
