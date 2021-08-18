//===-- IndependentSolver.cpp ---------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "independent-solver"
#include "klee/Solver.h"

#include "klee/Expr.h"
#include "klee/Constraints.h"
#include "klee/SolverImpl.h"
#include "klee/Internal/Support/Debug.h"

#include "klee/util/ExprUtil.h"
#include "klee/util/Assignment.h"

#include "llvm/Support/raw_ostream.h"
#include <map>
#include <vector>
#include <ostream>
#include <list>
#include <unordered_set>

#include "klee/Internal/System/Time.h"

using namespace klee;
using namespace llvm;

extern UFElement * UF_Find(UFElement *);
extern void worker_exit();
extern bool useUF;


template<class T>
class DenseSet {
  typedef std::set<T> set_ty;
  set_ty s;

public:
  DenseSet() {}

  void add(T x) {
    s.insert(x);
  }
  void add(T start, T end) {
    for (; start<end; start++)
      s.insert(start);
  }

  // returns true iff set is changed by addition
  bool add(const DenseSet &b) {
    bool modified = false;
    for (typename set_ty::const_iterator it = b.s.begin(), ie = b.s.end(); 
         it != ie; ++it) {
      if (modified || !s.count(*it)) {
        modified = true;
        s.insert(*it);
      }
    }
    return modified;
  }

  bool intersects(const DenseSet &b) {
    for (typename set_ty::iterator it = s.begin(), ie = s.end(); 
         it != ie; ++it)
      if (b.s.count(*it))
        return true;
    return false;
  }

  std::set<unsigned>::iterator begin(){
    return s.begin();
  }

  std::set<unsigned>::iterator end(){
    return s.end();
  }

  void print(llvm::raw_ostream &os) const {
    bool first = true;
    os << "{";
    for (typename set_ty::iterator it = s.begin(), ie = s.end(); 
         it != ie; ++it) {
      if (first) {
        first = false;
      } else {
        os << ",";
      }
      os << *it;
    }
    os << "}";
  }
};

template <class T>
inline llvm::raw_ostream &operator<<(llvm::raw_ostream &os,
                                     const ::DenseSet<T> &dis) {
  dis.print(os);
  return os;
}

class IndependentElementSet {
public:
  typedef std::map<const Array*, ::DenseSet<unsigned> > elements_ty;
  elements_ty elements;                 // Represents individual elements of array accesses (arr[1])
  std::set<const Array*> wholeObjects;  // Represents symbolically accessed arrays (arr[x])
  std::vector<ref<Expr> > exprs;        // All expressions that are associated with this factor
                                        // Although order doesn't matter, we use a vector to match
                                        // the ConstraintManager constructor that will eventually
                                        // be invoked.

  IndependentElementSet() {}
  IndependentElementSet(ref<Expr> e) {
    exprs.push_back(e);
    // Track all reads in the program.  Determines whether reads are
    // concrete or symbolic.  If they are symbolic, "collapses" array
    // by adding it to wholeObjects.  Otherwise, creates a mapping of
    // the form Map<array, set<index>> which tracks which parts of the
    // array are being accessed.
    std::vector< ref<ReadExpr> > reads;
    findReads(e, /* visitUpdates= */ true, reads);
    for (unsigned i = 0; i != reads.size(); ++i) {
      ReadExpr *re = reads[i].get();
      const Array *array = re->updates.root;
      
      // Reads of a constant array don't alias.
      if (re->updates.root->isConstantArray() &&
          !re->updates.head)
        continue;

      if (!wholeObjects.count(array)) {
        if (ConstantExpr *CE = dyn_cast<ConstantExpr>(re->index)) {
          // if index constant, then add to set of constraints operating
          // on that array (actually, don't add constraint, just set index)
          ::DenseSet<unsigned> &dis = elements[array];
          dis.add((unsigned) CE->getZExtValue(32));
        } else {
          elements_ty::iterator it2 = elements.find(array);
          if (it2!=elements.end())
            elements.erase(it2);
          wholeObjects.insert(array);
        }
      }
    }
  }
  IndependentElementSet(const IndependentElementSet &ies) : 
    elements(ies.elements),
    wholeObjects(ies.wholeObjects),
    exprs(ies.exprs) {}

  IndependentElementSet &operator=(const IndependentElementSet &ies) {
    elements = ies.elements;
    wholeObjects = ies.wholeObjects;
    exprs = ies.exprs;
    return *this;
  }

  void print(llvm::raw_ostream &os) const {
    os << "{";
    bool first = true;
    for (std::set<const Array*>::iterator it = wholeObjects.begin(), 
           ie = wholeObjects.end(); it != ie; ++it) {
      const Array *array = *it;

      if (first) {
        first = false;
      } else {
        os << ", ";
      }

      os << "MO" << array->name;
    }
    for (elements_ty::const_iterator it = elements.begin(), ie = elements.end();
         it != ie; ++it) {
      const Array *array = it->first;
      const ::DenseSet<unsigned> &dis = it->second;

      if (first) {
        first = false;
      } else {
        os << ", ";
      }

      os << "MO" << array->name << " : " << dis;
    }
    os << "}";
  }

  // more efficient when this is the smaller set
  bool intersects(const IndependentElementSet &b) {
    // If there are any symbolic arrays in our query that b accesses
    for (std::set<const Array*>::iterator it = wholeObjects.begin(), 
           ie = wholeObjects.end(); it != ie; ++it) {
      const Array *array = *it;
      if (b.wholeObjects.count(array) || 
          b.elements.find(array) != b.elements.end())
        return true;
    }
    for (elements_ty::iterator it = elements.begin(), ie = elements.end();
         it != ie; ++it) {
      const Array *array = it->first;
      // if the array we access is symbolic in b
      if (b.wholeObjects.count(array))
        return true;
      elements_ty::const_iterator it2 = b.elements.find(array);
      // if any of the elements we access are also accessed by b
      if (it2 != b.elements.end()) {
        if (it->second.intersects(it2->second))
          return true;
      }
    }
    return false;
  }

  // returns true iff set is changed by addition
  bool add(const IndependentElementSet &b) {
    for(unsigned i = 0; i < b.exprs.size(); i ++){
      ref<Expr> expr = b.exprs[i];
      exprs.push_back(expr);
    }

    bool modified = false;
    for (std::set<const Array*>::const_iterator it = b.wholeObjects.begin(), 
           ie = b.wholeObjects.end(); it != ie; ++it) {
      const Array *array = *it;
      elements_ty::iterator it2 = elements.find(array);
      if (it2!=elements.end()) {
        modified = true;
        elements.erase(it2);
        wholeObjects.insert(array);
      } else {
        if (!wholeObjects.count(array)) {
          modified = true;
          wholeObjects.insert(array);
        }
      }
    }
    for (elements_ty::const_iterator it = b.elements.begin(), 
           ie = b.elements.end(); it != ie; ++it) {
      const Array *array = it->first;
      if (!wholeObjects.count(array)) {
        elements_ty::iterator it2 = elements.find(array);
        if (it2==elements.end()) {
          modified = true;
          elements.insert(*it);
        } else {
          // Now need to see if there are any (z=?)'s
          if (it2->second.add(it->second))
            modified = true;
        }
      }
    }
    return modified;
  }
};


class UFElement2 {
public:
  UFElement2 * parent;
  std::string accessName;
  int rank;
  int size;
  std::list<UFElement2 *>::iterator uniqueItr; //For collecting sets of indpenendent sets
  IndependentElementSet * IES;  //Only for representative of set

  //void UF_MakeSet2(UFElement2 * x);
  
  UFElement2(std::string an){
    accessName = an;
    IES = new IndependentElementSet();
  }
  UFElement2() {}

  
};


void UF_MakeSet2 (UFElement2 * x) {
  /*
  if (x->accessName == NULL) {
    printf("FATAL ERROR: accessName not set in MakeSet2 operation \n");
    fflush(stdout);
  }
  */
  x->parent = x;
  x->rank = 0;
  x->size = 1;


}


UFElement2 * UF_Find2 (UFElement2 *x) {
  if (x->parent != x)
    x->parent = UF_Find2(x->parent);
  return x->parent;
}

UFElement2 * UF_Union2 (UFElement2 * x, UFElement2 * y) {
  UFElement2 * xRep = UF_Find2(x);
  UFElement2 * yRep = UF_Find2(y);
  if (xRep == yRep)
    return xRep; //Nothing to do; already in same set.

  //Save constraints for a merge later
  //  std::vector<ref<Expr>> * xConstraints = &(x->constraints);
  //  std::vector<ref<Expr>> * yConstraints = &(y->constraints);

  //Merge and update rank
  //Case 1: Merge y into x
  //Also breaks tie if ranks equal
  if (xRep->rank >= yRep->rank) {
    yRep->parent = xRep;
    if (xRep->rank == yRep->rank)
      xRep->rank++;

    /*
    xConstraints->insert(xConstraints->end(), yConstraints->begin(), yConstraints->end());
    yConstraints->clear();
    */
    
    if (y->IES != NULL) {
      if (x->IES != NULL ) {
	x->IES->add(*(y->IES));
      }
    }
    
    return xRep;
  } else {
    //Case 2: Merge x into y
    xRep->parent = yRep;
    if (yRep->rank == xRep->rank)
      yRep->rank++;
    
    if (x->IES != NULL) {
      if (y->IES != NULL) {
	y->IES->add(*(x->IES));
      }
    }
    
    /*
    yConstraints->insert(yConstraints->end(), xConstraints->begin(), xConstraints->end());
    xConstraints->clear();
    */
    return yRep;
  }
}


inline llvm::raw_ostream &operator<<(llvm::raw_ostream &os,
                                     const IndependentElementSet &ies) {
  ies.print(os);
  return os;
}




//Implements getAllIndependentConstraintSets using union-find.  Vanilla implementation
// of getAllIndependentConstraintSets suffers ~ n^2 performance doing pairwise comparison
// when query contains a large number of simple independent constraints.

//WARNING -- assumes array accesses are at concrete offsets.  Will need to be generalized
//for symbolic address accesses.
static std::list<IndependentElementSet>*
getAllIndependentConstraintSetsUF(const Query &query) {

  //Todo -- Add query.expr like in the vanilla implementation
  
  typedef std::map<const Array*, ::DenseSet<unsigned> > elements_ty;
  std::list<IndependentElementSet> * factors = new std::list<IndependentElementSet>();
  std::vector<UFElement2 *> independentGroups;
  std::map<std::string, UFElement2 *> allElements;
  double IES_Creation_Time = 0.0;
  double UFE_Creation_Time = 0.0;
  std::list<UFElement2 *> uniqueSetReps;
  std::unordered_set<UFElement2 *> uniqueSReps;
  
  //Create UF elements for each array access
  for (ConstraintManager::const_iterator it = query.constraints.begin(),
	 ie = query.constraints.end();
       it != ie; ++it) {
      
    //Constructor populates info on concrete and symbolic array accesses
    IndependentElementSet * ies = new IndependentElementSet(*it);
    std::list<UFElement2 *> constraintAccesses;

    //Iterate through each element access
    for (elements_ty::iterator e_it = ies->elements.begin(), e_ie = ies->elements.end();
	 e_it != e_ie; ++e_it) {

      
      const Array *array = e_it->first;
      ::DenseSet<unsigned> ds = e_it->second;
      //If the element access represents multiple concrete offsets, check them here.
      for (std::set<unsigned>::iterator s_it = ds.begin(), s_ie = ds.end();
	   s_it != s_ie; ++s_it) {
	
	
	unsigned u = *s_it;
	std::string accessString = array->name + "__" +  std::to_string(u);
	
	if (allElements.find(accessString) == allElements.end()) {

	  UFElement2 * ufe = new UFElement2(accessString);
	  UF_MakeSet2(ufe);
	  
	  allElements.insert(std::make_pair(accessString, ufe));
	  uniqueSReps.insert(ufe);
	  constraintAccesses.push_back(ufe);
	  
	} else {

	  UFElement2 * ufe = ((allElements.find(accessString))->second);
	  constraintAccesses.push_back(ufe);

	}	
      }
    }
    
    //Run union find pairwise on all constraint accesses
    std::list<UFElement2 *>::iterator i = constraintAccesses.begin();
    
    UFElement2 * final = UF_Find2(*i);
    
    while (true) {
      UFElement2 * curr = *i;
      i++;
      if (i == constraintAccesses.end()) {
	break;
      } else {
	UFElement2 * next = *i;
	if (UF_Find2(curr) == UF_Find2(next)) {
	  uniqueSReps.erase(UF_Find2(curr));
	} else {
	  uniqueSReps.erase(UF_Find2(next));
	  uniqueSReps.erase(UF_Find2(curr));
	}
	  
	final = UF_Union2(curr,next);
	uniqueSReps.insert(final);
      }
    }
    
    if (final->IES != NULL) {
      final->IES->add(*ies);
    } else {
      final->IES = ies;
    }
  }

  std::list<IndependentElementSet> * factorList = new std::list<IndependentElementSet>;
  
  for (std::unordered_set<UFElement2 *>::iterator i = uniqueSReps.begin(); i != uniqueSReps.end(); i++) {
    UFElement2 * curr = *i;
    factorList->push_back(*(curr->IES));
  }
  
  return factorList;
  
}
  

// Breaks down a constraint into all of it's individual pieces, returning a
// list of IndependentElementSets or the independent factors.
//
// Caller takes ownership of returned std::list.
static std::list<IndependentElementSet>*
getAllIndependentConstraintsSets(const Query &query) {
  std::list<IndependentElementSet> *factors = new std::list<IndependentElementSet>();
  ConstantExpr *CE = dyn_cast<ConstantExpr>(query.expr);
  if (CE) {
    assert(CE && CE->isFalse() && "the expr should always be false and "
                                  "therefore not included in factors");
  } else {
    ref<Expr> neg = Expr::createIsZero(query.expr);
    factors->push_back(IndependentElementSet(neg));
  }

  for (ConstraintManager::const_iterator it = query.constraints.begin(),
                                         ie = query.constraints.end();
       it != ie; ++it) {
    // iterate through all the previously separated constraints.  Until we
    // actually return, factors is treated as a queue of expressions to be
    // evaluated.  If the queue property isn't maintained, then the exprs
    // could be returned in an order different from how they came it, negatively
    // affecting later stages.
    factors->push_back(IndependentElementSet(*it));
  }

  bool doneLoop = false;
  do {
    doneLoop = true;
    std::list<IndependentElementSet> *done =
        new std::list<IndependentElementSet>;
    while (factors->size() > 0) {
      IndependentElementSet current = factors->front();
      factors->pop_front();
      // This list represents the set of factors that are separate from current.
      // Those that are not inserted into this list (queue) intersect with
      // current.
      std::list<IndependentElementSet> *keep =
          new std::list<IndependentElementSet>;
      while (factors->size() > 0) {
        IndependentElementSet compare = factors->front();
        factors->pop_front();
        if (current.intersects(compare)) {
          if (current.add(compare)) {
            // Means that we have added (z=y)added to (x=y)
            // Now need to see if there are any (z=?)'s
            doneLoop = false;
          }
        } else {
          keep->push_back(compare);
        }
      }
      done->push_back(current);
      delete factors;
      factors = keep;
    }
    delete factors;
    factors = done;
  } while (!doneLoop);

  return factors;
}

static 
IndependentElementSet getIndependentConstraints(const Query& query,
                                                std::vector< ref<Expr> > &result) {
  IndependentElementSet eltsClosure(query.expr);
  std::vector< std::pair<ref<Expr>, IndependentElementSet> > worklist;

  for (ConstraintManager::const_iterator it = query.constraints.begin(), 
         ie = query.constraints.end(); it != ie; ++it)
    worklist.push_back(std::make_pair(*it, IndependentElementSet(*it)));

  // XXX This should be more efficient (in terms of low level copy stuff).
  bool done = false;
  do {
    done = true;
    std::vector< std::pair<ref<Expr>, IndependentElementSet> > newWorklist;
    for (std::vector< std::pair<ref<Expr>, IndependentElementSet> >::iterator
           it = worklist.begin(), ie = worklist.end(); it != ie; ++it) {
      if (it->second.intersects(eltsClosure)) {
        if (eltsClosure.add(it->second))
          done = false;
        result.push_back(it->first);
        // Means that we have added (z=y)added to (x=y)
        // Now need to see if there are any (z=?)'s
      } else {
        newWorklist.push_back(*it);
      }
    }
    worklist.swap(newWorklist);
  } while (!done);

  KLEE_DEBUG(
    std::set< ref<Expr> > reqset(result.begin(), result.end());
    errs() << "--\n";
    errs() << "Q: " << query.expr << "\n";
    errs() << "\telts: " << IndependentElementSet(query.expr) << "\n";
    int i = 0;
    for (ConstraintManager::const_iterator it = query.constraints.begin(),
        ie = query.constraints.end(); it != ie; ++it) {
      errs() << "C" << i++ << ": " << *it;
      errs() << " " << (reqset.count(*it) ? "(required)" : "(independent)") << "\n";
      errs() << "\telts: " << IndependentElementSet(*it) << "\n";
    }
    errs() << "elts closure: " << eltsClosure << "\n";
 );


  return eltsClosure;
}


// Extracts which arrays are referenced from a particular independent set.  Examines both
// the actual known array accesses arr[1] plus the undetermined accesses arr[x].
static
void calculateArrayReferences(const IndependentElementSet & ie,
                              std::vector<const Array *> &returnVector){
  std::set<const Array*> thisSeen;
  for(std::map<const Array*, ::DenseSet<unsigned> >::const_iterator it = ie.elements.begin();
      it != ie.elements.end(); it ++){
    thisSeen.insert(it->first);
  }
  for(std::set<const Array *>::iterator it = ie.wholeObjects.begin();
      it != ie.wholeObjects.end(); it ++){
    thisSeen.insert(*it);
  }
  for(std::set<const Array *>::iterator it = thisSeen.begin(); it != thisSeen.end();
      it ++){
    returnVector.push_back(*it);
  }
}

class IndependentSolver : public SolverImpl {
private:
  Solver *solver;
  bool legacy_mode; //Added from cliver to simplify computeInitialValues calls.
  
public:
  IndependentSolver(Solver *_solver, bool _legacy_mode=false) 
    : solver(_solver), legacy_mode(_legacy_mode) {}
  ~IndependentSolver() { delete solver; }

  bool computeTruth(const Query&, bool &isValid);
  bool computeValidity(const Query&, Solver::Validity &result);
  bool computeValue(const Query&, ref<Expr> &result);
  bool computeInitialValues(const Query& query,
                            const std::vector<const Array*> &objects,
                            std::vector< std::vector<unsigned char> > &values,
                            bool &hasSolution);
  SolverRunStatus getOperationStatusCode();
  char *getConstraintLog(const Query&);
  void setCoreSolverTimeout(double timeout);
  bool computeValidityCheat(const Query& query,
					       Solver::Validity &result);
};

IndependentSolver * IS;

bool IndependentSolver::computeValidity(const Query& query,
                                        Solver::Validity &result) {
  std::vector< ref<Expr> > required;


  if (useUF) {
    std::vector<const klee::Array *> arrs;
    findSymbolicObjects(query.expr, arrs);
    
    //Get the constraints associated with each arr variable 
    for (auto it = arrs.begin(); it != arrs.end(); it++) {
      
      UFElement * ufe = const_cast<UFElement *>((&((*it)->UFE)));
      UFElement * rep = UF_Find(ufe);
      required.insert(required.end(), rep->constraints.begin(), rep->constraints.end());
      
    }
    
  }else{    
    //required.clear();
    IndependentElementSet eltsClosure =
      getIndependentConstraints(query, required);
  }
  ConstraintManager tmp(required);
  return solver->impl->computeValidity(Query(tmp, query.expr), 
				       result);
}


//Cheat and just  evaluate the main expr in the query
bool IndependentSolver::computeValidityCheat(const Query& query,
					     Solver::Validity &result) {
  printf("Calling computeValidityCheat in IndependentSolver \n");
  fflush(stdout);
  std::vector< ref<Expr> > required;
  //required.push_back(query.expr);
  ConstraintManager tmp(required);
  return solver->impl->computeValidity(Query(tmp, query.expr), result);
  
}

bool IndependentSolver::computeTruth(const Query& query, bool &isValid) {
  std::vector< ref<Expr> > required;
  IndependentElementSet eltsClosure = 
    getIndependentConstraints(query, required);
  ConstraintManager tmp(required);
  return solver->impl->computeTruth(Query(tmp, query.expr), 
                                    isValid);
}

bool IndependentSolver::computeValue(const Query& query, ref<Expr> &result) {
  std::vector< ref<Expr> > required;
  IndependentElementSet eltsClosure = 
    getIndependentConstraints(query, required);
  ConstraintManager tmp(required);
  return solver->impl->computeValue(Query(tmp, query.expr), result);
}

// Helper function used only for assertions to make sure point created
// during computeInitialValues is in fact correct. The ``retMap`` is used
// in the case ``objects`` doesn't contain all the assignments needed.
bool assertCreatedPointEvaluatesToTrue(const Query &query,
                                       const std::vector<const Array*> &objects,
                                       std::vector< std::vector<unsigned char> > &values,
                                       std::map<const Array*, std::vector<unsigned char> > &retMap){
  // _allowFreeValues is set to true so that if there are missing bytes in the assigment
  // we will end up with a non ConstantExpr after evaluating the assignment and fail
  Assignment assign = Assignment(objects, values, /*_allowFreeValues=*/true);

  // Add any additional bindings.
  // The semantics of std::map should be to not insert a (key, value)
  // pair if it already exists so we should continue to use the assignment
  // from ``objects`` and ``values``.
  if (retMap.size() > 0)
    assign.bindings.insert(retMap.begin(), retMap.end());

  for(ConstraintManager::constraint_iterator it = query.constraints.begin();
      it != query.constraints.end(); ++it){
    ref<Expr> ret = assign.evaluate(*it);

    assert(isa<ConstantExpr>(ret) && "assignment evaluation did not result in constant");
    ref<ConstantExpr> evaluatedConstraint = dyn_cast<ConstantExpr>(ret);
    if(evaluatedConstraint->isFalse()){
      return false;
    }
  }
  ref<Expr> neg = Expr::createIsZero(query.expr);
  ref<Expr> q = assign.evaluate(neg);
  assert(isa<ConstantExpr>(q) && "assignment evaluation did not result in constant");
  return cast<ConstantExpr>(q)->isTrue();
}


//Placeholder for a trivial equality solver.
//Looks for query of the Form EQ Constant, (Read w8 idx array_name)
//For example, (Eq 32 (Read w8 104 stdin_3))
bool static getTrivialSolution(Query query, std::vector<const Array *> &objects,
			       std::vector<std::vector<unsigned char>> &values,
			       bool &hasSolution) {
  //printf("Attempting to get trivial solution \n");

  //fflush(stdout);
  if ( query.constraints.size() == 1) {
    ref<Expr> c = *(query.constraints.begin());
    if (klee::EqExpr * e = dyn_cast<klee::EqExpr>(c)) {
      //printf("TRIV DBG: Looking at a single equality expr... \n");
      //fflush(stdout);
      ref<Expr> left = e->getKid(0);
      if (klee::ConstantExpr * ve = dyn_cast<klee::ConstantExpr>(left)) {
	//printf("TRIV DBG: Left side is a constant... \n");
	//fflush(stdout);
	unsigned char value = ve->getZExtValue(8);
	//Add check for width
	ref<Expr> right = e->getKid(1);
	if (klee::ReadExpr * re = dyn_cast<klee::ReadExpr> (right)) {
	  //printf("TRIV DBG: Right side is a read... \n");
	  //fflush(stdout);
	  const Array *array = re->updates.root;
	  
	  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(re->index)) {
	    unsigned index = CE->getZExtValue(32);
	    //printf("TRIV DBG: Read is at index %u \n", index);
	    //fflush(stdout);
	    values = std::vector<std::vector<unsigned char>> (1);
	    //printf("DBG1 \n");
	    //fflush(stdout);
	    values[0] = std::vector<unsigned char> (array->size);
	    //printf("DBG2 \n");
	    //fflush(stdout);
	    values[0][index] = (unsigned char) value; 
	    //printf("TRIV DBG: Trivial assignment is %u", value);
	    //fflush(stdout);
	    hasSolution = true;
	    
	    //fflush(stdout);
	    return true;
	  }
	}
      }
    }
  }
  //hasSolution = false;
  return false;
}

bool IndependentSolver::computeInitialValues(const Query& query,
                                             const std::vector<const Array*> &objects,
                                             std::vector< std::vector<unsigned char> > &values,
                                             bool &hasSolution){

  
  double T0 = util::getWallTime();
  
  //getAllIndependentConstraintSetsUF(query);
  //printf("DBG UF: Spent %lf seconds running new UF code \n", util::getWallTime() - T0);
  
  //Added from Cliver
  if (legacy_mode) {
    return solver->impl->computeInitialValues(query, objects, values, hasSolution);
  }


  // We assume the query has a solution except proven differently
  // This is important in case we don't have any constraints but
  // we need initial values for requested array objects.
  hasSolution = true;
  // FIXME: When we switch to C++11 this should be a std::unique_ptr so we don't need
  // to remember to manually call delete

  T0 = util::getWallTime();
  std::list<IndependentElementSet> * factors = getAllIndependentConstraintSetsUF(query);
  printf("DBG UF: Spent %lf seconds running new UF code \n", util::getWallTime() - T0);

  //std::list<IndependentElementSet> *factors = getAllIndependentConstraintsSets(query);
  //printf("UF DBG: Vanilla got %d constraints \n", factors->size());
  //printf("Solver DBG1: %lf seconds in getAllIndependentConstraintsSets \n", util::getWallTime() - T0);
  //Used to rearrange all of the answers into the correct order
  std::map<const Array*, std::vector<unsigned char> > retMap;
  for (std::list<IndependentElementSet>::iterator it = factors->begin();
       it != factors->end(); ++it) {
    T0 = util::getWallTime();
    std::vector<const Array*> arraysInFactor;
    calculateArrayReferences(*it, arraysInFactor);
    // Going to use this as the "fresh" expression for the Query() invocation below
    assert(it->exprs.size() >= 1 && "No null/empty factors");
    if (arraysInFactor.size() == 0){
      continue;
    }
    ConstraintManager tmp(it->exprs);
    std::vector<std::vector<unsigned char> > tempValues;
    //printf("Solver DBG: %lf seconds setting up for call to computeInitialValues \n", util::getWallTime() - T0);
    if (false) {
      printf("--Printing Query: \n");
      outs() << "Constraints: \n";
      //printf("Constraints: \n");
      for (auto it = tmp.begin(); it != tmp.end(); it++) {
	(*it)->print(outs());
      }
      outs().flush();
      fflush(stdout);
      //printf("Query Expr: \n");
      outs() << "Query Expr: \n";
      outs().flush();
      fflush(stdout);
      
      ref<Expr> theQuery = ConstantExpr::alloc(0, Expr::Bool);
      theQuery->print(outs());
      outs().flush();
      fflush(stdout);
      printf("\n");
      //Query q =  Query(tmp, ConstantExpr::alloc(0, Expr::Bool));
      
      //outs().flush();
    }
    
    bool cheapTrySuccess = getTrivialSolution(Query(tmp, ConstantExpr::alloc(0, Expr::Bool)),
		       arraysInFactor, tempValues, hasSolution);
    //---------
    bool solverRes;
    if (!cheapTrySuccess || !hasSolution) 
      solverRes = (solver->impl->computeInitialValues(Query(tmp, ConstantExpr::alloc(0, Expr::Bool)),
						       arraysInFactor, tempValues, hasSolution));
    else
      solverRes = cheapTrySuccess;
	
	
    //if (!solver->impl->computeInitialValues(Query(tmp, ConstantExpr::alloc(0, Expr::Bool)),
    //                                       arraysInFactor, tempValues, hasSolution)){
    if (!solverRes) {
      values.clear();
      delete factors;
      return false;
    } else if (!hasSolution){
      values.clear();
      delete factors;
      return true;
    } else {
      //--------------
      
      //printf("arraysInFactor.size is %d after query returns \n", arraysInFactor.size());
      //printf("tempValues.size is %d after query returns \n", tempValues.size());
      if (tempValues.size() >= 1) {
	//printf("first vector in tempValues return is length %lu \n", tempValues[0].size());
      }
      
      //--------------
      
      //T0 = util::getWallTime();
      assert(tempValues.size() == arraysInFactor.size() &&
             "Should be equal number arrays and answers");
      for (unsigned i = 0; i < tempValues.size(); i++){
        if (retMap.count(arraysInFactor[i])){
          // We already have an array with some partially correct answers,
          // so we need to place the answers to the new query into the right
          // spot while avoiding the undetermined values also in the array
          std::vector<unsigned char> * tempPtr = &retMap[arraysInFactor[i]];
          assert(tempPtr->size() == tempValues[i].size() &&
                 "we're talking about the same array here");
          ::DenseSet<unsigned> * ds = &(it->elements[arraysInFactor[i]]);
          for (std::set<unsigned>::iterator it2 = ds->begin(); it2 != ds->end(); it2++){
            unsigned index = * it2;
	    //printf("Copying partial solution at index %u \n", index) ;
	    
            (* tempPtr)[index] = tempValues[i][index];
          }
        } else {
          // Dump all the new values into the array
          retMap[arraysInFactor[i]] = tempValues[i];
        }
      }
      //printf("Solver DBG3: %lf seconds cleaning up after getInitialValues call \n", util::getWallTime() - T0);
    }
  }
  
  T0 = util::getWallTime();
  for (std::vector<const Array *>::const_iterator it = objects.begin();
       it != objects.end(); it++){
    const Array * arr = * it;
    if (!retMap.count(arr)){
      // this means we have an array that is somehow related to the
      // constraint, but whose values aren't actually required to
      // satisfy the query.
      std::vector<unsigned char> ret(arr->size);
      values.push_back(ret);
    } else {
      values.push_back(retMap[arr]);
    }
  }
  assert(assertCreatedPointEvaluatesToTrue(query, objects, values, retMap) && "should satisfy the equation");
  delete factors;
  printf("Solver DBG: Final cleanup in IndependentSolver took %lf seconds \n", util::getWallTime() - T0);
  return true;
}

SolverImpl::SolverRunStatus IndependentSolver::getOperationStatusCode() {
  return solver->impl->getOperationStatusCode();      
}

char *IndependentSolver::getConstraintLog(const Query& query) {
  return solver->impl->getConstraintLog(query);
}

void IndependentSolver::setCoreSolverTimeout(double timeout) {
  solver->impl->setCoreSolverTimeout(timeout);
}

Solver *klee::createIndependentSolver(Solver *s, bool legacy_mode) {
  return new Solver(new IndependentSolver(s, legacy_mode));
}
