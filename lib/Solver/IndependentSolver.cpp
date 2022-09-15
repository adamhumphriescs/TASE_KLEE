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


extern UFManager *ufmanager;
//extern UFElement * UF_Find(UFElement *);
extern bool useUF;


template<class T>
class DenseSet {
  std::set<T> s;


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
    auto ee = s.end();
    for (auto it = b.cbegin(); it != b.cend(); ++it) {
      if (modified || s.find(*it) != ee) {
        modified = true;
        s.insert(*it);
      }
    }
    return modified;
  }

  bool intersects(const DenseSet &b) {
    for (auto it = s.cbegin(); it != s.cend(); ++it)
      if ( b.s.find(*it) != b.cend() )
        return true;
    return false;
  }

  typename std::set<T>::iterator begin(){
    return s.begin();
  }

  typename std::set<T>::iterator end(){
    return s.end();
  }

  const typename std::set<T>::iterator cbegin() const {
    return s.cbegin();
  }

  const typename std::set<T>::iterator cend() const {
    return s.cend();
  }
  
  void print(llvm::raw_ostream &os) const {
    bool first = true;
    os << "{";
    for (const auto &it : s ) {
      if (first) {
        first = false;
      } else {
        os << ",";
      }
      os << it;
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

  bool empty() {
    return wholeObjects.empty() && exprs.empty();
  }
  
  IndependentElementSet() {}
  IndependentElementSet(const ref<Expr>& e) {
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

  
  IndependentElementSet(IndependentElementSet &&o)
    : elements(std::move(o.elements))
    , wholeObjects(std::move(o.wholeObjects))
    , exprs(std::move(o.exprs))
  {}		   
		 
  
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
    for (auto it = wholeObjects.begin(), ie = wholeObjects.end(); it != ie; ++it) {
      const Array *array = *it;

      if (first) {
        first = false;
      } else {
        os << ", ";
      }

      os << "MO" << array->name;
    }
    
    for (auto it = elements.begin(), ie = elements.end(); it != ie; ++it) {
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
    for (auto it = wholeObjects.begin(), ie = wholeObjects.end(); it != ie; ++it) {
      const Array *array = *it;
      if (b.wholeObjects.find(array) != b.wholeObjects.end() ||
          b.elements.find(array) != b.elements.end())            
        return true;
    }
    
    for (auto it = elements.begin(), ie = elements.end(); it != ie; ++it) {
      const Array *array = it->first;
      // if the array we access is symbolic in b
      if ( b.wholeObjects.find(array) != b.wholeObjects.end() )
        return true;

      auto it2 = b.elements.find(array);
      // if any of the elements we access are also accessed by b
      if ( it2 != b.elements.end() ) {
        if (it->second.intersects(it2->second))
          return true;
      }
    }

    return false;
  }
  
  // returns true iff set is changed by addition
  bool add(const IndependentElementSet &b) {
    for(unsigned i = 0; i < b.exprs.size(); i ++){
      exprs.push_back(b.exprs[i]);
    }

    bool modified = false;
    for (auto it = b.wholeObjects.begin(), ie = b.wholeObjects.end(); it != ie; ++it) {
      const Array *array = *it;

      // if we access array that's symbolic in b, add symbolic and remove from regular access
      // otherwise if it's not symbolic already add it
      auto it2 = elements.find(array);
      if ( it2 != elements.end() ) { 
        modified = true;
        elements.erase(it2);
        wholeObjects.insert(array);
      } else {
        if ( wholeObjects.count(array) == 0 ) {
          modified = true;
          wholeObjects.insert(array);
        }
      }
    }

    // add new accesses if they aren't into something symbolic
    for (auto it = b.elements.begin(), ie = b.elements.end(); it != ie; ++it) {
      const Array *array = it->first;
      if ( !wholeObjects.count(array) ) {
        auto it2 = elements.find(array);
        if ( it2 == elements.end() ) {
          modified = true;
          elements.insert(*it);
        } else {
          // Now need to see if there are any (z=?)'s
          if ( it2->second.add(it->second) )
            modified = true;
        }
      }
    }
    return modified;
  }
};





class UFElement2 {
public:
  int parent;
  int rank;
  int size;
  std::string accessName;
  IndependentElementSet ies;  //Only for representative of set


  void add(IndependentElementSet &x) {
    if( ies.empty() ) {
      ies = x;
    } else {
      ies.add(x);
    }
  }

  UFElement2(const int parent, const std::string& an)
    : parent(parent)
    , rank(0)
    , size(1)
    , accessName(an)
    , ies()
  {}
  
  UFElement2() {}
};


struct UFManager2 {
  std::vector<UFElement2> elements;

  int create(const std::string& an){
    elements.emplace_back( elements.size()+1, an );
    return elements.size(); 
  }
  
  UFElement2 & operator[](const int i){
    return elements[i];
  }

  int find( UFElement2 &x ) {
    if ( &x != &elements[x.parent] ) {
      x.parent = find( elements[x.parent] );
    }
    return x.parent;
  }

  int find(const int i) {
    return find( elements[i] );
  }

  int join( const int a, const int b ) {
    auto &x = elements[a];
    auto &y = elements[b];
    
    auto i = find( x );
    auto j = find( y );
    
    if ( i == j ) {
      return i;
    }

    if ( x.rank >= y.rank ) {
      elements[j].parent = i;
      
      if ( x.rank > y.rank ) {
	++x.rank;
      }
      
      if ( !y.ies.empty() ) {
        if ( !x.ies.empty() ) {
          x.ies.add( y.ies );
        }
      }
      return i;
    } else {
      elements[i].parent = j;

      if ( !x.ies.empty() ) {
        if ( !y.ies.empty() ) {
          y.ies.add( x.ies );
        }
      }

      return j;
    }
  }
};


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
static std::vector<IndependentElementSet>
getAllIndependentConstraintSetsUF(const Query &query) {
  //Todo -- Add query.expr like in the vanilla implementation
  std::map<std::string, int> allElements;
  std::unordered_set<int> uniqueSReps;
  UFManager2 manager;
  //Create UF elements for each array access
  for (const auto &it : query.constraints ) {
    //Constructor populates info on concrete and symbolic array accesses
    auto ies = IndependentElementSet(it);
    std::vector<int> constraintAccesses;
    
    //Iterate through each element access
    for (auto &e_it : ies.elements ) {
      const Array *array = e_it.first;
      auto ds = e_it.second;
      //If the element access represents multiple concrete offsets, check them here.
      for (auto u : ds ) {
	std::string accessString = array->name + "__" +  std::to_string(u);
	
	if ( allElements.find(accessString) == allElements.end() ) {
	  int ufe = manager.create(accessString);
	  
	  allElements.insert(std::make_pair(accessString, ufe));
	  uniqueSReps.insert(ufe);
	  constraintAccesses.push_back(ufe);
	  
	} else {
	  int ufe = (allElements.find(accessString))->second;
	  constraintAccesses.push_back(ufe);
	}	
      }
    }
    
    //Run union find pairwise on all constraint accesses
    auto i = constraintAccesses.begin();
    int ff = manager.find(*i);
    
    while (true) {
      int curr = *i;
      ++i;
      if (i == constraintAccesses.end()) {
	break;
      } else {
	int next = *i;
	int a = manager.find(curr);
	int b = manager.find(next);
	if ( a == b ) {
	  uniqueSReps.erase(a);
	} else {
	  uniqueSReps.erase(b);
	  uniqueSReps.erase(a);
	}
	  
	ff = manager.join(curr, next);
	uniqueSReps.insert(ff);
      }
    }
    
    manager[ff].add(ies);
  }

  auto factorList = std::vector<IndependentElementSet>();
  
  for (const auto idx : uniqueSReps) {
    factorList.emplace_back( std::move( manager[idx].ies ) );
  }
  
  return factorList;
}
  


// Breaks down a constraint into all of it's individual pieces, returning a
// list of IndependentElementSets or the independent factors.
//
// Caller takes ownership of returned std::list.
//static std::list<IndependentElementSet>*
static std::vector<IndependentElementSet>
getAllIndependentConstraintsSets(const Query &query) {
  std::vector<IndependentElementSet> factors;
  
  ConstantExpr *CE = dyn_cast<ConstantExpr>(query.expr);
  if (CE) {
    assert(CE && CE->isFalse() && "the expr should always be false and "
                                  "therefore not included in factors");
  } else {
    ref<Expr> neg = Expr::createIsZero(query.expr);
    factors.push_back(IndependentElementSet(neg));
  }

  auto size = (query.constraints.end() - query.constraints.begin()) + factors.size();
  factors.reserve(size);
    
  // iterate through all the previously separated constraints.  Until we
  // actually return, factors is treated as a queue of expressions to be
  // evaluated.  If the queue property isn't maintained, then the exprs
  // could be returned in an order different from how they came it, negatively
  // affecting later stages.
  
  std::transform(query.constraints.begin(), query.constraints.end(), std::back_inserter(factors),
		 [](const ref<Expr>& x){return IndependentElementSet(x);});

  
  auto done = std::vector<bool>(size, false);
  unsigned i = 0;
  bool changed = false;
  do {
    changed = false;
    if ( !done[i] ) {
      for ( unsigned j = i + 1; j < size; ++j ) {
	if ( !done[j] ) {
	  if ( factors[i].intersects(factors[j]) ) {
	    done[j] = true;
	    changed |= factors[i].add(factors[j]);
	  }
	}
      }
    }
    i = i == size - 1 ? 0 : i + 1;
  } while ( changed || std::find(done.begin(), done.end(), true) != done.end() );

  std::vector<IndependentElementSet> keep;
  for( auto x = factors.begin(); x != factors.end(); ++x ) {
    if( done[x-factors.begin()] ){
      keep.push_back(*x);
    }
  }

  return keep;
}
  

static 
IndependentElementSet getIndependentConstraints(const Query& query,
                                                std::vector< ref<Expr> > &result) {
  IndependentElementSet eltsClosure(query.expr);
  std::vector< std::pair<ref<Expr>, IndependentElementSet> > worklist;

  for (const auto &it : query.constraints )
    worklist.push_back(std::make_pair(it, IndependentElementSet(it)));

  // XXX This should be more efficient (in terms of low level copy stuff).
  bool done = false;
  do {
    done = true;
    std::vector< std::pair<ref<Expr>, IndependentElementSet> > newWorklist;
    for (auto &it : worklist ) {
      if (it.second.intersects(eltsClosure)) {
        if (eltsClosure.add(it.second))
          done = false;
        result.push_back(it.first);
        // Means that we have added (z=y)added to (x=y)
        // Now need to see if there are any (z=?)'s
      } else {
        newWorklist.push_back(it);
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
    for (auto &it : query.constraints ) {
      errs() << "C" << i++ << ": " << it;
      errs() << " " << (reqset.count(it) ? "(required)" : "(independent)") << "\n";
      errs() << "\telts: " << IndependentElementSet(it) << "\n";
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
  for(auto it = ie.elements.begin(); it != ie.elements.end(); it++){
    thisSeen.insert(it->first);
  }
  
  for(auto it = ie.wholeObjects.begin(); it != ie.wholeObjects.end(); it ++){
    thisSeen.insert(*it);
  }
  
  for(auto it = thisSeen.begin(); it != thisSeen.end(); it ++){
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


std::vector<const klee::Array *> arrs;    
bool IndependentSolver::computeValidity(const Query& query,
                                        Solver::Validity &result) {
  //  query.dump();
  std::vector< ref<Expr> > required;
  double T0 = util::getWallTime();

  if (useUF) {
    if ( arrs.size() == 0 ) {
      findSymbolicObjects(query.expr, arrs);
    }
    printf("Query Results (size %d): ", arrs.size());
    for( auto *x : arrs ) {
      printf("\t%s", x->name.c_str());
    }
    printf("\n");

    
    printf("computeValidity time A: %lf\n", util::getWallTime() - T0);
    T0 = util::getWallTime();
    //Get the constraints associated with each arr variable 
    for (auto it = arrs.begin(); it != arrs.end(); it++) {
      
      int ufe = (*it)->UFE;
      auto &rep = (*ufmanager)[ufmanager->find(ufe)];
      required.insert(required.end(), rep.constraints.begin(), rep.constraints.end());      
    }
    printf("computeValidity time B: %lf\n", util::getWallTime() - T0);
    T0 = util::getWallTime();
  }else{    
    //required.clear();
    IndependentElementSet eltsClosure =
      getIndependentConstraints(query, required);
  }
  ConstraintManager tmp(required);
  printf("computeValidity time C: %lf\n", util::getWallTime() - T0);
  
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
   
  //Added from Cliver
  // passes the query down to the rest of the solver chain
  if (legacy_mode) {
    printf("Passing query to legacy solver: legacy_mode=TRUE\n");
    return solver->impl->computeInitialValues(query, objects, values, hasSolution);
  }


  // We assume the query has a solution except proven differently
  // This is important in case we don't have any constraints but
  // we need initial values for requested array objects.
  hasSolution = true;
  T0 = util::getWallTime();

  auto factors = getAllIndependentConstraintSetsUF(query);
  printf("DBG UF: Spent %lf seconds running new UF code \n", util::getWallTime() - T0);

  //Used to rearrange all of the answers into the correct order
  std::map<const Array*, std::vector<unsigned char> > retMap;
  for (auto &x : factors ) {
    T0 = util::getWallTime();
    std::vector<const Array*> arraysInFactor;
    calculateArrayReferences(x, arraysInFactor);
    // Going to use this as the "fresh" expression for the Query() invocation below
    assert(x.exprs.size() >= 1 && "No null/empty factors");
    if (arraysInFactor.size() == 0){
      continue;
    }
    ConstraintManager tmp(x.exprs);
    std::vector<std::vector<unsigned char> > tempValues;
    //printf("Solver DBG: %lf seconds setting up for call to computeInitialValues \n", util::getWallTime() - T0);
    if (false) {
      printf("--Printing Query: \n");
      outs() << "Constraints: \n";
      //printf("Constraints: \n");
      for (const auto &it : tmp ) {
	it->print(outs());
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
      return false;
    } else if (!hasSolution){
      values.clear();
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
          ::DenseSet<unsigned> * ds = &(x.elements[arraysInFactor[i]]);
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
      printf("Solver DBG3: %lf seconds cleaning up after getInitialValues call \n", util::getWallTime() - T0);
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
  //  delete factors;
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
