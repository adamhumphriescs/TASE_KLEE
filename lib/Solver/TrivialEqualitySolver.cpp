/*
Trivial equality solver for simple equality queries
*/

//Looks for query of the Form EQ Constant, (Read w8 idx array_name)
//For example, (Eq 32 (Read w8 104 stdin_3))

#include "klee/Solver.h"
                                                                                                                        #include "klee/Constraints.h"
#include "klee/Expr.h"
#include "klee/IncompleteSolver.h"
#include "klee/util/ExprEvaluator.h"
#include "klee/util/ExprRangeEvaluator.h"
#include "klee/util/ExprVisitor.h"
// FIXME: Use APInt.
#include "klee/Internal/Support/Debug.h"
#include "klee/Internal/Support/IntEvaluation.h"

#include "llvm/Support/raw_ostream.h"
#include <sstream>
#include <cassert>
#include <map>
#include <vector> 

using namespace klee;

class TrivialEqSolver : public IncompleteSolver {
public:
  TrivialEqSolver();
  ~TrivialEqSolver();

  bool computeInitialValues(const Query&,                                                                                                           const std::vector<const Array*> &objects,                                                                               std::vector< std::vector<unsigned char> > &values,                                                                      bool &hasSolution);
  
};

bool computeInitialValues(const Query&,const std::vector<const Array*> &objects,std::vector< std::vector<unsigned char> > &values, bool &hasSolution) {

  if ( query.constraints.size() == 1) {
    ref<Expr> c = *(query.constraints.begin());
    if (klee::EqExpr * e = dyn_cast<klee::EqExpr>(c)) {
      ref<Expr> left = e->getKid(0);
      if (klee::ConstantExpr * ve = dyn_cast<klee::ConstantExpr>(left)) {
	unsigned char value = ve->getZExtValue(8);
	//TODO: Width check?  Expr.h suggests read exprs represent 1 byte. Do others exist?
	ref<Expr> right = e->getKid(1);
	if (klee::ReadExpr * re = dyn_cast<klee::ReadExpr> (right)) {
	  const Array *array = re->updates.root;  //Reliable way to get array name?
	  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(re->index)) {
	    unsigned index = CE->getZExtValue(32);
	    //printf("TRIV DBG: Read is at index %u \n", index);
	    //fflush(stdout);
	    values = std::vector<std::vector<unsigned char>> (1);
	    values[0] = std::vector<unsigned char> (array->size);
	    values[0][index] = (unsigned char) value;
	    //printf("TRIV DBG: Trivial assignment is %u", value);
	    //fflush(stdout);
	    hasSolution = true;
	    return true;
	  }
	}
      }
    }
  } else {
    return solver->impl->computeInitialValues(query, objects, values,
					      hasSolution);
  }   
}

Solver *klee::createTrivialEqSolver(Solver *s) {
  return new Solver(new StagedSolverImpl(new TrivialEqSolver(), s));
}
