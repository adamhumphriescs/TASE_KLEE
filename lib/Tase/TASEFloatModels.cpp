//This file contains models for soft float implementation functions.  The idea is that we can
//intercept these function calls in TASE, and concretize the operands before the
//functions are called.

#include "../Core/Executor.h"
#include "../Core/Context.h"
#include "../Core/CoreStats.h"
#include "../Core/ExternalDispatcher.h"
#include "../Core/ImpliedValue.h"
#include "../Core/Memory.h"
#include "../Core/MemoryManager.h"
#include "../Core/PTree.h"
#include "../Core/Searcher.h"
#include "../Core/SeedInfo.h"
#include "../Core/SpecialFunctionHandler.h"
#include "../Core/StatsTracker.h"
#include "../Core/TimingSolver.h"
#include "../Core/UserSearcher.h"
#include "../Core/ExecutorTimerInfo.h"

#include "klee/ExecutionState.h"
#include "klee/Expr.h"
#include "klee/Interpreter.h"
#include "klee/TimerStatIncrementer.h"
#include "klee/CommandLine.h"
#include "klee/Common.h"
#include "klee/util/Assignment.h"
#include "klee/util/ExprPPrinter.h"
#include "klee/util/ExprSMTLIBPrinter.h"
#include "klee/util/ExprUtil.h"
#include "klee/util/GetElementPtrTypeIterator.h"
#include "klee/Config/Version.h"
#include "klee/Internal/ADT/KTest.h"
#include "klee/Internal/ADT/RNG.h"
#include "klee/Internal/Module/Cell.h"
#include "klee/Internal/Module/InstructionInfoTable.h"
#include "klee/Internal/Module/KInstruction.h"
#include "klee/Internal/Module/KModule.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/Internal/Support/FloatEvaluation.h"
#include "klee/Internal/Support/ModuleUtil.h"
#include "klee/Internal/System/Time.h"
#include "klee/Internal/System/MemoryUsage.h"
#include "klee/SolverStats.h"

#include "llvm/IR/Function.h"
#include "llvm/IR/Attributes.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/TypeBuilder.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace klee;

//#include "../../../test/proj_defs.h"
#include "tase_interp.h"
#include <iostream>
#include "klee/CVAssignment.h"
#include "klee/util/ExprUtil.h"
#include "klee/Constraints.h"
//#include "tase/TASEControl.h"
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <errno.h>
#include <cxxabi.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <netdb.h>
#include <fcntl.h>
#include <fstream>

#include <byteswap.h>


//extern void tase_exit();

extern uint64_t total_interp_returns;
extern bool taseManager;
extern tase_greg_t * target_ctx_gregs;
extern klee::Interpreter * GlobalInterpreter;
extern MemoryObject * target_ctx_gregs_MO;
extern ObjectState * target_ctx_gregs_OS;
extern ExecutionState * GlobalExecutionStatePtr;
extern bool gprsAreConcrete();

extern uint64_t interpCtr;
//extern void printCtx(tase_greg_t *);
extern void * rodata_base_ptr;
extern uint64_t rodata_size;

#include "TASESoftFloatEmulation.h"

//This union "regBits" is a trick for easily accessing the 64 bits
//in a general purpose register as different types, including
// narrower types such as int and float.
union regBits {
  uint64_t bits;
  double asDouble;
  float asFloat;
  int asInt;
  unsigned int asUnsignedInt;
  long asLong;
  long long asLongLong;
  unsigned asUnsigned;
  unsigned long asUnsignedLong;
  unsigned long long asUnsignedLongLong;
};


//Enum "functype" represents return type and arg type of
//different functions.  E.g, "ddi" is a function that
//returns double and takes a double and int arg, "f_ull",
//returns float given unsigned long long.
//"f" = float
//"d" = double
//"i" = int
//"l" = long
//"ll" = long long
//"ui" = unsigned int
//"ul" = unsigned long
//"ull" = unsigned long long
enum funcType {
	       fff, fdd, dff, ddd, iff, idd, ffi, ddi, ff, dd, fd, df,
	       if_, id, lf, ld, ll_f,ll_d, ui_f, ui_d, ul_f, ul_d, ull_f, ull_d,
	       fi, di, fl, dl, f_ll, d_ll, f_ui, d_ui, f_ul, d_ul, f_ull, d_ull,
	       
	       
};

union inputFn {
  float (*fff) (float f1, float f2);
  float (*fdd) (double d1, double d2);
  double (*dff) (float f1, float f2);
  double (*ddd) (double d1, double d2);
  int (*iff) (float f1, float f2);
  int (*idd) (double d1, double d2);
  float (*ffi) (float f1, int i1);
  double (*ddi) (double d1, int i1);
  float (*ff) (float f1);
  double (*dd) (double d1);
  float (*fd) (double d1);
  double (*df) (float f1);
  int (*if_) (float f1);
  int (*id ) (double d1);
  long (*lf) (float f1);
  long (*ld) (double d1);
  long long (*ll_f) (float f1);
  long long (*ll_d) (double d1);
  unsigned int (*ui_f) (float f1);
  unsigned int (*ui_d) (double d1);
  unsigned long (*ul_f) (float f1);
  unsigned long (*ul_d) (double d1);
  unsigned long long (*ull_f) (float f1);
  unsigned long long (*ull_d) (double d1);
  float (*fi) (int i1);
  double (*di) (int i1);
  float (*fl) (long l1);
  double (*dl) (long l1);
  float (*f_ll) (long long ll1);
  double (*d_ll) (long long ll1);
  float (*f_ui) (unsigned int ui1);
  double (*d_ui) (unsigned int ui1);
  float (*f_ul) (unsigned long ul1);
  double (*d_ul) (unsigned long ul1);
  float (*f_ull) (unsigned long long ull1);
  double (*d_ull) (unsigned long long ull1);
  
};

void Executor::model_float_uni_op(std::string name, inputFn F, funcType FT) {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if (taseFloatDebug)
    printf("Entering model_float_uni_op for %s \n", name.c_str());
  

  if (isa<ConstantExpr>(arg1Expr)) {
    regBits rv, a1;
    a1.bits = target_ctx_gregs[GREG_RDI].u64;

    switch (FT) {
    case ff:{
      float res = F.ff(a1.asFloat);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case dd: {
      double res = F.dd(a1.asDouble);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case fd: {
      float res = F.fd(a1.asDouble);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case df: {
      double res = F.df(a1.asFloat);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case if_: {
      int res = F.if_(a1.asFloat);
      rv.asInt = res;
      if (taseFloatDebug) {
	printf("Returning int %d with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case id: {
      int res = F.id(a1.asDouble);
      rv.asInt = res;
      if (taseFloatDebug) {
	printf("Returning int %d with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case lf: {
      long res = F.lf(a1.asFloat);
      rv.asLong = res;
      if (taseFloatDebug) {
	printf("Returning long %ld with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case ld: {
      long res = F.ld(a1.asDouble);
      rv.asLong = res;
      if (taseFloatDebug) {
	printf("Returning long %ld with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case ll_f: {
      long long res = F.ll_f(a1.asFloat);
      rv.asLongLong = res;
      if (taseFloatDebug) {
	printf("returning long long %lld with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case ll_d: {
      long long res = F.ll_d(a1.asDouble);
      rv.asLongLong = res;
      if (taseFloatDebug) {
	printf("Returning long long %lld with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case ui_f: {
      unsigned int res = F.ui_f(a1.asFloat);
      rv.asUnsignedInt = res;
      if (taseFloatDebug) {
	printf("Returning unsigned int %u with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case ui_d: {
      unsigned int res = F.ui_d(a1.asDouble);
      rv.asUnsignedInt = res;
      if (taseFloatDebug) {
	printf("Returning unsigned int %u with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case ul_f: {
      unsigned long res = F.ul_f(a1.asFloat);
      rv.asUnsignedLong = res;
      if (taseFloatDebug) {
	printf("Returing unsigned long %lu with float arg %f \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case ul_d: {
      unsigned long res = F.ul_d(a1.asDouble);
      rv.asUnsignedLong = res;
      if (taseFloatDebug) {
	printf("Returing unsigned long %lu with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case ull_f: {
      unsigned long long res = F.ull_f(a1.asFloat);
      rv.asUnsignedLongLong = res;
      if (taseFloatDebug) {
	printf("Returning unsigned long long %llu with double arg %lf \n", res, a1.asFloat);
	fflush(stdout);
      }
    }
      break;
    case ull_d: {
      unsigned long long res = F.ull_d(a1.asDouble);
      rv.asUnsignedLongLong = res;
      if (taseFloatDebug) {
	printf("Returning unsigned long long %llu with double arg %lf \n", res, a1.asDouble);
	fflush(stdout);
      }
    }
      break;
    case fi: {
      float res = F.fi(a1.asInt);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with int arg %d \n", res, a1.asInt);
	fflush(stdout);
      }
    }
      break;
    case di: {
      double res = F.di(a1.asInt);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with int arg %d  \n", res, a1.asInt);
	fflush(stdout);
      }
    }
      break;
    case fl: {
      float res = F.fl(a1.asLong);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with long arg %ld \n", res, a1.asLong);
	fflush(stdout);
      }
    }
      break;
    case dl: {
      double res = F.dl(a1.asLong);
      rv.asDouble = F.dl(a1.asLong);
      if (taseFloatDebug) {
	printf("Returing double %lf with long arg %ld \n", res, a1.asLong);
	fflush(stdout);
      }
    }
      break;
    case f_ll: {
      float res = F.f_ll(a1.asLongLong);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returing float %f with long long arg %lld \n", res, a1.asLongLong);
	fflush(stdout);
      }
    }
      break;
    case d_ll: {
      double res = F.d_ll(a1.asLongLong);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with long long arg %lld \n", res, a1.asLongLong);
	fflush(stdout);
      }
    }
      break;
    case f_ui: {
      float res = F.f_ui(a1.asUnsignedInt);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with unsigned int arg %u \n", res, a1.asUnsignedInt);
	fflush(stdout);
      }
    }
      break;
    case d_ui: {
      double res = F.d_ui(a1.asUnsignedInt);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returing double %lf with unsigned int arg %u \n", res, a1.asUnsignedInt);
	fflush(stdout);
      }
    }
      break;
    case f_ul: {
      float res = F.f_ul(a1.asUnsignedLong);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with unsigned long arg %lu \n", res, a1.asUnsignedLong);
	fflush(stdout);
      }
    }
      break;
    case d_ul: {
      double res = F.d_ul(a1.asUnsignedLong);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returing double %lf with unsigned long arg %lu \n", res, a1.asUnsignedLong);
	fflush(stdout);
      }
    }
      break;
    case f_ull: {
      float res = F.f_ull(a1.asUnsignedLongLong);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with unsigned long long arg %llu \n", res, a1.asUnsignedLongLong);
	fflush(stdout);
      }
    }
      break;
    case d_ull: {
      double res = F.d_ull(a1.asUnsignedLongLong);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returing double %lf with unsigned long long arg %llu \n", res, a1.asUnsignedLongLong);
	fflush(stdout);
      }
    }
      break;
    default:
      printf("ERROR in model_float_uni_op: Unrecognized arg type \n");
      std::cout.flush();
      std::exit(EXIT_FAILURE);
    }

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) rv.bits, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();
    
  } else {
    printf("Symbolic argument provided as input for a soft float emulation function. \n");
    concretizeGPRArgs(1,name.c_str());
    model_float_uni_op(name, F, FT);
  }
}

void Executor::model_float_bin_op(std::string name, inputFn F, funcType FT) {

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  if (taseFloatDebug)
    printf("Entering model_float_bin_op for %s \n", name.c_str());
  
  if ((isa<ConstantExpr>(arg1Expr)) &&
      (isa<ConstantExpr>(arg2Expr)) ) {

    regBits a1, a2, rv;
    a1.bits = target_ctx_gregs[GREG_RDI].u64;
    a2.bits = target_ctx_gregs[GREG_RSI].u64;
    
    switch (FT) {

    case fff: {
      float res = F.fff(a1.asFloat, a2.asFloat);
      rv.asFloat =res;
      if (taseFloatDebug) {
	printf("Returning float %f with float args %f %f \n", res, a1.asFloat, a2.asFloat);
	fflush(stdout);
      }
      break;
    }
    case fdd: {
      float res = F.fdd(a1.asDouble, a2.asDouble);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with double args %lf %lf \n", res, a1.asDouble, a2.asDouble);
	fflush(stdout);
      }
      break;
    }
    case dff: {
      double res = F.dff(a1.asFloat, a2.asFloat);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with float args %f %f \n", res, a1.asFloat, a2.asFloat);
	fflush(stdout);
      }
      break;
    }
    case ddd: {
      double res = F.ddd(a1.asDouble, a2.asDouble);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with double args %lf %lf \n", res, a1.asDouble, a2.asDouble);
	fflush(stdout);
      }
      break;
    }
    case iff: {
      int res = F.iff(a1.asFloat, a2.asFloat);
      rv.asInt = res;
      if (taseFloatDebug) {
	printf("Returning int %d with float args %f %f \n", res, a1.asFloat, a2.asFloat);
	fflush(stdout);
      }
    }
      break;
    case idd: {
      int res = F.idd(a1.asDouble, a2.asDouble);
      rv.asInt = res;
      if (taseFloatDebug) {
	printf("Returning int %d with double args %lf and %lf \n", res, a1.asDouble, a2.asDouble);
	fflush(stdout);
      }
    }
      break;
    case ffi: {
      float res = F.ffi(a1.asFloat, a2.asInt);
      rv.asFloat = res;
      if (taseFloatDebug) {
	printf("Returning float %f with float arg %f and int arg %d \n", res, a1.asFloat, a2.asInt);
	fflush(stdout);
      }
    }
      break;
    case ddi: {
      double res = F.ddi(a1.asDouble, a2.asInt);
      rv.asDouble = res;
      if (taseFloatDebug) {
	printf("Returning double %lf with double arg %lf and int arg %d \n", res, a1.asDouble, a2.asInt);
	fflush(stdout);
      }
    }
      break;
    default:
      printf("ERROR in model_float_bin_op: Unrecognized arg type \n");
      std::cout.flush();
      std::exit(EXIT_FAILURE);
      
    }

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) rv.bits, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();
    
  } else {
    printf("Symbolic argument provided as input for a soft float emulation function. \n");
    concretizeGPRArgs(2,name.c_str());
    model_float_bin_op(name, F, FT);
  }
}


void Executor::model__addsf3() {
  inputFn fn;
  fn.fff = __addsf3;
  model_float_bin_op("__addsf3", fn, fff);
}


void Executor::model__adddf3() {
  inputFn fn;
  fn.ddd = __adddf3;
  model_float_bin_op("__adddf3",fn,ddd);
}

void Executor::model__subsf3() {
  inputFn fn;
  fn.fff = __subsf3;
  model_float_bin_op("__subsf3", fn, fff);
}

void Executor::model__subdf3() {
  inputFn fn;
  fn.ddd = __subdf3;
  model_float_bin_op("__subdf3", fn, ddd);
}

void Executor::model__mulsf3() {
  inputFn fn;
  fn.fff = __mulsf3;
  model_float_bin_op("__mulsf3", fn, fff);
}

void Executor::model__muldf3() {
  inputFn fn;
  fn.ddd = __muldf3;
  model_float_bin_op("__muldf3", fn, ddd);
}

void Executor::model__divsf3() {
  inputFn fn;
  fn.fff = __divsf3;
  model_float_bin_op("__divsf3", fn, fff);
}

void Executor::model__divdf3() {
  inputFn fn;
  fn.ddd = __divdf3;
  model_float_bin_op("__divdf3", fn, ddd);
}

void Executor::model__negsf2() {
  inputFn fn;
  fn.ff = __negsf2;
  model_float_uni_op("__negsf2", fn, ff);
}

void Executor::model__negdf2() {
  inputFn fn;
  fn.dd = __negdf2;
  model_float_uni_op("__negdf2", fn, dd);
}

//Conversion

void Executor::model__extendsfdf2() {
  inputFn fn;
  fn.df = __extendsfdf2;
  model_float_uni_op("__extendsfdf2", fn, df);
}

void Executor::model__truncdfsf2() {
  inputFn fn;
  fn.fd = __truncdfsf2;
  model_float_uni_op("__truncdfsf2", fn, fd);
}

void Executor::model__fixsfsi() {
  inputFn fn;
  fn.if_ = __fixsfsi;
  model_float_uni_op("__fixsfsi", fn, if_);
}

void Executor::model__fixdfsi() {
  inputFn fn;
  fn.id = __fixdfsi;
  model_float_uni_op("__fixdfsi", fn, id);
}

void Executor::model__fixsfdi() {
  inputFn fn;
  fn.lf = __fixsfdi;
  model_float_uni_op("__fixsfdi", fn, lf);
}
void Executor::model__fixdfdi() {
  inputFn fn;
  fn.ld = __fixdfdi;
  model_float_uni_op("__fixdfdi", fn, ld);
}
void Executor::model__fixsfti() {
  inputFn fn;
  fn.ll_f = __fixsfti;
  model_float_uni_op("__fixsfti", fn, ll_f);
}
void Executor::model__fixdfti() {
  inputFn fn;
  fn.ll_d = __fixdfti;
  model_float_uni_op("__fixdfti", fn, ll_d);
}
void Executor::model__fixunssfsi() {
  inputFn fn;
  fn.ui_f = __fixunssfsi;
  model_float_uni_op("__fixunssfsi", fn, ui_f);
}
void Executor::model__fixunsdfsi() {
  inputFn fn;
  fn.ui_d = __fixunsdfsi;
  model_float_uni_op("__fixunsdfsi", fn, ui_d);
}
void Executor::model__fixunssfdi() {
  inputFn fn;
  fn.ul_f = __fixunssfdi;
  model_float_uni_op("__fixunssfdi", fn, ul_f);
}
void Executor::model__fixunsdfdi() {
  inputFn fn;
  fn.ul_d = __fixunsdfdi;
  model_float_uni_op("__fixunsdfdi", fn, ul_d);
}
void Executor::model__fixunssfti() {
  inputFn fn;
  fn.ull_f = __fixunssfti;
  model_float_uni_op("__fixunssfti", fn, ull_f);
}
void Executor::model__fixunsdfti() {
  inputFn fn;
  fn.ull_d = __fixunsdfti;
  model_float_uni_op("__fixunsdfti", fn, ull_d);
}
void Executor::model__floatsisf() {
  inputFn fn;
  fn.fi = __floatsisf;
  model_float_uni_op("__floatsisf", fn, fi);
}
void Executor::model__floatsidf() {
  inputFn fn;
  fn.di = __floatsidf;
  model_float_uni_op("__floatsidf", fn, di);
}
void Executor::model__floatdisf() {
  inputFn fn;
  fn.fl = __floatdisf;
  model_float_uni_op("__floatdisf", fn, fl);
}
void Executor::model__floatdidf() {
  inputFn fn;
  fn.dl = __floatdidf;
  model_float_uni_op("__floatdidf", fn, dl);
}
void Executor::model__floattisf() {
  inputFn fn;
  fn.f_ll = __floattisf;
  model_float_uni_op("__floattisf", fn, f_ll);
}
void Executor::model__floattidf() {
  inputFn fn;
  fn.d_ll = __floattidf;
  model_float_uni_op("__floattidf", fn, d_ll);
}
void Executor::model__floatunsisf() {
  inputFn fn;
  fn.f_ui = __floatunsisf;
  model_float_uni_op("__floatunsisf", fn, f_ui);
}
void Executor::model__floatunsidf() {
  inputFn fn;
  fn.d_ui = __floatunsidf;
  model_float_uni_op("__floatunsidf", fn, d_ui);
}
void Executor::model__floatundisf() {
  inputFn fn;
  fn.f_ul = __floatundisf;
  model_float_uni_op("__floatundisf", fn , f_ul);
}
void Executor::model__floatundidf() {
  inputFn fn;
  fn.d_ul = __floatundidf;
  model_float_uni_op("__floatundidf", fn, d_ul);
}
void Executor::model__floatuntisf() {
  inputFn fn;
  fn.f_ull = __floatuntisf;
  model_float_uni_op("__floatuntisf", fn, f_ull);
}
void Executor::model__floatuntidf() {
  inputFn fn;
  fn.d_ull = __floatuntidf;
  model_float_uni_op("__floatuntidf", fn, d_ull);
}

//Comparison
void Executor::model__cmpsf2() {
  inputFn fn;
  fn.iff = __cmpsf2;
  model_float_bin_op("__cmpsf2", fn, iff);
}
void Executor::model__cmpdf2() {
  inputFn fn;
  fn.idd = __cmpdf2;
  model_float_bin_op("__cmpdf2", fn, idd);
}
void Executor::model__unordsf2() {
  inputFn fn;
  fn.iff = __unordsf2;
  model_float_bin_op("__unordsf2", fn, iff);
}
void Executor::model__unorddf2() {
  inputFn fn;
  fn.idd = __unorddf2;
  model_float_bin_op("__unorddf2", fn, idd);
}
void Executor::model__eqsf2() {
  inputFn fn;
  fn.iff = __eqsf2;
  model_float_bin_op("__eqsf2", fn, iff);
}
void Executor::model__eqdf2() {
  inputFn fn;
  fn.idd = __eqdf2;
  model_float_bin_op("__eqdf2", fn, idd);
}
void Executor::model__nesf2() {
  inputFn fn;
  fn.iff = __nesf2;
  model_float_bin_op("__nesf2", fn, iff);
}
void Executor::model__nedf2() {
  inputFn fn;
  fn.idd = __nedf2;
  model_float_bin_op("__nedf2", fn, idd);
}
void Executor::model__gesf2() {
  inputFn fn;
  fn.iff = __gesf2;
  model_float_bin_op("__gesf2", fn, iff);
}
void Executor::model__gedf2() {
  inputFn fn;
  fn.idd = __gedf2;
  model_float_bin_op("__gedf2", fn, idd);
}
void Executor::model__ltsf2() {
  inputFn fn;
  fn.iff = __ltsf2;
  model_float_bin_op("__ltsf2", fn, iff);
}
void Executor::model__ltdf2() {
  inputFn fn;
  fn.idd = __ltdf2;
  model_float_bin_op("__ltdf2", fn, idd);
}
void Executor::model__lesf2() {
  inputFn fn;
  fn.iff = __lesf2;
  model_float_bin_op("__lesf2", fn, iff);
}
void Executor::model__ledf2() {
  inputFn fn;
  fn.idd = __ledf2;
  model_float_bin_op("__ledf2", fn, idd);
}
void Executor::model__gtsf2() {
  inputFn fn;
  fn.iff = __gtsf2;
  model_float_bin_op("__gtsf2", fn, iff);
}
void Executor::model__gtdf2() {
  inputFn fn;
  fn.idd = __gtdf2;
  model_float_bin_op("__gtdf2", fn, idd);
}

//Other
void Executor::model__powisf2() {
  inputFn fn;
  fn.ffi = __powisf2;
  model_float_bin_op("__powisf2", fn, ffi);
}
void Executor::model__powidf2() {
  inputFn fn;
  fn.ddi = __powidf2;
  model_float_bin_op("__powidf2", fn, ddi);
}

