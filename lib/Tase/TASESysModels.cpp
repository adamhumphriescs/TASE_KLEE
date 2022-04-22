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

//AH: Our additions below. --------------------------------------
#include "../../../test/proj_defs.h"
#include "../../../test/tase/include/tase/tase_interp.h"
#include <iostream>
#include "klee/CVAssignment.h"
#include "klee/util/ExprUtil.h"
#include "klee/Constraints.h"
#include "tase/TASEControl.h"
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
#include <regex>
#include <sys/ioctl.h>

//#include "../../../musl/arch/x86_64/pthread_arch.h"
//#include "../../../musl/src/internal/pthread_impl.h"

extern uint64_t interpCtr;
extern uint64_t rodata_size;
extern uint16_t poison_val;
extern tase_greg_t * target_ctx_gregs;
extern klee::Interpreter * GlobalInterpreter;
extern MemoryObject * target_ctx_gregs_MO;
extern ObjectState * target_ctx_gregs_OS;
extern ExecutionState * GlobalExecutionStatePtr;
extern void * rodata_base_ptr;
extern std::stringstream worker_ID_stream;

uint64_t native_ret_off = 0;

extern bool taseDebug;
extern bool bufferGuard;
extern bool skipFree;
extern bool taseManager;
extern bool noLog;

extern bool gprsAreConcrete();
extern void tase_exit();
extern void printCtx(tase_greg_t *);
extern void worker_exit();
extern bool tase_buf_has_taint(void * addr, int size);

bool roundUpHeapAllocations = true; //Round the size Arg of malloc, realloc, and calloc up to a multiple of 8
//This matters because it controls the size of the MemoryObjects allocated in klee.  Reads executed by
//some library functions (ex memcpy) may copy 4 or 8 byte-chunks of data at a time that cross over the edge
//of memory object buffers that aren't 4 or 8 byte aligned.

std::map<void *, void *> heap_guard_map; //

template<typename T, bool U>
struct as_helper;

template<typename T> T _as(tase_greg_t t);

template<typename T>
struct as_helper<T, true> {
  static T conv(tase_greg_t t){return (T) t.u64;}
};

template<typename T>
struct as_helper<T, false> {
  static T conv(tase_greg_t t){return _as<T>(t);}
};

template<typename T> T as(tase_greg_t t){return as_helper<T, std::is_pointer<T>::value>::conv(t);}

template<> uint64_t _as(tase_greg_t t){return t.u64;}
template<> int64_t _as(tase_greg_t t){return t.i64;}
template<> uint32_t _as(tase_greg_t t){return t.u32;}
template<> int32_t _as(tase_greg_t t){return t.i32;}
template<> int16_t _as(tase_greg_t t){return t.i16;}
template<> uint16_t _as(tase_greg_t t){return t.u16;}
template<> double _as(tase_greg_t t){return t.dbl;}
template<> char _as(tase_greg_t t){return (char) t.u8;}

#define _LOG std::cout << "Entering " << __func__ << " at interpCtr " << interpCtr << std::endl;

void printBuf(FILE * f, void * buf, size_t count)
{
  fprintf(f,"Calling printBuf with count %d \n", count);
  fflush(f);
  for (size_t i = 0; i < count; i++) {
    fprintf(f,"%02x", *((uint8_t *) buf + i));
    fflush(f);
  }
  fprintf(f,"\n\n");
  fflush(f);
}

//Used to restore concrete values for buffers that are
//entirely made up of constant expressions
void Executor::rewriteConstants(uint64_t base, size_t size) {
  if (modelDebug) {
    std::cout << "Rewriting constant array" << std::endl;
  }

  //Fast path -- if no taint in buffer, can't have exprs
  if (!tase_buf_has_taint((void *) base, size)) {
    return;
  }
  
  if (!(
	base > ((uint64_t) rodata_base_ptr)
	 &&
	base < (((uint64_t) rodata_base_ptr) + rodata_size)
	)
      ) {
    if (modelDebug) {
      std::cout << "Base does not appear to be in rodata" << std::endl;
    }
  } else {
    if (modelDebug) {
      std::cout << "Found base in rodata.  Returning from rewriteConstants without doing anything" << std::endl;
    }
    return;
  }
  
  for (size_t i = 0; i < size; i++) {

    //We're assuming
    //1. Every byte's 2-byte aligned buffer containing it has been mapped with a MO/OS at some point.
    //2. It's OK to duplicate some of these read/write ops
    uint64_t writeAddr;
    if( (base + i) %2 != 0)
      writeAddr = base + i -1;
    else
      writeAddr = base + i;
    
    ref<Expr> val = tase_helper_read(writeAddr, 2);
    tase_helper_write(writeAddr, val);

  }
  if (modelDebug) {
    std::cout << "End result:" << std::endl;
    printBuf(stdout, (void *) base, size);
  }
}



void Executor::model_putchar(){
  if(!noLog){
    _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;

  char c;
  get_val(count, s_offset, __func__, c);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) putchar(c), Expr::Int64);
  tase_helper_write((uint64_t)&target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

//Todo -- figure out the endianness issue with
//copying 2 bytes at a time in the slow unaligned path
void Executor::model_memcpy_tase() {
  if (!noLog) {
    _LOG
  }
  double T0 = util::getWallTime();

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;
  void* dst;
  void* src;
  size_t s;
  get_vals(count, s_offset, __func__, dst, src, s);

  int zero = 0; //Force kill rcx -- Should be fine because it's caller-saved.
  ref<ConstantExpr> zeroExpr = ConstantExpr::create((uint64_t) zero, Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RCX], zeroExpr);

  if (isBufferEntirelyConcrete((uint64_t) dst, s) && isBufferEntirelyConcrete((uint64_t) src, s)) {
    rewriteConstants((uint64_t) dst, s);
    rewriteConstants((uint64_t) src, s);
    memcpy(dst, src, s);
  } else {

    //Fast path aligned case
    if ((((uint64_t) dst) %2 == 0) && (((uint64_t) src) % 2 == 0) && (((uint64_t) s) %2 == 0)) {
      for (uint i = 0; i < s; i++) {
	  
        if (i%2 == 0
            && *((uint16_t *) ((uint64_t) src + i) ) != poison_val
            && *((uint16_t *) ((uint64_t) dst + i) ) != poison_val
	      )
        {
          *((uint16_t *) ((uint64_t) dst + i) )  = *((uint16_t *) ((uint64_t) src + i) );
          i++;
        } else {

          ref <Expr> b = tase_helper_read((uint64_t) src + i, 1);
          tase_helper_write((uint64_t) dst+i, b);
	    
        }
      }

    } else {
      for (uint i = 0; i < s; i++) {
        ref <Expr> b = tase_helper_read((uint64_t) src + i, 1);
        tase_helper_write((uint64_t) dst+i, b);
      }
    }
  }
    
  double T1 = util::getWallTime();
  if (!noLog) {
    std::cout << "Memcpy took " << (T1-T0) << " seconds \n" << std::endl;
  }
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) dst, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();
}


//Todo -- Just rip it out?
//Eliminate dead and reserved registers in case they contain
//symbolic taint.  Used to help avoid interpreting through
//prohibitive functions.
void Executor::killDeadRegsPreCall() {
  ref<ConstantExpr> zeroExpr = ConstantExpr::create((uint64_t) 0, Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_R14].u64, zeroExpr);
}


//Utility function to fake x86_64 retq instruction
//at end of model.
void Executor::do_ret() {
  target_ctx_gregs[GREG_RIP].u64 = *((uint64_t*) target_ctx_gregs[GREG_RSP].u64);
  target_ctx_gregs[GREG_RSP].u64 += 8;
}



template<typename T>
void Executor::get_val(int& count, uint64_t* &s_offset, const std::string& reason, T& t){
  auto rr = reason + "\n";
  if(count < 6){
    ref<Expr> aref = target_ctx_gregs_OS->read(count < 4 ? (5-count)*8 : (4+count)*8, Expr::Int64);
    if(isa<ConstantExpr>(aref)){
      t = as<T>(target_ctx_gregs[count < 4 ? 5-count : 4+count]);
    } else {
      ref<Expr> aref2 = toConstant(*GlobalExecutionStatePtr, aref, rr.c_str());
      tase_helper_write((uint64_t) &target_ctx_gregs[count < 4 ? 5-count : 4+count].i64, aref2);
      t = as<T>(target_ctx_gregs[count < 4 ? 5-count : 4+count]);
    }
    ++count;
  } else {
    ref<Expr> aref = tase_helper_read((uint64_t) s_offset, 8);
    if(isa<ConstantExpr>(aref)){
      t = *((T*)s_offset);
    } else {
      ref<ConstantExpr> aref2 = toConstant(*GlobalExecutionStatePtr, aref, rr.c_str());
      tase_helper_write((uint64_t) s_offset, aref2);
    }
    ++s_offset;
  }
}


// template<typename T>
// void Executor::get_val_va(uint64_t* &s_offset, const std::string& reason, T& t){
//   auto rr = reason + "\n";
//   ref<Expr> aref = tase_helper_read((uint64_t) s_offset, 8);
//   if(isa<ConstantExpr>(aref)){
//     t = *((T*)s_offset);
//   } else {
//     ref<ConstantExpr> aref2 = toConstant(*GlobalExecutionStatePtr, aref, rr.c_str());
//     tase_helper_write((uint64_t) s_offset, aref2);
//   }
//   ++s_offset;
// }


template<typename U, typename... T>
void Executor::get_vals(int& count, uint64_t* &s_offset, const std::string& reason, U& u, T&... ts){
  get_val(count, s_offset, reason, u);
  get_vals(count, s_offset, reason, ts...);
}

template<typename... T>
void Executor::get_vals(int& count, uint64_t* &s_offset, const std::string& reason){
  return;
}

/*
uint64_t * get_val(int fpcount, uint64_t *s_offset, double& t, const char* reason){
  if(fpcount < 16){
    int idx = fpcount / 2;
    bool even = fpcount % 2 == 0
    auto ref = target_ctx_xmms_OS->read(idx*XMMREG_SIZE + (even ? 0 : 1), Expr::Int64); // Width given here, not a type
    if(isa<ConstantExpr>(ref)){
      t = target_ctx_xmms[idx][(even ? 0 : 1)];
    }
  } else {
    auto ref = tase_helper_read(s_offset);
    if(isa<ConstantExpr>(ref)){
      t = *s_offset;
    } else {
      auto ref2 = toConstant(*GlobalExecutionStatePtr, ref, reason);
      tase_helper_write((uint64_t) s_offset, ref2);
    }
    return s_offset + 8;
  }
  return s_offset;
}
*/


template<typename... Ts>
std::string Executor::model_printf_base_helper(int& count, uint64_t* &s_offset, const std::string& reason, char type, const std::string& ff, const std::string& out, Ts... ts){
  char outstr[255];

  switch(type){
    case 'd': //signed int
    case 'i':
    {
      // printf will down-convert (u)int64_t to whatever was specified in fmt string
      int64_t arg;
      get_val(count, s_offset, reason, arg);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;
    case 'u': //unsigned int
    case 'o':
    case 'x':
    case 'X':
      // same as above but unsigned
    {
      uint64_t arg;
      get_val(count, s_offset, reason, arg);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;
    case 'f': // fp - the difference in size matters here. Check if x[3] is L or not? for now just ignore - no long doubles allowed!
    case 'F':
    case 'e':
    case 'E':
    case 'g':
    case 'G':
    case 'a':
    case 'A':
    {
      double arg;
      get_val(count, s_offset, reason, arg);
      sprintf_helper( &outstr[0], ff, ts..., arg);
      //fpcount++;
    }
    break;

    case 'c': // char
    {
      char arg;
      get_val(count, s_offset, reason, arg);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;
    case 's': // char*
    {
      char* arg;
      get_val(count, s_offset, reason, arg);
      printf("printf get_val<char*>: %d, \"%s\"\n", ts..., arg);
      fflush(stdout);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;

    case 'n': // ptr to int, stores the # chars printed so far and elides the %n
      // save out.length() to the pointer
    {
      int* arg;
      get_val(count, s_offset, reason, arg);
      *arg = out.length();
    }
    break;
  }

  return type == 'n' ? "" : std::string(outstr);
}


template<typename T>
void Executor::sanitize_va_arg(T& t){
  ref<Expr> aref = tase_helper_read((uint64_t) &t, sizeof(T));
  if(isa<ConstantExpr>(aref)){
    return;
  } else {
    ref<ConstantExpr> aref2 = toConstant(*GlobalExecutionStatePtr, aref, "sanitize_va_arg\n");
    tase_helper_write((uint64_t) &t, aref2);
  }
}


template<typename... Ts>
std::string Executor::model_printf_base_helper_va(uint64_t* &s_offset, const std::string& reason, char type, const std::string& ff, const std::string& out, va_list lst, Ts... ts){
  char outstr[255];

  switch(type){
    case 'd': //signed int
    case 'i':
    {
      // printf will down-convert (u)int64_t to whatever was specified in fmt string
      int64_t arg = va_arg(lst, int64_t);
      sanitize_va_arg(arg);
      //get_val_va(s_offset, reason, arg);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;
    case 'u': //unsigned int
    case 'o':
    case 'x':
    case 'X':
      // same as above but unsigned
    {
      uint64_t arg = va_arg(lst, uint64_t);
      sanitize_va_arg(arg);
      //get_val_va(s_offset, reason, arg);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;
    case 'f': // fp - the difference in size matters here. Check if x[3] is L or not? for now just ignore - no long doubles allowed!
    case 'F':
    case 'e':
    case 'E':
    case 'g':
    case 'G':
    case 'a':
    case 'A':
    {
      double arg = va_arg(lst, double);
      sanitize_va_arg(arg);
      //get_val_va(s_offset, reason, arg);
      sprintf_helper( &outstr[0], ff, ts..., arg);
      //fpcount++;
    }
    break;

    case 'c': // char
    {
      char arg = va_arg(lst, char);
      sanitize_va_arg(arg);
      //get_val_va(s_offset, reason, arg);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;
    case 's': // char*
    {
      char* arg = va_arg(lst, char*);
      sanitize_va_arg(arg);
      //get_val_va(s_offset, reason, arg);
      printf("printf get_val<char*>: %d, \"%s\"\n", ts..., arg);
      fflush(stdout);
      sprintf_helper(&outstr[0], ff, ts..., arg);
    }
    break;

    case 'n': // ptr to int, stores the # chars printed so far and elides the %n
      // save out.length() to the pointer
    {
      int* arg = va_arg(lst, int*);
      sanitize_va_arg(arg);
      //get_val_va(s_offset, reason, arg);
      *arg = out.length();
    }
    break;
  }

  return type == 'n' ? "" : std::string(outstr);
}


void Executor::sprintf_helper(char* outstr, const std::string& ff, ...){
  va_list args;
  va_start(args, ff);
  vsprintf(outstr, ff.c_str(), args);
  va_end(args);
}



std::string Executor::model_printf_base(int& count, uint64_t* &s_offset, const std::string& reason){
  char * fmtc;
  get_val(count, s_offset, reason, fmtc);

  std::string fmt = std::string(fmtc);
  if(modelDebug){
    std::cout << reason << " with fmt string: \"" << fmt << "\"" << std::endl;
  }

  // possibly useful alternative for doubles:
  // check al
  // if al is zero, no fp args
  // else dump xmm 0-7 to array

  std::regex specifier("%([-+#0 ])?([0-9*])?(.[0-9]+|.[*])?(hh|h|l|ll|j|z|t|L)?([diouxXfFeEgGaAcspn])", std::regex::egrep);
  auto match_begin = std::sregex_iterator(fmt.begin(), fmt.end(), specifier);
  auto out = std::string();
  auto last = fmt.cbegin();
  for(auto it = match_begin; it != std::sregex_iterator(); ++it){
    auto x = *it;
    out += fmt.substr(last - fmt.begin(), x[0].first - last); // non-format characters up to current match
    last = x[5].second;

    char type = x[5].str()[0];
    std::string ff = x.str(0);

    int width;
    int precision;

    bool gw = x[2].str().find('*') != std::string::npos;
    bool gp = x[3].str().find('*') != std::string::npos;

    if(gw){
      get_val(count, s_offset, reason, width);
      std::cout << ff << " width: " << width << std::endl;
    }

    if(gp){
      get_val(count, s_offset, reason, precision);
      std::cout << ff << " precision: " << precision << std::endl;
    }

    out += gw ? (gp ? model_printf_base_helper(count, s_offset, reason, type, ff, out, width, precision)  :
                      model_printf_base_helper(count, s_offset, reason, type, ff, out, width)  ) :
                (gp ? model_printf_base_helper(count, s_offset, reason, type, ff, out, precision)  :
                      model_printf_base_helper(count, s_offset, reason, type, ff, out)  );
  }
  out += fmt.substr(last - fmt.begin(), fmt.end() - last);
  return out;
}


struct tase_va_list {
  uint32_t gp_offset;
  uint32_t fp_offset;
  uint64_t* overflow;
  uint64_t* reg;
};

std::string Executor::model_printf_base_va(int& count, uint64_t* &s_offset, const std::string& reason){
  char * fmtc;
  tase_va_list* lst;
  get_vals(count, s_offset, reason, fmtc, lst);

  std::string fmt = std::string(fmtc);
  if(modelDebug){
    std::cout << reason << " with fmt string: \"" << fmt << "\"" << std::endl;
  }

  // possibly useful alternative for doubles:
  // check al
  // if al is zero, no fp args
  // else dump xmm 0-7 to array

  std::regex specifier("%([-+#0 ])?([0-9*])?(.[0-9]+|.[*])?(hh|h|l|ll|j|z|t|L)?([diouxXfFeEgGaAcspn])", std::regex::egrep);
  auto match_begin = std::sregex_iterator(fmt.begin(), fmt.end(), specifier);
  auto out = std::string();
  auto last = fmt.cbegin();
  for(auto it = match_begin; it != std::sregex_iterator(); ++it){
    auto x = *it;
    out += fmt.substr(last - fmt.begin(), x[0].first - last); // non-format characters up to current match
    last = x[5].second;

    char type = x[5].str()[0];
    std::string ff = x.str(0);

    int width;
    int precision;

    bool gw = x[2].str().find('*') != std::string::npos;
    bool gp = x[3].str().find('*') != std::string::npos;

    if(gw){
      get_val_va(s_offset, reason, width);
      std::cout << ff << " width: " << width << std::endl;
    }

    if(gp){
      get_val_va(s_offset, reason, precision);
      std::cout << ff << " precision: " << precision << std::endl;
    }

    out += gw ? (gp ? model_printf_base_helper_va(s_offset, reason, type, ff, out, lst, width, precision)  :
                      model_printf_base_helper_va(s_offset, reason, type, ff, out, lst, width)  ) :
                (gp ? model_printf_base_helper_va(s_offset, reason, type, ff, out, lst, precision)  :
                      model_printf_base_helper_va(s_offset, reason, type, ff, out, lst)  );
  }
  out += fmt.substr(last - fmt.begin(), fmt.end() - last);
  return out;
}

// specifiers:
// %([-+#0 ])?([0-9*])?(.[0-9]+|.*)?(length)?(type)
// type/length table: see here https://cplusplus.com/reference/cstdio/printf/
// *-items -> extra arg given to fill in, precedes the value to be interpolated
// abi reference: https://www.intel.com/content/dam/develop/external/us/en/documents/mpx-linux64-abi.pdf
// printf(const char * fmt, ...)
void Executor::model_printf(){
  if(!noLog){
    _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;

  std::string out = model_printf_base(count, s_offset, __func__);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) printf("%s", out.c_str()), Expr::Int64);
  tase_helper_write((uint64_t)&target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}


void Executor::model_sprintf(){
  if(!noLog){
    _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;

  char* argout;
  get_val(count, s_offset, __func__, argout);

  std::string out = model_printf_base(count, s_offset, __func__);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) sprintf(argout, "%s", out.c_str()), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}


void Executor::model_fprintf(){
  if(!noLog){
     _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;

  FILE* argout;
  get_val(count, s_offset, __func__, argout);

  std::string out = model_printf_base(count, s_offset, __func__);
  printf("fprintf string: %s", out.c_str());
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) fprintf(argout, "%s", out.c_str()), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

void Executor::model_vsnprintf(){
  if(!noLog){
     _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;

  char * argout;
  size_t size;
  get_vals(count, s_offset, __func__, argout, size);

  std::string out = model_printf_base_va(count, s_offset, __func__);
  sprintf(argout, "%s", out.substr(0, size-1 <= out.size() ? size-1 : out.size()).c_str());
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) out.size(), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}


// sprintf but allocate a c str large enough, pass to char**. va_list
void Executor::model_vasprintf(){
  if(!noLog){
     _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;

  char ** argout;
  get_val(count, s_offset, __func__, argout);

  std::string out = model_printf_base_va(count, s_offset, __func__);

  char * outstr = (char*) calloc(1, (out.size()+1)*sizeof(char));
  tase_map_buf((uint64_t) outstr, (out.size()+1)*sizeof(char));
  argout = &outstr;
  strcpy(outstr, out.c_str());
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) out.size(), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}


void Executor::model_sigemptyset(){
  if(!noLog){
     _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  sigset_t * set;
  get_val(count, s_offset, __func__, set);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) sigemptyset(set), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

void Executor::model_sigfillset(){
  if(!noLog){
     _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  sigset_t * set;
  get_val(count, s_offset, __func__, set);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) sigfillset(set), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

void Executor::model_sigaddset(){
  if(!noLog){
     _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  sigset_t * set;
  int signum;
  get_vals(count, s_offset, __func__, set, signum);

  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) sigaddset(set, signum), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

void Executor::model_sigaction(){
  if(!noLog){
     _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int signum;
  struct sigaction * set;
  struct sigaction * oldset;
  get_vals(count, s_offset, __func__, signum, set, oldset);

  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) sigaction(signum, set, oldset), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}


//int sigprocmask(int how, const sigset_t *restrict set,
//                       sigset_t *restrict oldset);
void Executor::model_sigprocmask(){
  if(!noLog){
     _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int how;
  sigset_t * set;
  sigset_t * oldset;
  get_vals(count, s_offset, __func__, how, set, oldset);

  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) sigprocmask(how, set, oldset), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}


void Executor::model_gethostname(){
  if(!noLog){
    _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  char* name;
  size_t len;
  get_vals(count, s_offset, __func__, name, len);

  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) gethostname(name, len), Expr::Int64);
  tase_helper_write((int64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

// for samba, which calls once with a single int* param in varargs
void Executor::model_ioctl(){
  if (!noLog) {
     _LOG
  }

  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int fd;
  int request;
  int *value;
  get_vals(count, s_offset, __func__, fd, request, value);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) ioctl(fd, request, value), Expr::Int64);
  tase_helper_write((int64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}



extern int * __errno_location();

void Executor::model___errno_location() {
  if (modelDebug && !noLog) {
    _LOG
  }
  //Perform the call
  int * res = __errno_location();
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

  //If it doesn't exit, back errno with a memory object.
  ObjectPair OP;
  ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP)) {
    if (modelDebug && !noLog) {
      printf("errno var appears to have MO already backing it \n");
    }
  } else {
    if (modelDebug && !noLog) {
      printf("Creating MO to back errno at 0x%lx with size 0x%lx \n", (uint64_t) res, sizeof(int));
    }
    MemoryObject * newMO = addExternalObject(*GlobalExecutionStatePtr, (void *) res, sizeof(int), false);
    const ObjectState * newOSConst = GlobalExecutionStatePtr->addressSpace.findObject(newMO);
    ObjectState *newOS = GlobalExecutionStatePtr->addressSpace.getWriteable(newMO,newOSConst);
    newOS->concreteStore = (uint8_t *) res;
  }
  
  do_ret();//fake a ret
}



void Executor::model_exit() {

  std::cout << " Found call to exit.  TASE should shutdown." << std::endl;
  std::cout.flush();
  //Todo: Make a flag to only print round/pass for multipass
  //printf("IMPORTANT: Worker exiting from terminal path in round %d pass %d from model_exit \n", round_count, pass_count);
  std::cout.flush();
  worker_exit();
  std::exit(EXIT_SUCCESS);
}

//http://man7.org/linux/man-pages/man2/write.2.html
//ssize_t write(int fd, const void *buf, size_t count);
//This is NOT the model used for
//verification socket writes: see model_writesocket
void Executor::model_write() {
  //Just print the second arg for debugging.

  char * theBuf = (char *)  target_ctx_gregs[GREG_RSI].u64;
  size_t size = target_ctx_gregs[GREG_RDX].u64;
  if (modelDebug) {
    printf("Entering model_write \n");
    fflush(stdout);
    char printMe [size];
    strncpy (printMe, theBuf, size);
    printf("Found call to write.  Buf appears to be \"%s\" \n", printMe);
  }
  //Assume that the write succeeds 
  uint64_t res = target_ctx_gregs[GREG_RDX].u64;
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX].u64, resExpr);
  
  do_ret();//fake a ret

}

void Executor::model___printf_chk() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int flag;
  char* fmt;
  get_vals(count, s_offset, __func__, flag, fmt);
    //Ignore varargs for now and just print the second arg
  std::cout << "Second arg to __printf_chk is " <<  fmt << std::endl;

  ref<ConstantExpr> zeroResultExpr = ConstantExpr::create(0, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, zeroResultExpr);
    
  do_ret();
}

//https://man7.org/linux/man-pages/man3/isatty.3.html
//int isatty(int fd);

void Executor::model_isatty() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int fd;
  get_val(count, s_offset, __func__, fd);

  int res = isatty(fd);
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();
}



//https://linux.die.net/man/3/fileno
//int fileno(FILE *stream); 
void Executor::model_fileno() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  FILE* file;
  get_val(count, s_offset, __func__, file);
  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) fileno(file), Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();
}

//http://man7.org/linux/man-pages/man2/fcntl.2.html
//int fcntl(int fd, int cmd, ... /* arg */ );
void Executor::model_fcntl() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int fd;
  int cmd;
  int flag;
  get_vals(count, s_offset, __func__, fd, cmd, flag);

  if ( cmd == F_SETFL && flag == O_NONBLOCK) {
    std::cout << "fcntl call to set fd as nonblocking" << std::endl;
  } else {
    std::cout << "fcntl called with unmodeled args" << std::endl;
  }

  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) 0, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();
}


//http://man7.org/linux/man-pages/man2/stat.2.html
//int stat(const char *pathname, struct stat *statbuf);
//Todo: Make option to return symbolic result, and proprerly inspect input
void Executor::model_stat() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  char* pathname;
  struct stat* statbuf;
  get_vals(count, s_offset, __func__, pathname, statbuf);

  ref<ConstantExpr> resExpr = ConstantExpr::create((int64_t) stat(pathname, statbuf), Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();
}


//Just returns the current process's pid.  We can make this symbolic if we want later, or force a val
//that returns the same number regardless of worker forking.
void Executor::model_getpid() {
  int pid = getpid();
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) pid, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();

}

//uid_t getuid(void)
//http://man7.org/linux/man-pages/man2/getuid.2.html
//Todo -- determine if we should fix result, see if uid_t is ever > 64 bits
void Executor::model_getuid() {
  uid_t uidResult = getuid();
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) uidResult, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();//Fake a ret
  
}

//uid_t geteuid(void)
//http://man7.org/linux/man-pages/man2/geteuid.2.html
//Todo -- determine if we should fix result prior to forking, see if uid_t is ever > 64 bits
void Executor::model_geteuid() {
  uid_t euidResult = geteuid();
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) euidResult, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();//Fake a ret

}

//gid_t getgid(void)
//http://man7.org/linux/man-pages/man2/getgid.2.html
//Todo -- determine if we should fix result, see if gid_t is ever > 64 bits
void Executor::model_getgid() {
  gid_t gidResult = getgid();
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) gidResult, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();//Fake a ret

}

//gid_t getegid(void)
//http://man7.org/linux/man-pages/man2/getegid.2.html
//Todo -- determine if we should fix result, see if gid_t is ever > 64 bits
void Executor::model_getegid() {
  printf("Calling model_getegid() \n");
  gid_t egidResult = getegid();
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) egidResult, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

  do_ret();//Fake a ret

}

//char * getenv(const char * name)
//http://man7.org/linux/man-pages/man3/getenv.3.html
//Todo: This should be generalized, and also technically should inspect the input string's bytes
void Executor::model_getenv() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  char* name;
  get_val(count, s_offset, __func__, name);
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) getenv(name), Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();//Fake a ret
}





//time_t time(time_t *tloc);
// http://man7.org/linux/man-pages/man2/time.2.html
void Executor::model_time() {
  if(!noLog){
    _LOG
  }
  int count;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  time_t* tloc;
  get_val(count, s_offset, __func__, tloc);
  time_t res = time(tloc);

  if (!noLog) {
    char * timeString = ctime(tloc);
    std::cout << "timeString is " << timeString << std::endl;
    std::cout << "Size of timeVal is " << sizeof(time_t) << std::endl;
  }
    
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();
}



//struct tm *gmtime(const time_t *timep);
//https://linux.die.net/man/3/gmtime
void Executor::model_gmtime() {
  if (!noLog) {
    printf("Entering call to gmtime at interpCtr %lu \n", interpCtr);
  }
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  
  if  (
       (isa<ConstantExpr>(arg1Expr)) ) {
    //Do call
    struct tm * res = gmtime( (time_t *) target_ctx_gregs[GREG_RDI].u64);
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    char timeBuf[30];
    strftime(timeBuf, 30, "%Y-%m-%d %H:%M:%S", res);
    if (!noLog) {
      printf("gmtime result is %s \n", timeBuf);
    }

    
    //If it doesn't exit, back returned struct with a memory object.
    ObjectPair OP;
    ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP)) {
      printf("model_gmtime result appears to have MO already backing it \n");
      fflush(stdout);
      
    } else {
      if (!noLog) {
	printf("Creating MO to back tm at 0x%lx with size 0x%lx \n", (uint64_t) res, sizeof(struct tm));
      }

      MemoryObject * newMO = addExternalObject(*GlobalExecutionStatePtr, (void *) res, sizeof(struct tm), false);
      const ObjectState * newOSConst = GlobalExecutionStatePtr->addressSpace.findObject(newMO);
      ObjectState *newOS = GlobalExecutionStatePtr->addressSpace.getWriteable(newMO,newOSConst);
      newOS->concreteStore = (uint8_t *) res;
    }
    
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();//fake a ret
    
  } else {
    concretizeGPRArgs(1, "model_gmtime");
    model_gmtime();
  }

}

//int gettimeofday(struct timeval *tv, struct timezone *tz);
//http://man7.org/linux/man-pages/man2/gettimeofday.2.html
//Todo -- properly check contents of args for symbolic content, allow for symbolic returns
void Executor::model_gettimeofday() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) ) {

    //Do call
    int res = gettimeofday( (struct timeval *) target_ctx_gregs[GREG_RDI].u64, (struct timezone *) target_ctx_gregs[GREG_RSI].u64);
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();//fake a ret

  }  else {
    concretizeGPRArgs(2, "model_gettimeofday");
    model_gettimeofday();
  }
}

size_t roundUp(size_t input, size_t multiple) {

  if (input < 0 || multiple < 0) {
    printf("Check your implementation of round_up for negative vals \n");
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }
  
  if (input % multiple == 0)
    return input;
  else {
    size_t divRes = input/multiple;
    return (divRes +1)* multiple;
  }
}

//void *calloc(size_t nmemb, size_t size);
//https://linux.die.net/man/3/calloc

void Executor::model_calloc() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) ) {

    size_t nmemb = target_ctx_gregs[GREG_RDI].u64;
    size_t size  = target_ctx_gregs[GREG_RSI].u64;

    size_t initNmemb;
    if (bufferGuard) {
      initNmemb = nmemb;
      //Need a better way to adjust up for the extra bufferguard bytes for size > 1.  Maybe just
      //re-route call through malloc?
      
      nmemb += 4 + 16; // 4 bytes for  the psn, plus two 8-byte buffers between psn and the
      //allocated space.  Our poison checking in the compiler sometimes conservatively promotes
      //read/write checks (e.g., an 8 byte write to a 16 byte check) when it can't determine
      //alignment information, so the extra 8-byte buffers should help with that.
      if (taseDebug){
	printf("Requesting calloc of size %u for bufferGuard on heap \n",initNmemb * size);
      }
    }


    
    
    void * res = calloc(nmemb, size);

    
    
    size_t numBytes = size*nmemb;

    //Todo -- refactor and roundup work properly with and without bufferguard.
    //if (roundUpHeapAllocations)
    // numBytes = roundUp(numBytes,8);

    void * returnedBuf;
    if (bufferGuard) {
      returnedBuf = (void *) ((uint64_t) res + 2 + 8);
      //Need to be able to line up returned ptr vs actual buf later when free is called.
      if (taseDebug) {
	printf("bufferGuard obtained mapping on heap at 0x%lx \n", (uint64_t) res);
	printf("bufferGuard returning ptr at 0x%lx \n", (uint64_t) returnedBuf);
      }
      heap_guard_map.insert(std::pair<void *, void *>(returnedBuf, res));

    } else {
      returnedBuf = res;
    }




    if (bufferGuard) {
      tase_map_buf((uint64_t) returnedBuf, initNmemb * size + 6);
      //Poison edges around buffer
      * ((uint16_t *) (res)) = poison_val;
      void * endAddr =(void *)( (uint64_t) res + 2 + 8 + (initNmemb * size)  + 8);
      *((uint16_t *) (endAddr)) = poison_val;
    } else {
      tase_map_buf((uint64_t) returnedBuf, numBytes);
    }


    
    if (!noLog) {
      printf("calloc at 0x%lx for 0x%lx bytes \n", (uint64_t) res, numBytes);
    }

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) returnedBuf, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();//fake a ret
    
  } else {
    concretizeGPRArgs(2, "model_calloc");
    model_calloc();
  }    
}



//void *realloc(void *ptr, size_t size);
//https://linux.die.net/man/3/realloc
//Todo: Set up additional memory objects if realloc adds extra space
void Executor::model_realloc() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  if (modelDebug) {
    printf("Calling model_realloc \n");
  }
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) ) {

    if (bufferGuard) {
      void * ptrIn = (void *) target_ctx_gregs[GREG_RDI].u64;
      size_t sizeIn = (size_t) target_ctx_gregs[GREG_RSI].u64;

      printf("Attempting to call realloc with buffer guards enabled\n");
      fflush(stdout);


      if (ptrIn == NULL) {
	//handle this case like a malloc
	size_t size =  2 + 8 + sizeIn + 8 + 2;
	void * res = malloc(size);
	void * returnedBuf = (void *) ((uint64_t) res + 2 + 8);
	//Need to be able to line up returned ptr vs actual buf later when free is called.
	if (taseDebug) {
	  printf("bufferGuard obtained mapping on heap at 0x%lx \n", (uint64_t) res);
	  printf("bufferGuard returning ptr at 0x%lx \n", (uint64_t) returnedBuf);
	}
	heap_guard_map.insert(std::pair<void *, void *>(returnedBuf, res));

	tase_map_buf((uint64_t) returnedBuf, sizeIn + 6);
	//Poison edges around buffer
	* ((uint16_t *) (res)) = poison_val;
	void * endAddr =(void *)( (uint64_t) res + 2 + 8 + (sizeIn)  + 8);
	*((uint16_t *) (endAddr)) = poison_val;
	ref<ConstantExpr> resultExpr = ConstantExpr::create( (uint64_t) returnedBuf, Expr::Int64);
	target_ctx_gregs_OS->write(GREG_RAX * 8, resultExpr);

	do_ret();//Fake a return
	return;
	
      } else {
	//For the sake of simplicity, just wipe the old mapping and issue a new one.

	size_t size =  2 + 8 + sizeIn + 8 + 2;
	void * res = malloc(size);
	void * returnedBuf = (void *) ((uint64_t) res + 2 + 8);
	//Need to be able to line up returned ptr vs actual buf later when free is called.
	if (taseDebug) {
	  printf("bufferGuard obtained mapping on heap at 0x%lx \n", (uint64_t) res);
	  printf("bufferGuard returning ptr at 0x%lx \n", (uint64_t) returnedBuf);
	}
	heap_guard_map.insert(std::pair<void *, void *>(returnedBuf, res));

	tase_map_buf((uint64_t) returnedBuf, sizeIn + 6);
	//Poison edges around buffer
	* ((uint16_t *) (res)) = poison_val;
	void * endAddr =(void *)( (uint64_t) res + 2 + 8 + (sizeIn)  + 8);
	*((uint16_t *) (endAddr)) = poison_val;

	//Todo -- copy potentially symbolic data byte-by-byte
	memcpy(returnedBuf, ptrIn, sizeIn);

	//Unbind the old mapping
	void * translatedPtrIn;
	auto lookup = heap_guard_map.find(ptrIn);
	if (lookup != heap_guard_map.end()) {
	  translatedPtrIn = (void *) ((uint64_t) (lookup->second));
	  heap_guard_map.erase(ptrIn);
	} else {
	  //No translation needed
	  translatedPtrIn = ptrIn;
	}

	ObjectPair OP;
	ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) ptrIn, Expr::Int64);
	if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP) ) {
	  const MemoryObject * MO = OP.first;
	  GlobalExecutionStatePtr->addressSpace.unbindObject(MO);
	  free(translatedPtrIn);
	} else {
	  printf("Unable to resolve realloc call to underlying buffer in bufferGuard case \n");
	  fflush(stdout);
	  std::exit(EXIT_FAILURE);
	}

	ref<ConstantExpr> resultExpr = ConstantExpr::create( (uint64_t) returnedBuf, Expr::Int64);
	target_ctx_gregs_OS->write(GREG_RAX * 8, resultExpr);
	do_ret();//Fake a return
	return; 
      
      }
      
      
    } else {
      
      void * ptr = (void *) target_ctx_gregs[GREG_RDI].u64;
      size_t size = (size_t) target_ctx_gregs[GREG_RSI].u64;
      void * res = realloc(ptr,size);
      if (modelDebug) {
	printf("Calling realloc on 0x%lx with size 0x%lx.  Ret val is 0x%lx \n", (uint64_t) ptr, (uint64_t) size, (uint64_t) res);
      }
      if (roundUpHeapAllocations)
	size = roundUp(size, 8);
	
      ref<ConstantExpr> resultExpr = ConstantExpr::create( (uint64_t) res, Expr::Int64);
      target_ctx_gregs_OS->write(GREG_RAX * 8, resultExpr);

      //Treat the realloc(0,size) call like a call to malloc(size)
      if (ptr == NULL) {
	tase_map_buf((uint64_t) res, size);
	
	do_ret();
	return;
      }
	
      if (res != ptr) {
	if (modelDebug) {
	  printf("REALLOC call moved site of allocation \n");
	  std::cout.flush();
	}
	ObjectPair OP;
	ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) ptr, Expr::Int64);
	if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP) ) {
	  const MemoryObject * MO = OP.first;
	  //Todo: carefully copy out/ copy in symbolic data if present
	    
	  GlobalExecutionStatePtr->addressSpace.unbindObject(MO);
	    
	  MemoryObject * newMO = addExternalObject(*GlobalExecutionStatePtr, (void *) res, size, false);
	  const ObjectState * newOSConst = GlobalExecutionStatePtr->addressSpace.findObject(newMO);
	  ObjectState *newOS = GlobalExecutionStatePtr->addressSpace.getWriteable(newMO,newOSConst);
	  newOS->concreteStore = (uint8_t *) res;
	  if (modelDebug) {
	    printf("added MO for realloc at 0x%lx with size 0x%lx after orig location 0x%lx  \n", (uint64_t) res, size, (uint64_t) ptr);
	  }
	    
	} else {
	  printf("ERROR: realloc called on ptr without underlying buffer \n");
	  std::cout.flush();
	  std::exit(EXIT_FAILURE);
	    
	}
	  
      } else {
	ObjectPair OP;
	ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
	if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP)) {
	  const MemoryObject * MO = OP.first;
	  size_t origObjSize = MO->size;
	  printf("REALLOC call kept buffer in same location \n");
	  std::cout.flush();
	    
	  if (size <= origObjSize) {
	    //Don't need to do anything
	    printf("Realloc to smaller or equal size buffer -- no action needed \n");
	  } else {
	    printf("Realloc to larger buffer \n");
	    //extend size of MO
	    //Todo: carefully copy out/ copy in symbolic data if present
	    GlobalExecutionStatePtr->addressSpace.unbindObject(MO);
	      
	    MemoryObject * newMO = addExternalObject(*GlobalExecutionStatePtr, (void *) res, size, false);
	    const ObjectState * newOSConst = GlobalExecutionStatePtr->addressSpace.findObject(newMO);
	    ObjectState *newOS = GlobalExecutionStatePtr->addressSpace.getWriteable(newMO,newOSConst);
	    newOS->concreteStore = (uint8_t *) res;
	    printf("added MO for realloc at 0x%lx with size 0x%lx after orig size 0x%lx  \n", (uint64_t) res, size, origObjSize);
	  }
	} else {
	  printf("Error in realloc -- could not find original buffer info for ptr \n");
	  std::cout.flush();
	  std::exit(EXIT_FAILURE);
	}
      }
	
      do_ret();//Fake a return
    }
      
  } else {
    concretizeGPRArgs(2, "model_realloc");
    model_realloc();
  }
}

//http://man7.org/linux/man-pages/man3/malloc.3.html

//Todo -- be careful of interaction between sizeArg, initSizeArg, and roundUp.
//Clean that code up so it's simpler.
void Executor::model_malloc() {
  static int times_model_malloc_called = 0;
  times_model_malloc_called++;

  if (isBufferEntirelyConcrete((uint64_t) &(target_ctx_gregs[GREG_RDI].u64), 8)) {
    size_t sizeArg = (size_t) target_ctx_gregs[GREG_RDI].u64;
    if (taseDebug)
      printf("Entered model_malloc for time %d with requested size 0x%lx \n",times_model_malloc_called, sizeArg);

    
    if (roundUpHeapAllocations) 
      sizeArg = roundUp(sizeArg, 8);

    size_t initSizeArg;
    if (bufferGuard) {
      initSizeArg = sizeArg;
      sizeArg += 20; // 4 bytes for  the psn, plus two 8-byte buffers between psn and the
      //allocated space.  Our poison checking in the compiler sometimes conservatively promotes
      //read/write checks (e.g., an 8 byte write to a 16 byte check) when it can't determine
      //alignment information, so the extra 8-byte buffers should help with that.
      if (taseDebug){
	printf("Requesting malloc of size 0x%lx for bufferGuard on heap \n",sizeArg);
      }
    }
    void * buf = malloc(sizeArg);

    void * returnedBuf;
    if (bufferGuard) {
      returnedBuf = (void *) ( ((uint64_t) buf) + 2 + 8);
      //Need to be able to line up returned ptr vs actual buf later when free is called.
      if (taseDebug) {
	printf("bufferGuard obtained mapping on heap at 0x%lx \n", (uint64_t) buf);
	printf("bufferGuard returning ptr at 0x%lx \n", (uint64_t) returnedBuf);
      }
      heap_guard_map.insert(std::pair<void *, void *>(returnedBuf, buf));
    }else {
      returnedBuf = buf;
    }

    if (bufferGuard) {
      //printf("Calling tase_map_buf on addr 0x%lx with size 0x%lx \n", (uint64_t) returnedBuf, initSizeArg);
      //Todo -- Should we force-align both the poison buffer and pads to 8 bytes for all, giving
      //a total of 32 bytes of padding?
      tase_map_buf((uint64_t) returnedBuf, initSizeArg +6) ; //6 added to bring us up to alignment
      //Poison edges around buffer
      * ((uint16_t *) (buf)) = poison_val;
      void * endAddr =(void *)( (uint64_t) buf + 2 + 8 + (initSizeArg)  + 8);
      *((uint16_t *) (endAddr)) = poison_val;
      //printf("First Addr with poison in bufferGuard is 0x%lx \n", (uint64_t) endAddr);
    } else {
      tase_map_buf((uint64_t) returnedBuf, sizeArg);
    }
    
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) returnedBuf, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr); 

    do_ret();//Fake a return
    
    if (taseDebug) {
      printf("INTERPRETER: Exiting model_malloc \n"); 
      std::cout.flush();
    }
  } else {
    concretizeGPRArgs(1, "model_malloc");
    model_malloc();
  }
}

//https://linux.die.net/man/3/free
//Todo -- add check to see if rsp is symbolic, or points to symbolic data (somehow)


void Executor::model_free() {
  static int freeCtr = 0;
  freeCtr++;
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if (isa<ConstantExpr>(arg1Expr)) {

    if (!skipFree) {
    
    
      void * freePtr = (void *) target_ctx_gregs[GREG_RDI].u64;
      //printf("Calling model_free on addr 0x%lx \n", (uint64_t) freePtr);
      if (bufferGuard) {
	printf("Attempting to free a heap object with buffer guards enabled\n");
	fflush(stdout);
	
	auto lookup = heap_guard_map.find(freePtr);
	if (lookup != heap_guard_map.end()) {
	  void * translatedAddr = (void *) ((uint64_t) (lookup->second));
	  free (translatedAddr);
	  heap_guard_map.erase(freePtr);
	} else {
	  
	  free(freePtr);
	}
      } else {
	free(freePtr);
      }
	
      ObjectPair OP;
      ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) freePtr, Expr::Int64);
      if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP)) {
	//printf("Unbinding object in free \n");
	//std::cout.flush();
	GlobalExecutionStatePtr->addressSpace.unbindObject(OP.first);
      
      } else {
	printf("ERROR: Found free called without buffer corresponding to ptr \n");
	std::cout.flush();
	std::exit(EXIT_FAILURE);
      }

    }
    
    do_ret();//Fake a return
    
  } else {
    concretizeGPRArgs(1, "model_free");
    model_free();
  } 
}

//
//https://linux.die.net/man/3/freopen
//FILE *freopen(const char *path, const char *mode, FILE *stream);
void Executor::model_freopen() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);

  if (  (isa<ConstantExpr>(arg1Expr)) &&
	(isa<ConstantExpr>(arg2Expr)) &&
	(isa<ConstantExpr>(arg3Expr)) ) {
    FILE * res = freopen((char *) target_ctx_gregs[GREG_RDI].u64, (char *) target_ctx_gregs[GREG_RSI].u64, (FILE *) target_ctx_gregs[GREG_RDX].u64);
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();//fake a ret

  } else {
    concretizeGPRArgs(3, "model_freopen");
    model_freopen();
  }


}

//Todo -- check byte-by-byte through the input args for symbolic data
//http://man7.org/linux/man-pages/man3/fopen.3.html
//FILE *fopen(const char *pathname, const char *mode);
void Executor::model_fopen() {

  printf("Entering model_fopen \n");
  
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr))
       ){

    FILE * res = fopen( (char *) target_ctx_gregs[GREG_RDI].u64, (char *) target_ctx_gregs[GREG_RSI].u64);
    printf("Calling fopen on file %s \n", (char *) target_ctx_gregs[GREG_RDI].u64);
    //Return result
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();//fake a ret
    
  } else {
    concretizeGPRArgs(2, "model_fopen");
    model_fopen();
  }

}

//Todo -- check byte-by-byte through the input args for symbolic data
//http://man7.org/linux/man-pages/man3/fopen.3.html
//FILE *fopen64(const char *pathname, const char *mode);
void Executor::model_fopen64() {

  printf("Entering model_fopen64 \n");

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr))
       ){

    FILE * res = fopen64( (char *) target_ctx_gregs[GREG_RDI].u64, (char *) target_ctx_gregs[GREG_RSI].u64);
    printf("Calling fopen64 on file %s \n", (char *) target_ctx_gregs[GREG_RDI].u64);
    //Return result
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();//fake a ret
    
  } else {
    concretizeGPRArgs(2, "model_fopen64");
    model_fopen64();
  }

}

//https://linux.die.net/man/3/getc_unlocked
//int getc_unlocked(FILE *stream);
void Executor::model_getc_unlocked() {
  static int numCalls = 0;
  numCalls++;

  if (taseDebug) {
    printf("Calling model_getc_unlocked for time %d \n", numCalls);
  }

  //TODO: Generalize this fast path for all the other models to avoid unnecessary obj
  //creation.
  if (isBufferEntirelyConcrete((uint64_t) &target_ctx_gregs[GREG_RDI].u64, 8)
      && isBufferEntirelyConcrete((uint64_t) &target_ctx_gregs[GREG_RAX].u64, 8)) {
  
    int res = getc_unlocked((FILE *) target_ctx_gregs[GREG_RDI].u64);
    target_ctx_gregs[GREG_RAX].u64 = (uint64_t) res;
    do_ret();
  } else {

    
    if (taseDebug) {
      printf("Calling model_getc_unlocked for time %d \n", numCalls);
    }
    
    ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
    if (isa<ConstantExpr>(arg1Expr)) {
      int res = getc_unlocked( (FILE *) target_ctx_gregs[GREG_RDI].u64);
      
      ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
      target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
      do_ret();//fake a ret
      
    } else {
      concretizeGPRArgs(1, "model_getc_unlocked");
      model_getc_unlocked();
    }
    
  }

}


//https://www.man7.org/linux/man-pages/man3/feof.3.html
//int feof(FILE *stream);


void Executor::model_feof() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr))
       ){

    int res = feof((FILE *) target_ctx_gregs[GREG_RDI].u64);

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();//Fake a return
  } else {
    concretizeGPRArgs(1, "model_feof");
    model_feof();
  }


}

//https://man7.org/linux/man-pages/man3/ferror.3.html
//int ferror(FILE *stream);

void Executor::model_ferror() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr))
       ){

    int res = ferror((FILE *) target_ctx_gregs[GREG_RDI].u64);

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();//Fake a return
  } else {
    concretizeGPRArgs(1, "model_ferror");
    model_ferror();
  }


}

//https://man7.org/linux/man-pages/man2/posix_fadvise.2.html
//int posix_fadvise(int fd, off_t offset, off_t len, int advice);

void Executor::model_posix_fadvise() {

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  ref<Expr> arg4Expr = target_ctx_gregs_OS->read(GREG_RCX * 8, Expr::Int64);

  if (  (isa<ConstantExpr>(arg1Expr)) &&
	(isa<ConstantExpr>(arg2Expr)) &&
	(isa<ConstantExpr>(arg3Expr)) &&
	(isa<ConstantExpr>(arg4Expr))
	) {

    int res = posix_fadvise( (int) target_ctx_gregs[GREG_RDI].u64, (off_t) target_ctx_gregs[GREG_RSI].u64, (off_t) target_ctx_gregs[GREG_RDX].u64, (int) target_ctx_gregs[GREG_RCX].u64);

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();
    
  }  else {
    concretizeGPRArgs(4, "model_posix_fadvise");
    model_posix_fadvise();
  }

    
}

//http://man7.org/linux/man-pages/man3/fclose.3.html
//int fclose(FILE *stream);
//Todo -- examine all bytes of stream for symbolic taint
void Executor::model_fclose() {
  printf("Entering model_fclose at %lu \n", interpCtr);
  fflush(stdout);
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr))
       ){
    
    //We don't need to make any call
    
    do_ret();//Fake a return
    
  } else {
    concretizeGPRArgs(1, "model_fclose");
    model_fclose();
  }
  
}

// http://man7.org/linux/man-pages/man3/fseek.3.html
//int fseek(FILE *stream, long offset, int whence);

// Just pass the call through
void Executor::model_fseek() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  
  if (  (isa<ConstantExpr>(arg1Expr)) &&
	(isa<ConstantExpr>(arg2Expr)) &&
	(isa<ConstantExpr>(arg3Expr)) ) {

    FILE* stream = (FILE *) target_ctx_gregs[GREG_RDI].u64;
    long offset = (long) target_ctx_gregs[GREG_RSI].u64;
    int whence = (int) target_ctx_gregs[GREG_RDX].u64;
    int res = fseek(stream, offset, whence);
    
    //Return result
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();
    
  } else {
    concretizeGPRArgs(3, "model_fseek");
    model_fseek();
  }
}

//http://man7.org/linux/man-pages/man3/ftell.3p.html
// long ftell(FILE *stream);

// Just pass the call through
void Executor::model_ftell() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if (  (isa<ConstantExpr>(arg1Expr)) ) {

    FILE * stream = (FILE *) target_ctx_gregs[GREG_RDI].u64;
    long res = ftell(stream);

    //Return result
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();
    
  } else {
    concretizeGPRArgs(1, "model_ftell");
    model_ftell();
  }
}


//http://man7.org/linux/man-pages/man3/rewind.3p.html
//void rewind(FILE *stream);

//Just pass the call through
void Executor::model_rewind() {
  
   ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if (  (isa<ConstantExpr>(arg1Expr)) ) {

    FILE * stream = (FILE *) target_ctx_gregs[GREG_RDI].u64;
    rewind(stream);

    do_ret();
    
  } else {
    concretizeGPRArgs(1, "model_rewind");
    model_rewind();
  }

}

//http://man7.org/linux/man-pages/man3/fread.3.html
//size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
//Todo -- Inspect byte-by-byte for symbolic taint
void Executor::model_fread() {
  if (taseDebug) {
    printf("Entering model_fread \n");
  }
  
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  ref<Expr> arg4Expr = target_ctx_gregs_OS->read(GREG_RCX * 8, Expr::Int64);

  if (  (isa<ConstantExpr>(arg1Expr)) &&
	(isa<ConstantExpr>(arg2Expr)) &&
	(isa<ConstantExpr>(arg3Expr)) &&
	(isa<ConstantExpr>(arg4Expr)) 
	) {
  
    size_t res = fread( (void *) target_ctx_gregs[GREG_RDI].u64, (size_t) target_ctx_gregs[GREG_RSI].u64, (size_t) target_ctx_gregs[GREG_RDX].u64, (FILE *) target_ctx_gregs[GREG_RCX].u64);
    
    //Return result
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();//Fake a return

  } else {
    concretizeGPRArgs(4, "model_fread");
    model_fread();
  }
  
}

extern int __isoc99_sscanf ( const char * s, const char * format, ...);
void Executor::model___isoc99_sscanf() {
  
  printf("WARNING: Return 0 on unmodeled sscanf call \n");;
  
  int res = 0;
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  do_ret();//Fake a return
  
}


//http://man7.org/linux/man-pages/man3/gethostbyname.3.html
//struct hostent *gethostbyname(const char *name);
//Todo -- check bytes of input for symbolic taint
void Executor::model_gethostbyname() {
  printf("Entering model_gethostbyname at interpCtr %lu \n", interpCtr);
  fflush(stdout);
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr))
       ){
    //Do the call
    printf("Calling model_gethostbyname on %s \n", (char *) target_ctx_gregs[GREG_RDI].u64);
    fflush(stdout);
    struct hostent * res = (struct hostent *) gethostbyname ((const char *) target_ctx_gregs[GREG_RDI].u64);

    //If it doesn't exit, back hostent struct with a memory object.
    ObjectPair OP;
    ref<ConstantExpr> addrExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    if (GlobalExecutionStatePtr->addressSpace.resolveOne(addrExpr, OP)) {
      printf("hostent result appears to have MO already backing it \n");
      fflush(stdout);
      
    } else {
      printf("Creating MO to back hostent at 0x%lx with size 0x%lx \n", (uint64_t) res, sizeof(hostent));
      fflush(stdout);
      MemoryObject * newMO = addExternalObject(*GlobalExecutionStatePtr, (void *) res, sizeof(hostent), false);
      const ObjectState * newOSConst = GlobalExecutionStatePtr->addressSpace.findObject(newMO);
      ObjectState *newOS = GlobalExecutionStatePtr->addressSpace.getWriteable(newMO,newOSConst);
      newOS->concreteStore = (uint8_t *) res;

      //Also map in h_addr_list elements for now until we get a better way of mapping in env vars and their associated data
      //Todo -get rid of this hack.  For robust environment modeling, we need to find all the pointers-to-pointers and
      //back them with buffers.  We don't currently need that level of modeling just for behavioral verification.
      
      uint64_t  baseAddr = (uint64_t) &(res->h_addr_list[0]);
      size_t size = 0;
      for (char ** itrPtr = res->h_addr_list; *itrPtr != 0; itrPtr++) {
	printf("Iterating on h_addr_list \n");
	size++;
	fflush(stdout);
      }
      printf("h_addr_list has %lu entries \n", size);
      fflush(stdout);
      
      printf("Mapping in buf at 0x%lx with size 0x%lx for h_addr_list", baseAddr, size);
      MemoryObject * listMO = addExternalObject(*GlobalExecutionStatePtr, (void *) baseAddr, size, false);
      const ObjectState * listOSConst = GlobalExecutionStatePtr->addressSpace.findObject(listMO);
      ObjectState * listOS = GlobalExecutionStatePtr->addressSpace.getWriteable(listMO, listOSConst);
      listOS->concreteStore = (uint8_t *) baseAddr;
      
    }

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();//Fake a return
    
  } else {
    concretizeGPRArgs(1, "model_gethostbyname");
    model_gethostbyname();
  }

}


//int setsockopt(int sockfd, int level, int optname,
//             const void *optval, socklen_t optlen);
//https://linux.die.net/man/2/setsockopt
//Todo -- actually model this
void Executor::model_setsockopt() {
  printf("Entering model_setsockopt at interpCtr %lu \n", interpCtr);
  fflush(stdout);
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  ref<Expr> arg4Expr = target_ctx_gregs_OS->read(GREG_RCX * 8, Expr::Int64);
  ref<Expr> arg5Expr = target_ctx_gregs_OS->read(GREG_R8 * 8, Expr::Int64);

  if (  (isa<ConstantExpr>(arg1Expr)) &&
	(isa<ConstantExpr>(arg2Expr)) &&
	(isa<ConstantExpr>(arg3Expr)) &&
	(isa<ConstantExpr>(arg4Expr)) &&
	(isa<ConstantExpr>(arg5Expr))
	) {

    int res = 0; //Pass success
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();//Fake a return

  } else {
    concretizeGPRArgs(5, "model_setsockopt");
    model_setsockopt();
  }
}

//No args for this one
void Executor::model___ctype_b_loc() {
  if (!noLog) {
    printf("Entering model__ctype_b_loc at interpCtr %lu \n", interpCtr);
  }

  const unsigned short ** constRes = __ctype_b_loc();
  unsigned short ** res = const_cast<unsigned short **>(constRes);
  
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  
  do_ret();//Fake a return
  
}


//int32_t * * __ctype_tolower_loc(void);
//No args
//Todo -- allocate symbolic underlying results later for testing
void Executor::model___ctype_tolower_loc() {
  if (!noLog) {
    printf("Entering model__ctype_tolower_loc at interpCtr %lu \n", interpCtr);
  }
  
  const int  ** constRes = __ctype_tolower_loc();
  int ** res = const_cast<int **>(constRes);
  
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  
  do_ret();//Fake a return

}

//int fflush(FILE *stream);
//Todo -- Actually model this or provide a symbolic return status
void Executor::model_fflush(){
  if (!noLog) {
    printf("Entering model_fflush at %lu \n", interpCtr);
  }
  

  //Get the input args per system V linux ABI.

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr))
       ){

    int res = 0;
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();//fake a ret

  } else {
    concretizeGPRArgs(1, "model_fflush");
    model_fflush();
  }

}


//char *fgets(char *s, int size, FILE *stream);
//https://linux.die.net/man/3/fgets
void Executor::model_fgets() {
  if (!noLog) {
    printf("Entering model_fgets at %lu \n", interpCtr);
  }
  //Get the input args per system V linux ABI.  
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  
  if (
      (isa<ConstantExpr>(arg1Expr)) &&
      (isa<ConstantExpr>(arg2Expr)) &&
      (isa<ConstantExpr>(arg3Expr)) 
      ){
    //Do call
    char * res = fgets((char *) target_ctx_gregs[GREG_RDI].u64, (int) target_ctx_gregs[GREG_RSI].u64, (FILE *) target_ctx_gregs[GREG_RDX].u64);
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();//Fake a return

  } else {
    concretizeGPRArgs(3, "model_fgets");
    model_fgets();
  }
}

//Todo -- Inspect byte-by-byte for symbolic taint
// https://linux.die.net/man/3/fwrite
// size_t fwrite(const void *ptr, size_t size, size_t nmemb,
// FILE *stream);
void Executor::model_fwrite() {
  if (!noLog) {
    printf("Entering model_fwrite \n");
  }
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  ref<Expr> arg4Expr = target_ctx_gregs_OS->read(GREG_RCX * 8, Expr::Int64);
  
  if (  (isa<ConstantExpr>(arg1Expr)) &&
	(isa<ConstantExpr>(arg2Expr)) &&
	(isa<ConstantExpr>(arg3Expr)) &&
	(isa<ConstantExpr>(arg4Expr))
	) {

    //size_t res = ( (size_t) target_ctx_gregs[GREG_RDX].u64 );

    
    
    size_t res = fwrite( (void *) target_ctx_gregs[GREG_RDI].u64, (size_t) target_ctx_gregs[GREG_RSI].u64, (size_t) target_ctx_gregs[GREG_RDX].u64, (FILE *) target_ctx_gregs[GREG_RCX].u64);
    
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();//Fake a return

  } else {
    concretizeGPRArgs(4, "model_fwrite");
    model_fwrite();
  } 
}
/*
Executor::print_specifier Executor::parse_specifier (char * input, int * offset) {

  if (input[0] != '%') {
    printf("printf model ERROR: invalid print specifier \n");
    return bad_s;
  }

  if (input[1] == 'd') {
    *offset=2;
    return d_s;
  } else if (input[1] == 'f') {
    *offset=2;
    return f_s;
  } else if (input[1] == 'c') {
    *offset=2;
    return c_s;
  } else if (input[1] == 's') {
    *offset=2;
    return s_s;
  } else if (input[1] == 'l') {
    if (input[2] == 'f') {
      *offset = 3;
      return lf_s;
    } else if (input[2] == 'u') {
      *offset = 3;
      return lu_s;
    } else {
      printf("printf model ERROR: unrecognized print specifier at start of string: \n %s \n", input);
      return bad_s;
    }
  } else {
    printf("printf model ERROR: unrecognized print specifier at start of string: \n %s \n", input);
    return bad_s;
  }

}


//ABI -- RDI, RSI, RDX, RCX, R8, R9 for first integer/ptr args
//For our printf modeling, we're assuming the format string was
//passed into RDI as the 0th arg.  In our limited support for floats
//and doubles, XMM/YMM/ZMM regs are disable so the float/double args
//should go into those general purpose registers.

bool Executor::addPrintfArgNo(std::stringstream * output ,print_specifier type, unsigned int argNo) {
  union BITS {
    uint64_t as_u64;
    int      as_int;
    float    as_float;
    double  as_double;
    char    as_char;
    char * as_str;
  } b;

  switch (argNo){

  case 1 :
    b.as_u64 = target_ctx_gregs[GREG_RSI].u64;
    break;
  case 2:
    b.as_u64 = target_ctx_gregs[GREG_RDX].u64;
    break;
  case 3:
    b.as_u64 = target_ctx_gregs[GREG_RCX].u64;
    break;
  case 4:
    b.as_u64 = target_ctx_gregs[GREG_R8].u64;
    break;
  case 5:
    b.as_u64 = target_ctx_gregs[GREG_R9].u64;
    break;
  default: {
    printf("ERROR: Unable to parse printf arg number %u \n", argNo);
    return false;
  }
  }


  switch (type) {
  case d_s:
    *output << b.as_int;
    break;
  case f_s:
    printf("float arg appears to be %f \n", b.as_float);
    printf("With double spec, float arg appears to be %lf \n", b.as_float);
    printf("As double, arg is %lf \n", b.as_double);
    *output <<b.as_double;
    break;
  case lf_s:
    printf("double arg appears to be %lf \n", b.as_double);
    
    *output << b.as_double;
    break;
  case s_s:
    *output << b.as_str;
    break;
  case c_s:
    *output << b.as_char;
    break;
  case lu_s:
    *output << b.as_u64;
    break;
  default: {
    printf("Error: unrecognized print specifier in model_printf \n");
    return false;
  }
  }

  return true;

}
*/
/*
void Executor::model_printf() {

  char * input = (char *) target_ctx_gregs[GREG_RDI].u64;
  
  unsigned int input_idx = 0;
  size_t len = strlen(input);
  std::stringstream output;

  int currArgNum = 1;

  //Scan through input format string until we hit a %
  while ( input_idx < len) {
    

    if (input[input_idx] == '%') {
      int offset;
      print_specifier ps = parse_specifier((char *) &input[input_idx], &offset);
      if (ps == bad_s) {
	printf("ERROR: Failed to parse format string for printf.  Skipping call. \n");
	do_ret();
	return;
      }

      bool success = addPrintfArgNo(&output, ps, currArgNum);
      if (!success) {
	printf("Encountered error in addPrintfArgNo.  Skipping printf call \n");
	do_ret();
	return;
      }
      currArgNum++;

      input_idx += offset;

    } else {
      output << input[input_idx];

      input_idx++;
    }


  }
  printf("Final result of model_printf----------: \n %s \n", output.str().c_str());
  do_ret();

}
*/
//Modeles for strtod, strtol, etc. and wcstod, wcstol, etc.  These should just
//be user-level code in principal for which we don't need models, but because the musl
//stdlib implmentation reuses some file-based utilities from stdio to handle the string
//manipulation, we're just trapping on these for now to avoid linking in all of stdio.


//Models for strtoX

void Executor::model_strtof() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr))
       ){
    union BITS {
      float asFloat;
      double asDouble;
      uint64_t asUint64_t;
    } b;

    const char * nptr = (char *) target_ctx_gregs[GREG_RDI].u64;
    char ** endptr = (char **) target_ctx_gregs[GREG_RSI].u64;
    b.asFloat = strtof(nptr,endptr);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();
    
  } else {
    concretizeGPRArgs(2, "model_strtof");
    model_strtof();
  }
}
void Executor::model_strtod() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr))
       ){
    union BITS {
      float asFloat;
      double asDouble;
      uint64_t asUint64_t;
    } b;

    const char * nptr = (char *) target_ctx_gregs[GREG_RDI].u64;
    char ** endptr = (char **) target_ctx_gregs[GREG_RSI].u64;
    b.asDouble = strtod(nptr,endptr);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(2, "model_strtod");
    model_strtod();
  }
}
void Executor::model_strtold() {
  printf("TASE INFO: Calling strtold and returning 64 bits for long double \n");
  model_strtod();
}


void Executor::model_strtoull() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      unsigned long long asULL;
      long long asLL;
      unsigned long asUL;
      long asLong;
      uint64_t asUint64_t;
    } b;
    const char * str = (char *) target_ctx_gregs[GREG_RDI].u64;
    char ** endptr = (char **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asULL = strtoull(str, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();
    
  } else {
    concretizeGPRArgs(3, "model_strtoull");
    model_strtoull();
  }
    
}
void Executor::model_strtoll() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      unsigned long long asULL;
      long long asLL;
      unsigned long asUL;
      long asLong;
      uint64_t asUint64_t;
    } b;
    const char * str = (char *) target_ctx_gregs[GREG_RDI].u64;
    char ** endptr = (char **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asLL = strtoll(str, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_strtoll");
    model_strtoll();
  }
}
void Executor::model_strtoul() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      unsigned long long asULL;
      long long asLL;
      unsigned long asUL;
      long asLong;
      uint64_t asUint64_t;
    } b;
    const char * str = (char *) target_ctx_gregs[GREG_RDI].u64;
    char ** endptr = (char **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asUL = strtoul(str, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_strtoul");
    model_strtoul();
  }
}
void Executor::model_strtol() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      unsigned long long asULL;
      long long asLL;
      unsigned long asUL;
      long asLong;
      uint64_t asUint64_t;
    } b;
    const char * str = (char *) target_ctx_gregs[GREG_RDI].u64;
    char ** endptr = (char **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asLong = strtol(str, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_strtol");
    model_strtol();
  }
}
void Executor::model_strtoimax() {
  printf("TASE INFO: Modeling strtoimax as strtoll \n");
  model_strtoll();
}
void Executor::model_strtoumax() {
  printf("TASE INFO: Modeling strtoumax as strtoull \n");
  model_strtoull();
}

//Models for wcstoX

void Executor::model_wcstof() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) 
       ){
    union BITS {
      float asFloat;
      uint64_t asUint64_t;
    } b;
    
    const wchar_t *  nptr = (wchar_t *) target_ctx_gregs[GREG_RDI].u64; 
    wchar_t **  endptr = (wchar_t **) target_ctx_gregs[GREG_RSI].u64;
    b.asFloat = wcstof(nptr, endptr);
    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();
    
  } else {
    concretizeGPRArgs(2, "model_wcstof");
    model_wcstof();
  }
}
void Executor::model_wcstod() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr))
       ){
    union BITS {
      double asDouble;
      uint64_t asUint64_t;
    } b;

    const wchar_t *  nptr = (wchar_t *) target_ctx_gregs[GREG_RDI].u64;
    wchar_t **  endptr = (wchar_t **) target_ctx_gregs[GREG_RSI].u64;
    b.asDouble = wcstod(nptr, endptr);
    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(2, "model_wcstod");
    model_wcstod();
  }
}
void Executor::model_wcstold() {
  printf("TASE INFO: Calling model_wctold and returning 64 bits for long double \n");
  model_wcstod();   
}

//wcstol models

void Executor::model_wcstoull() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      unsigned long long asULL;
      uint64_t asUint64_t;
    } b;

    const wchar_t *  nptr = (wchar_t *) target_ctx_gregs[GREG_RDI].u64;
    wchar_t ** endptr = (wchar_t **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asULL = wcstoull(nptr, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_wcstoull");
    model_wcstoull();
  }
}
void Executor::model_wcstoll() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      long long asLL;
      uint64_t asUint64_t;
    } b;

    const wchar_t *  nptr = (wchar_t *) target_ctx_gregs[GREG_RDI].u64;
    wchar_t ** endptr = (wchar_t **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asLL = wcstoll(nptr, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_wcstoll");
    model_wcstoll();
  }
}
void Executor::model_wcstoul() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      unsigned long asUL;
      uint64_t asUint64_t;
    } b;

    const wchar_t * nptr = (wchar_t *) target_ctx_gregs[GREG_RDI].u64;
    wchar_t ** endptr = (wchar_t **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asUL = wcstoul(nptr, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_wcstoul");
    model_wcstoul();
  }

}
void Executor::model_wcstol() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){

    union BITS {
      long asLong;
      uint64_t asUint64_t;
    } b;

    const wchar_t *  nptr = (wchar_t *) target_ctx_gregs[GREG_RDI].u64;
    wchar_t ** endptr = (wchar_t **) target_ctx_gregs[GREG_RSI].u64;
    int base = (int) target_ctx_gregs[GREG_RDX].u64;

    b.asLong = wcstol(nptr, endptr, base);

    ref<ConstantExpr> resExpr = ConstantExpr::create(b.asUint64_t, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(3, "model_wcstol");
    model_wcstol();
  }
}
void Executor::model_wcstoimax() {
  printf("TASE INFO: Modeling wcstoimax as wcstoll \n");
  model_wcstoll();
}
void Executor::model_wcstoumax() {
  printf("TASE INFO: Modeling wcstoumax as wcstoull \n");
  model_wcstoull();
}

// size_t mbsrtowcs (wchar_t* dest, const char** src, size_t max, mbstate_t* ps);
void Executor::model_mbsrtowcs(){
  if(!noLog){
    _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64; // RSP should be sitting on return addr
  ++s_offset;


  wchar_t* dest;
  const char** src;
  size_t max;
  mbstate_t* ps;
  get_vals(count, s_offset, __func__, dest, src, max, ps);

  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) mbsrtowcs(dest, src, max, ps), Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}
// just mapping in in initialization instead...
// Executor::model_getprogname(){
//   if(!noLog){
//     printf("Entering model_getprogrname at interpCtr %lu \n", interpCtr);
//   }
//   tase_map_buf((uint64_t) &program_invocation_short_name, strlen(program_invocation_short_name));
//   ref<ConstantExpr> res = ConstantExpr::create((uint64_t) &program_invocation_short_name[0], Expr::Int64);
//   tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
//   do_ret();
// }

void Executor::model_puts() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr))       
       ){

    printf("Passing puts call through: \n");
    puts((char *) target_ctx_gregs[GREG_RDI].u64);

    //Always model the call as succeeding.  In the future, for bugfinding it
    //would be interesting to make the return value symbolic to model failure.

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) 0, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    do_ret();
    
  } else {
    concretizeGPRArgs(1, "model_puts");
    model_puts();
  }
}

void Executor::model_setlocale(){
  if(!noLog){
    _LOG
  }
  int count = 0;
  uint64_t * s_offset = (uint64_t*) target_ctx_gregs[GREG_RSP].u64;
  ++s_offset;

  int category;
  char * locale;
  get_vals(count, s_offset, __func__, category, locale);

  char * out = setlocale(category, locale);
  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) out, Expr::Int64);
  tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RAX], resExpr);
  do_ret();
}

//We're not fully modeling sprintf just yet because of varargs.  However, some sub-libraries
// we do support in musl (e.g., stdlib, ctype, string) have dependencies on sprintf.
//So as a workaround, we cherry-pick those specific calls for now and otherwise
//return an error noting that the function isn't modeled yet.

//The specific calls we're cherry-picking now are ecvt, fcvt, and gcvt in stdlib.
//Todo: Be careful when using this for SSL verification.
/*
void Executor::model_sprintf() {

  char * str = (char *) target_ctx_gregs[GREG_RDI].u64;
  char * fmt = (char *) target_ctx_gregs[GREG_RSI].u64;

  union BITS {
    uint64_t asuint64_t;
    double asdouble;
  } arg4;
  arg4.asuint64_t = target_ctx_gregs[GREG_RCX].u64;
  
  if        (strcmp(fmt, "%.*e") == 0 ) { 
    sprintf(str, fmt, target_ctx_gregs[GREG_RDX].i32, arg4.asdouble);
  } else if (strcmp(fmt, "%.*f") == 0 ) {
    sprintf(str, fmt, target_ctx_gregs[GREG_RDX].i32, arg4.asdouble);
  } else if (strcmp(fmt, "%.*g") == 0 ) {
    sprintf(str, fmt, target_ctx_gregs[GREG_RDX].i32, arg4.asdouble);
  } else {
    printf("ERROR: Unmodeled sprintf call in TASE \n");
    worker_exit();
  }

  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) strlen(str), Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
  
  do_ret();
  
}
*/
//This struct comes from pthread_impl.h in musl.
struct pthread {
  /* Part 1 -- these fields may be external or
   * internal (accessed via asm) ABI. Do not change. */
  struct pthread *self;
  uintptr_t *dtv;
  struct pthread *prev, *next; /* non-ABI */
  uintptr_t sysinfo;
  uintptr_t canary, canary2;

  /* Part 2 -- implementation details, non-ABI. */
  int tid;
  int errno_val;
  volatile int detach_state;
  volatile int cancel;
  volatile unsigned char canceldisable, cancelasync;
  unsigned char tsd_used:1;
  unsigned char dlerror_flag:1;
  unsigned char *map_base;
  size_t map_size;
  void *stack;
  size_t stack_size;
  size_t guard_size;
  void *result;
  void *cancelbuf;
  void **tsd;
  struct {
    volatile void *volatile head;
    long off;
    volatile void *volatile pending;
  } robust_list;
  volatile int timer_id;
  void * locale;
  volatile int killlock[1];
  char *dlerror_buf;
  void *stdio_locks;

  /* Part 3 -- the positions of these fields relative to
   * the end of the structure is external and internal ABI. */
  uintptr_t canary_at_end;
  uintptr_t *dtv_copy;
};

//Handle calls to __pthread_self from our subset of musl libc.  We just
//set up a dummy pthread struct with the entire thing (including the
//errno field) set to 0.

//This is something we'd flesh out if we want to
//do more environment modeling; should be OK for now as-is because
//our subset of libc only uses this pthread struct for accessing errno.
struct pthread * dummy_pthread_struct;
void Executor::model___pthread_self() {
  static int pthread_self_calls = 0;
  if (pthread_self_calls == 0) {
    dummy_pthread_struct = (struct pthread *) calloc(sizeof( struct pthread), 1);
    tase_map_buf( (uint64_t) dummy_pthread_struct, sizeof( struct pthread));
    printf("TASE INFO: Initializing dummy pthread struct for target \n");
  } else {
    printf("TASE INFO: Returning dummy pthread struct in call to __pthread_self \n");    
  }

  ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) dummy_pthread_struct, Expr::Int64);
  target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

  do_ret();
  
}

//Implementation for a_ctz_64 and a_clz_64 comes directly from internal/atomic.h in musl.
//We're just trapping on these for now because the emitted code for them
//use bsr and bsl instructions, which aren't implemented in TASE yet.

void Executor::model_a_ctz_64() {
  static const char debruijn64[64] = {
				      0, 1, 2, 53, 3, 7, 54, 27, 4, 38, 41, 8, 34, 55, 48, 28,
				      62, 5, 39, 46, 44, 42, 22, 9, 24, 35, 59, 56, 49, 18, 29, 11,
				      63, 52, 6, 26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
				      51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12
  };

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr))
       ){
    
    uint64_t x = target_ctx_gregs[GREG_RDI].u64;
    
    int res =  debruijn64[(x&-x)*0x022fdd63cc95386dull >> 58];
    
    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);
    
    do_ret();
    
  } else {
    concretizeGPRArgs(1, "model_a_ctz_64");
    model_a_ctz_64();
  }
}


void Executor::model_a_clz_64() {
  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  if  (
       (isa<ConstantExpr>(arg1Expr)) 
       ){

    uint64_t x = target_ctx_gregs[GREG_RDI].u64;
    
    uint32_t y;
    int r;
    if (x>>32) y=x>>32, r=0; else y=x, r=32;
    if (y>>16) y>>=16; else r |= 16;
    if (y>>8) y>>=8; else r |= 8;
    if (y>>4) y>>=4; else r |= 4;
    if (y>>2) y>>=2; else r |= 2;
    int res =  r | !(y>>1);

    ref<ConstantExpr> resExpr = ConstantExpr::create((uint64_t) res, Expr::Int64);
    target_ctx_gregs_OS->write(GREG_RAX * 8, resExpr);

    do_ret();

  } else {
    concretizeGPRArgs(1, "model_a_clz_64");
    model_a_clz_64();
  }
}
