


//===-- Executor.cpp ------------------------------------------------------===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Executor.h"
#include "Context.h"
#include "CoreStats.h"
#include "ExternalDispatcher.h"
#include "ImpliedValue.h"
#include "Memory.h"
#include "MemoryManager.h"
#include "PTree.h"
#include "Searcher.h"
#include "SeedInfo.h"
#include "SpecialFunctionHandler.h"
#include "StatsTracker.h"
#include "TimingSolver.h"
#include "UserSearcher.h"
#include "ExecutorTimerInfo.h"


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

#if LLVM_VERSION_CODE < LLVM_VERSION(3, 5)
#include "llvm/Support/CallSite.h"
#else
#include "llvm/IR/CallSite.h"
#endif

#ifdef HAVE_ZLIB_H
#include "klee/Internal/Support/CompressionStream.h"
#endif

#include <sys/resource.h>

#include <cassert>
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <sys/mman.h>
#include <errno.h>
#include <cxxabi.h>

#include <immintrin.h>


using namespace llvm;
using namespace klee;

//AH: Our additions below. --------------------------------------


#include <iostream>
#include "klee/CVAssignment.h"
#include "klee/util/ExprUtil.h"
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <netdb.h>
#include <fcntl.h>
#include "API.h"
#include "tase_shims.h"
//#include "tase/TASEControl.h"
#include "../Tase/TASESoftFloatEmulation.h"
#include <sys/times.h>
#include <sys/time.h>
#include <unordered_set>


//Can't include signal.h directly if it has conflicts with our tase_interp.h definitions
//on enums like GREG_RSI, GREG_RDI, etc.
extern "C"  void ( *signal(int signum, void (*handler)(int)) ) (int){
  return NULL;
  }

//Symbols we need to map in for TASE
extern bool taseDebug;
extern bool modelDebug;
extern bool noLog;
extern char edata;
extern char __GNU_EH_FRAME_HDR,  _IO_stdin_used; //used for mapping .rodata section
extern int __ctype_tolower;
extern char ** environ;
extern int * __errno_location();
extern "C" int __isoc99_sscanf ( const char * s, const char * format, ...);


//TASE internals--------------------
extern uint16_t poison_val;
extern target_ctx_t target_ctx;
extern tase_greg_t * target_ctx_gregs;

extern int retryMax;
extern Module * interpModule;
extern klee::Interpreter * GlobalInterpreter;
extern std::unordered_set<uint64_t> cartridge_entry_points;
extern std::unordered_set<uint64_t> cartridges_with_flags_live;
extern double target_start_time;
extern double target_end_time;

extern std::unordered_set<uint32_t> kill_flags;

UFManager *ufmanager;

Executor * GlobalExecutorPtr;
MemoryObject * target_ctx_gregs_MO;
ObjectState * target_ctx_gregs_OS;
//MemoryObject * target_ctx_xmms_MO;
//ObjectState * target_ctx_xmms_OS;
ExecutionState * GlobalExecutionStatePtr;
void * rodata_base_ptr;
uint64_t rodata_size;
uint64_t bounceback_offset = 0;
uint64_t trap_off = 12;  //Offset from function address at which we trap
std::map<uint64_t, KFunction *> IR_KF_Map;
std::vector<ref<Expr> > arguments;

extern "C" void make_byte_symbolic(void * addr);

//TASE stats and logging

uint64_t interpCtr =0;
uint64_t instCtr = 0;
int forkSolverCalls = 0;
std::string prev_unique_log_ID = "NONE";
std::string curr_unique_log_ID = "ROOT";
extern std::stringstream worker_ID_stream;
extern std::string prev_worker_ID;

FILE * prev_stdout_log = NULL;
FILE * prev_stderr_log = NULL;

extern "C" void cycleTASELogs(bool isReplay);
//bool isSpecialInst(uint64_t rip);
extern "C" bool tase_buf_has_taint (const void * ptr, const int size);
//void printCtx(tase_greg_t *);

//Multipass
extern int c_special_cmds; //Int used by cliver to disable special commands to s_client.  Made global for debugging
extern bool UseForkedCoreSolver;
		 //extern bool singleStepping;
extern int round_count;
extern int pass_count;
extern int run_count;
int multipass_symbolic_vars = 0;
extern CVAssignment prevMPA;
std::vector<const klee::Array *> round_symbolics;

//Addition from cliver
std::map<std::string, uint64_t> array_name_index_map_;
std::string get_unique_array_name(const std::string &s) {
  // Look up unique name for this variable, incremented per variable name
  return s + "_" + llvm::utostr(array_name_index_map_[s]++);
}


//Todo : fix these functions and remove traps
#ifdef TASE_OPENSSL

extern "C" {
  void RAND_add(const void * buf, int num, double entropy);
  int RAND_load_file(const char *filename, long max_bytes);
}


void OpenSSLDie (const char * file, int line, const char * assertion);

extern "C" {
  int RAND_poll();
  int tls1_generate_master_secret(SSL *s, unsigned char *out, unsigned char *p, int len);
  int ssl3_connect(SSL *s);
  void gcm_gmult_4bit(u64 Xi[2],const u128 Htable[16]);
  void gcm_ghash_4bit(u64 Xi[2],const u128 Htable[16],const u8 *inp,size_t len);
}

extern void multipass_reset_round(bool isFirstCall);
extern void multipass_start_round (klee::Executor * theExecutor, bool isReplay);

//Distinction between prohib_fns and modeled_fns is that we sometimes may want to "jump back" into native execution
//for prohib_fns.  Modeled fns are always skipped and emulated with a return.
static const uint64_t prohib_fns [] = { (uint64_t) &AES_encrypt, (uint64_t) &ECDH_compute_key, (uint64_t) &EC_POINT_point2oct, (uint64_t) &EC_KEY_generate_key, (uint64_t) &SHA1_Update, (uint64_t) &SHA1_Final, (uint64_t) &SHA256_Update, (uint64_t) &SHA256_Final, (uint64_t) &gcm_gmult_4bit, (uint64_t) &gcm_ghash_4bit, (uint64_t) &tls1_generate_master_secret };
#endif // TASE_OPENSSL


// Network capture for Cliver
extern "C" { int ktest_connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
  int ktest_select(int nfds, fd_set *readfds, fd_set *writefds,
		   fd_set *exceptfds, struct timeval *timeout);
  ssize_t ktest_writesocket(int fd, const void *buf, size_t count);
  ssize_t ktest_readsocket(int fd, void *buf, size_t count);
  // stdin capture for Cliver
  int ktest_raw_read_stdin(void *buf, int siz);
  // Random number generator capture for Cliver
  int ktest_RAND_bytes(unsigned char *buf, int num);
  int ktest_RAND_pseudo_bytes(unsigned char *buf, int num);
  // Time capture for Cliver (actually unnecessary!)
  time_t ktest_time(time_t *t);
  // TLS Master Secret capture for Cliver
  void ktest_master_secret(unsigned char *ms, int len);
  void ktest_start(const char *filename, enum kTestMode mode);
  void ktest_finish();               // write capture to file  
}


namespace {
  cl::opt<bool>
  DumpStatesOnHalt("dump-states-on-halt",
                   cl::init(true),
		   cl::desc("Dump test cases for all active states on exit (default=on)"));
  
  cl::opt<bool>
  AllowExternalSymCalls("allow-external-sym-calls",
                        cl::init(false),
			cl::desc("Allow calls with symbolic arguments to external functions.  This concretizes the symbolic arguments.  (default=off)"));

  /// The different query logging solvers that can switched on/off
  enum PrintDebugInstructionsType {
    STDERR_ALL, ///
    STDERR_SRC,
    STDERR_COMPACT,
    FILE_ALL,    ///
    FILE_SRC,    ///
    FILE_COMPACT ///
  };

  llvm::cl::bits<PrintDebugInstructionsType> DebugPrintInstructions(
      "debug-print-instructions",
      llvm::cl::desc("Log instructions during execution."),
      llvm::cl::values(
          clEnumValN(STDERR_ALL, "all:stderr", "Log all instructions to stderr "
                                               "in format [src, inst_id, "
                                               "llvm_inst]"),
          clEnumValN(STDERR_SRC, "src:stderr",
                     "Log all instructions to stderr in format [src, inst_id]"),
          clEnumValN(STDERR_COMPACT, "compact:stderr",
                     "Log all instructions to stderr in format [inst_id]"),
          clEnumValN(FILE_ALL, "all:file", "Log all instructions to file "
                                           "instructions.txt in format [src, "
                                           "inst_id, llvm_inst]"),
          clEnumValN(FILE_SRC, "src:file", "Log all instructions to file "
                                           "instructions.txt in format [src, "
                                           "inst_id]"),
          clEnumValN(FILE_COMPACT, "compact:file",
                     "Log all instructions to file instructions.txt in format "
                     "[inst_id]")
          KLEE_LLVM_CL_VAL_END),
      llvm::cl::CommaSeparated);
#ifdef HAVE_ZLIB_H
  cl::opt<bool> DebugCompressInstructions(
      "debug-compress-instructions", cl::init(false),
      cl::desc("Compress the logged instructions in gzip format."));
#endif

  cl::opt<bool>
  DebugCheckForImpliedValues("debug-check-for-implied-values");


  cl::opt<bool>
  SimplifySymIndices("simplify-sym-indices",
                     cl::init(false),
		     cl::desc("Simplify symbolic accesses using equalities from other constraints (default=off)"));

  cl::opt<bool>
  EqualitySubstitution("equality-substitution",
		       cl::init(true),
		       cl::desc("Simplify equality expressions before querying the solver (default=on)."));
 
  cl::opt<unsigned>
  MaxSymArraySize("max-sym-array-size",
                  cl::init(0));

  cl::opt<bool>
  SuppressExternalWarnings("suppress-external-warnings",
			   cl::init(false),
			   cl::desc("Supress warnings about calling external functions."));

  cl::opt<bool>
  AllExternalWarnings("all-external-warnings",
		      cl::init(false),
		      cl::desc("Issue an warning everytime an external call is made," 
			       "as opposed to once per function (default=off)"));

  cl::opt<bool>
  OnlyOutputStatesCoveringNew("only-output-states-covering-new",
                              cl::init(false),
			      cl::desc("Only output test cases covering new code (default=off)."));

  cl::opt<bool>
  EmitAllErrors("emit-all-errors",
                cl::init(false),
                cl::desc("Generate tests cases for all errors "
                         "(default=off, i.e. one per (error,instruction) pair)"));
  
  cl::opt<bool>
  NoExternals("no-externals", 
           cl::desc("Do not allow external function calls (default=off)"));

  cl::opt<bool>
  AlwaysOutputSeeds("always-output-seeds",
		    cl::init(true));

  cl::opt<bool>
  OnlyReplaySeeds("only-replay-seeds",
		  cl::init(false),
                  cl::desc("Discard states that do not have a seed (default=off)."));
 
  cl::opt<bool>
  OnlySeed("only-seed",
	   cl::init(false),
           cl::desc("Stop execution after seeding is done without doing regular search (default=off)."));
 
  cl::opt<bool>
  AllowSeedExtension("allow-seed-extension",
		     cl::init(false),
                     cl::desc("Allow extra (unbound) values to become symbolic during seeding (default=false)."));
 
  cl::opt<bool>
  ZeroSeedExtension("zero-seed-extension",
		    cl::init(false),
		    cl::desc("(default=off)"));
 
  cl::opt<bool>
  AllowSeedTruncation("allow-seed-truncation",
		      cl::init(false),
                      cl::desc("Allow smaller buffers than in seeds (default=off)."));
 
  cl::opt<bool>
  NamedSeedMatching("named-seed-matching",
		    cl::init(false),
                    cl::desc("Use names to match symbolic objects to inputs (default=off)."));

  cl::opt<double>
  MaxStaticForkPct("max-static-fork-pct", 
		   cl::init(1.),
		   cl::desc("(default=1.0)"));

  cl::opt<double>
  MaxStaticSolvePct("max-static-solve-pct",
		    cl::init(1.),
		    cl::desc("(default=1.0)"));

  cl::opt<double>
  MaxStaticCPForkPct("max-static-cpfork-pct", 
		     cl::init(1.),
		     cl::desc("(default=1.0)"));

  cl::opt<double>
  MaxStaticCPSolvePct("max-static-cpsolve-pct",
		      cl::init(1.),
		      cl::desc("(default=1.0)"));

  cl::opt<double>
  MaxInstructionTime("max-instruction-time",
                     cl::desc("Only allow a single instruction to take this much time (default=0s (off)). Enables --use-forked-solver"),
                     cl::init(0));
  
  cl::opt<double>
  SeedTime("seed-time",
           cl::desc("Amount of time to dedicate to seeds, before normal search (default=0 (off))"),
           cl::init(0));
  
  cl::list<Executor::TerminateReason>
  ExitOnErrorType("exit-on-error-type",
		  cl::desc("Stop execution after reaching a specified condition.  (default=off)"),
		  cl::values(
		    clEnumValN(Executor::Abort, "Abort", "The program crashed"),
		    clEnumValN(Executor::Assert, "Assert", "An assertion was hit"),
		    clEnumValN(Executor::BadVectorAccess, "BadVectorAccess", "Vector accessed out of bounds"),
		    clEnumValN(Executor::Exec, "Exec", "Trying to execute an unexpected instruction"),
		    clEnumValN(Executor::External, "External", "External objects referenced"),
		    clEnumValN(Executor::Free, "Free", "Freeing invalid memory"),
		    clEnumValN(Executor::Model, "Model", "Memory model limit hit"),
		    clEnumValN(Executor::Overflow, "Overflow", "An overflow occurred"),
		    clEnumValN(Executor::Ptr, "Ptr", "Pointer error"),
		    clEnumValN(Executor::ReadOnly, "ReadOnly", "Write to read-only memory"),
		    clEnumValN(Executor::ReportError, "ReportError", "klee_report_error called"),
		    clEnumValN(Executor::User, "User", "Wrong klee_* functions invocation"),
		    clEnumValN(Executor::Unhandled, "Unhandled", "Unhandled instruction hit")
		    KLEE_LLVM_CL_VAL_END),
		  cl::ZeroOrMore);

  cl::opt<unsigned long long>
  StopAfterNInstructions("stop-after-n-instructions",
                         cl::desc("Stop execution after specified number of instructions (default=0 (off))"),
                         cl::init(0));
  
  cl::opt<unsigned>
  MaxForks("max-forks",
           cl::desc("Only fork this many times (default=-1 (off))"),
           cl::init(~0u));
  
  cl::opt<unsigned>
  MaxDepth("max-depth",
           cl::desc("Only allow this many symbolic branches (default=0 (off))"),
           cl::init(0));
  
  cl::opt<unsigned>
  MaxMemory("max-memory",
            cl::desc("Refuse to fork when above this amount of memory (in MB, default=2000)"),
            cl::init(2000));

  cl::opt<bool>
  MaxMemoryInhibit("max-memory-inhibit",
            cl::desc("Inhibit forking at memory cap (vs. random terminate) (default=on)"),
            cl::init(true));
}



EXECUTION_STATE ex_state;
ABORT_INFO abort_info{};

// only occurs w/ mixed-mode checks. INTERP_ONLY mode makes this always false, see update()
bool EXECUTION_STATE::is_concrete() const {
  return (txn_status & CONCRETE) != 0;
}

// singleStepping makes this always true, see update()
bool EXECUTION_STATE::is_txn_start() const {
  return (txn_status & TXN_START) != 0;
}

bool EXECUTION_STATE::is_singleStepping() const {
  return (mode & SSTEP) != 0;
}

bool EXECUTION_STATE::is_mixedMode() const {
  return (mode & MIXED) != 0;
}

// bounceback disabled in INTERP_ONLY mode and when SSTEP is set, will always be false (see constructor in Executor.h)
bool EXECUTION_STATE::is_bouncing() const {
  return (mode & BOUNCE) != 0;
}

bool EXECUTION_STATE::istate_none_of(uint16_t mask) const {
  return (istate & mask) == 0;
}

// check concreteness, transaction start, and update state based on abort type
void EXECUTION_STATE::update(ABORT_INFO::ABORT_TYPE type){
  bool conc = (mode & MIXED) != 0 && !tase_buf_has_taint((void *) &target_ctx_gregs[0], TASE_NGREG * TASE_GREG_SIZE);
  bool start = (mode & SSTEP) != 0 || cartridge_entry_points.find(target_ctx_gregs[GREG_RIP].u64) != cartridge_entry_points.end();
  txn_status = (conc ? CONCRETE : 0) | (start ? TXN_START : 0);

  switch( type ){
  case ABORT_INFO::MODEL:
    istate = MODEL;
    break;
  case ABORT_INFO::POISON:
    if( (mode & SSTEP) != 0 ) {
      istate = INTERP;
      break;
    }
  case ABORT_INFO::UNKNOWN:
  case ABORT_INFO::OTHER:
    if( (mode & BOUNCE) != 0 ) {
      if( tran_max > 0 ) {
	istate = BOUNCEBACK;
	target_ctx_gregs[GREG_RIP].u64 -= bounceback_offset;
      } else {
	istate = FAULT;
      }
    }
  }
}


void EXECUTION_STATE::update(){
  bool conc = (mode & MIXED) != 0 && !tase_buf_has_taint((void *) &target_ctx_gregs[0], TASE_NGREG * TASE_GREG_SIZE);
  bool start = (mode & SSTEP) != 0 || cartridge_entry_points.find(target_ctx_gregs[GREG_RIP].u64) != cartridge_entry_points.end();
  txn_status = (conc ? CONCRETE : 0) | (start ? TXN_START : 0);

  if( (mode & MIXED) != 0 && taseDebug ) {
    std::cout << "Inst " << (start ? "begins" : "doesn't begin") << " transaction\n";
    std::cout << "Registers " << (conc ? "are" : "aren't") << " concrete" << std::endl;    
  }
  
  if( conc && start ) {
    istate = RESUME;
  }
}


void ABORT_INFO::print_counts() const {
  std::cout << "BB_UNKNOWN: " << counts.unknown << "\n";
  std::cout << "BB_MODEL: " << counts.model << "\n";
  std::cout << "BB_POISON: " << counts.poison << "\n";
  std::cout << "BB_OTHER: " << counts.other << "\n";
}


void ABORT_INFO::reset_counts(){
  counts = ABORT_COUNTS{0, 0, 0, 0};
}



/*
  uint32_t abort_status: first bit is for xabort (or fake xabort)
  high 8 bits are for xabort return value 

  model abort is fake xabort with ff in high bits
  poison is xabort with 0-15 (0x00-0x0f - high bits
 */
void ABORT_INFO::classify_and_count(){
  uint8_t high_bits = (uint8_t) ((target_ctx.abort_status & TSX_XABORT_MASK)>>24);
  bool tsx = target_ctx.abort_status & (1<<TSX_XABORT);

  type = tsx ? ( high_bits == 0xff ? ABORT_TYPE::MODEL : (high_bits < 16) ?  ABORT_TYPE::POISON : ABORT_TYPE::UNKNOWN ) :
               high_bits == 0 ? ABORT_TYPE::UNKNOWN :
                                ABORT_TYPE::OTHER; 

  tran_max = (type & (ABORT_TYPE::UNKNOWN | ABORT_TYPE::OTHER)) == 0 ?
    (type & ABORT_TYPE::POISON) == 0 ? tran_max : high_bits :
    tran_max/2; 
  
  switch( type ){
  case ABORT_TYPE::MODEL:
    ex_state = EXECUTION_STATE::MODEL;
    counts.model++;
    break;
  case ABORT_TYPE::POISON:
    counts.poison++;
    break;
  case ABORT_TYPE::UNKNOWN:
    counts.unknown++;
    break;
  case ABORT_TYPE::OTHER:
    counts.other++;
    break;
  }
}


void ABORT_INFO::print() const {
  std::cout << "Abort type: ";
  switch( type ){
  case ABORT_TYPE::UNKNOWN:
    std::cout << "Unknown\n";
    break;
  case ABORT_TYPE::MODEL:
    std::cout << "Model\n";
    break;
  case ABORT_TYPE::POISON:
    std::cout << "Poison\n";
    break;
  case ABORT_TYPE::OTHER:
    std::cout << "Other/fallthrough\n";
    break;
  }
}



namespace klee {
  RNG theRNG;
}

const char *Executor::TerminateReasonNames[] = {
  [ Abort ] = "abort",
  [ Assert ] = "assert",
  [ BadVectorAccess ] = "bad_vector_access",
  [ Exec ] = "exec",
  [ External ] = "external",
  [ Free ] = "free",
  [ Model ] = "model",
  [ Overflow ] = "overflow",
  [ Ptr ] = "ptr",
  [ ReadOnly ] = "readonly",
  [ ReportError ] = "reporterror",
  [ User ] = "user",
  [ Unhandled ] = "xxx",
};


double interp_setup_time = 0.0;
double interp_find_fn_time = 0.0; //Should also account for interp_setup_time
double interp_run_time = 0.0;
double interp_cleanup_time = 0.0;
double interp_enter_time;
double interp_exit_time;
double solver_start_time;
double solver_end_time;
double solver_diff_time;

double run_start_time = 0;
double run_end_time = 0;

int run_interp_insts = 0;
int run_interp_traps = 0; //Number of traps to the interpreter
int run_model_count = 0; //Number of model calls
int run_bb_count = 0; //Number of basic blocks interpreted through

double run_model_time = 0;
double run_interp_time = 0;  //Total time in interpreter
double run_core_interp_time = 0; //Total time interpreting insts
double run_fork_time = 0;
double run_solver_time = 0;
double run_fault_time = 0;
double run_mem_op_time = 0;
double run_tmp_1_time = 0;
double run_tmp_2_time = 0;
double run_tmp_3_time = 0;
double mem_op_eval_time = 0;
double mo_resolve_time = 0;


void reset_run_timers() {

  run_start_time = util::getWallTime();
  interp_enter_time = util::getWallTime();

  run_interp_insts = 0;
  run_interp_traps = 0;
  run_model_count = 0;
  run_bb_count = 0;

  run_model_time = 0;
  run_interp_time = 0;
  run_core_interp_time = 0;
  run_fork_time = 0;
  run_solver_time = 0;
  run_fault_time =0;
  run_mem_op_time = 0;
  run_tmp_1_time = 0;
  run_tmp_2_time = 0;
  run_tmp_3_time = 0;
  mem_op_eval_time = 0;
  mo_resolve_time = 0;


  abort_info.reset_counts();
}

void print_run_timers() {
  if (!noLog) {
    printf(" %lf seconds elapsed since target started \n", (util::getWallTime() - target_start_time));
    printf(" --- Printing run timers ----\n");
    printf("ID string is %s \n", worker_ID_stream.str().c_str());
    printf("Curr Unique log ID is %s\n", curr_unique_log_ID.c_str());
    printf("Prev Unique log ID is %s\n", prev_unique_log_ID.c_str());

    double totalRunTime =  util::getWallTime() - run_start_time;
    run_interp_time += (util::getWallTime() - interp_enter_time);
    printf("Total basic blocks %d \n", run_interp_insts );
    printf("Total run time    : %lf \n",  totalRunTime);
    printf(" - Model time     : %lf \n", run_model_time);
    printf(" - Interp time    : %lf \n", run_interp_time);
    printf("       -Core      : %lf \n", run_core_interp_time);
    printf("       -Solver    : %lf \n", run_solver_time);
    printf("       -Fork      : %lf \n", run_fork_time );
    printf("       -Fault time: %lf \n", run_fault_time);
    printf("       -Mem op    : %lf \n", run_mem_op_time);
    printf("       -TMP1 time : %lf \n", run_tmp_1_time);
    printf("       -TMP2 time : %lf \n", run_tmp_2_time);
    printf("       -TMP3 time : %lf \n", run_tmp_3_time);
    printf("MEM OP TIME       : %lf \n", run_mem_op_time);
    printf(" mem op eval args : %lf \n", mem_op_eval_time);
    printf("   -mo_resolve_t  : %lf \n", mo_resolve_time);

    if (taseDebug) {
      abort_info.print();
    }
    abort_info.reset_counts();


    FILE * logFile = fopen(curr_unique_log_ID.c_str(), "w+");
    fprintf(logFile, "Prev log name, Round, Pass, Total Runtime, Interp Time, Solver Time, Fork Time, Core Interp Time,TMP1, TMP2, TMP3, RUN INTERP TRAPS, RUN BB COUNT, RUN MODEL COUNT \n");
    fprintf(logFile, "%s, %d, %d,", prev_unique_log_ID.c_str(), round_count, pass_count);
    fprintf(logFile, " %lf, %lf, %lf, %lf, %d, %lf, %lf, %lf, %lf, %d, %d, %d \n", totalRunTime, run_interp_time, run_solver_time, run_fork_time, run_interp_insts, run_core_interp_time, run_tmp_1_time, run_tmp_2_time, run_tmp_3_time, run_interp_traps, run_bb_count, run_model_count);
    fclose(logFile);

  }
}

std::string makeNewLogID () {
  static int logCtr = 0;
  logCtr++;
  std::stringstream stream;
  int i = getpid();
  stream << "LOG." << i << ".";
  struct timeval t;
  gettimeofday(&t, NULL);
  stream << t.tv_sec << "." << t.tv_usec << "." << logCtr;

  return stream.str();

}

void cycleTASELogs(bool isReplay) {
  if (isReplay) {
    prev_unique_log_ID = prev_worker_ID;
  } else {
    prev_unique_log_ID = curr_unique_log_ID;
  }
  curr_unique_log_ID = makeNewLogID();


  prev_worker_ID = worker_ID_stream.str();
  reset_run_timers();

  if (!noLog) {
    fflush(stdout);
    double T0 = util::getWallTime();
    int i = getpid();
    worker_ID_stream << ".";
    worker_ID_stream << i;
    std::string pidString ;

    pidString = worker_ID_stream.str();
    if (pidString.size() > 250) {
      std::cout << "Cycling log names due to large size" << std::endl;
      worker_ID_stream.str("");
      worker_ID_stream << "Monitor.Wrapped.";
      worker_ID_stream << i;
      pidString = worker_ID_stream.str();
      std::cout << "Cycled log name is " << pidString.c_str() << std::endl;

    }

    if (prev_stdout_log != NULL)
      fclose(prev_stdout_log);
    if (prev_stderr_log != NULL)
      fclose(prev_stderr_log);

    prev_stdout_log = freopen(pidString.c_str(), "w", stdout);

    fflush(stdout);
    fflush(stderr);

    double T1 = util::getWallTime();
    std::cout << "Spent " << ( T1 - T0 ) << " seconds resetting log streams" << std::endl;;
    std::cout << "Time since start is " << ( util::getWallTime() - target_start_time ) << std::endl;
    if (prev_stdout_log == NULL ) {
      std::cout << "ERROR opening new file for child process logging" << std::endl;
      std::cerr << "ERROR opening new file for child process logging for pid " << i << std::endl;
      //      worker_exit(EXIT_FAILURE);
      exit(1);
    }
  }

  run_interp_time = 0;
  interp_enter_time = util::getWallTime();
}


void measure_interp_time(uint64_t interpCtr_init, uint64_t rip) {

  double interp_exit_time = util::getWallTime();
  double diff_time = (interp_exit_time) - (interp_enter_time);
  run_interp_time += diff_time;

  if (target_ctx.abort_status == 0) {
    run_fault_time += diff_time;
  }

  if (!noLog) {
    printf("Elapsed time is %lf at interpCtr %lu rip 0x%lx with %lu interpreter loops and abort code 0x%08x \n", diff_time, interpCtr, rip, interpCtr - interpCtr_init, target_ctx.abort_status);
    printf("------------------------------\n");
  }
}


Executor::Executor(LLVMContext &ctx, const InterpreterOptions &opts,
    InterpreterHandler *ih)
    : Interpreter(opts), kmodule(0), interpreterHandler(ih), searcher(0),
      externalDispatcher(new ExternalDispatcher(ctx)), statsTracker(0),
      pathWriter(0), symPathWriter(0), specialFunctionHandler(0),
      processTree(0), replayKTest(0), replayPath(0), usingSeeds(0),
      atMemoryLimit(false), inhibitForking(false), haltExecution(false),
      ivcEnabled(false),
      coreSolverTimeout(MaxCoreSolverTime != 0 && MaxInstructionTime != 0
                            ? std::min(MaxCoreSolverTime, MaxInstructionTime)
                            : std::max(MaxCoreSolverTime, MaxInstructionTime)),
      debugInstFile(0), debugLogBuffer(debugBufferString) {

  if (coreSolverTimeout) UseForkedCoreSolver = true;
  Solver *coreSolver = klee::createCoreSolver(CoreSolverToUse);
  if (!coreSolver) {
    klee_error("Failed to create core solver\n");
  }

  Solver *solver = constructSolverChain(
      coreSolver,
      interpreterHandler->getOutputFilename(ALL_QUERIES_SMT2_FILE_NAME),
      interpreterHandler->getOutputFilename(SOLVER_QUERIES_SMT2_FILE_NAME),
      interpreterHandler->getOutputFilename(ALL_QUERIES_KQUERY_FILE_NAME),
      interpreterHandler->getOutputFilename(SOLVER_QUERIES_KQUERY_FILE_NAME));

  this->solver = new TimingSolver(solver, EqualitySubstitution);
  memory = new MemoryManager(&arrayCache);

  initializeSearchOptions();

  if (DebugPrintInstructions.isSet(FILE_ALL) ||
      DebugPrintInstructions.isSet(FILE_COMPACT) ||
      DebugPrintInstructions.isSet(FILE_SRC)) {
    std::string debug_file_name =
        interpreterHandler->getOutputFilename("instructions.txt");
    std::string ErrorInfo;
#ifdef HAVE_ZLIB_H
    if (!DebugCompressInstructions) {
#endif

#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 6)
    std::error_code ec;
    debugInstFile = new llvm::raw_fd_ostream(debug_file_name.c_str(), ec,
                                             llvm::sys::fs::OpenFlags::F_Text);
    if (ec)
	    ErrorInfo = ec.message();
#elif LLVM_VERSION_CODE >= LLVM_VERSION(3, 5)
    debugInstFile = new llvm::raw_fd_ostream(debug_file_name.c_str(), ErrorInfo,
                                             llvm::sys::fs::OpenFlags::F_Text);
#else
    debugInstFile =
        new llvm::raw_fd_ostream(debug_file_name.c_str(), ErrorInfo);
#endif
#ifdef HAVE_ZLIB_H
    } else {
      debugInstFile = new compressed_fd_ostream(
          (debug_file_name + ".gz").c_str(), ErrorInfo);
    }
#endif
    if (ErrorInfo != "") {
      klee_error("Could not open file %s : %s", debug_file_name.c_str(),
                 ErrorInfo.c_str());
    }
  }
}


const Module *Executor::setModule(llvm::Module *module, 
                                  const ModuleOptions &opts) {
  
  assert (!kmodule && "kmodule fail \n");
  assert (module && "module fail \n");
  assert(!kmodule && module && "can only register one module"); // XXX gross
  kmodule = new KModule(module);
  // Initialize the context.
  DataLayout *TD = kmodule->targetData;
  Context::initialize(TD->isLittleEndian(),
                      (Expr::Width) TD->getPointerSizeInBits());
  specialFunctionHandler = new SpecialFunctionHandler(*this);

  specialFunctionHandler->prepare();
  kmodule->prepare(opts, interpreterHandler);
  specialFunctionHandler->bind();

  //Can't use the KLEE built in stats trackers because we fork a
  //process for each state in TASE.
  /*
  if (StatsTracker::useStatistics() || userSearcherRequiresMD2U()) {
    statsTracker = 
      new StatsTracker(*this,
                       interpreterHandler->getOutputFilename("assembly.ll"),
                       userSearcherRequiresMD2U());
  }
  */

  return module;
}

Executor::~Executor() {
  delete memory;
  delete externalDispatcher;
  delete processTree;
  delete specialFunctionHandler;
  delete statsTracker;
  delete solver;
  delete kmodule;
  while(!timers.empty()) {
    delete timers.back();
    timers.pop_back();
  }
  delete debugInstFile;
}

/***/

void Executor::initializeGlobalObject(ExecutionState &state, ObjectState *os,
                                      const Constant *c, 
                                      unsigned offset) {
  DataLayout *targetData = kmodule->targetData;
  if (const ConstantVector *cp = dyn_cast<ConstantVector>(c)) {
    unsigned elementSize =
      targetData->getTypeStoreSize(cp->getType()->getElementType());
    for (unsigned i=0, e=cp->getNumOperands(); i != e; ++i)
      initializeGlobalObject(state, os, cp->getOperand(i), 
			     offset + i*elementSize);
  } else if (isa<ConstantAggregateZero>(c)) {
    unsigned i, size = targetData->getTypeStoreSize(c->getType());
    for (i=0; i<size; i++)
      os->write8(offset+i, (uint8_t) 0);
  } else if (const ConstantArray *ca = dyn_cast<ConstantArray>(c)) {
    unsigned elementSize =
      targetData->getTypeStoreSize(ca->getType()->getElementType());
    for (unsigned i=0, e=ca->getNumOperands(); i != e; ++i)
      initializeGlobalObject(state, os, ca->getOperand(i), 
			     offset + i*elementSize);
  } else if (const ConstantStruct *cs = dyn_cast<ConstantStruct>(c)) {
    const StructLayout *sl =
      targetData->getStructLayout(cast<StructType>(cs->getType()));
    for (unsigned i=0, e=cs->getNumOperands(); i != e; ++i)
      initializeGlobalObject(state, os, cs->getOperand(i), 
			     offset + sl->getElementOffset(i));
  } else if (const ConstantDataSequential *cds =
               dyn_cast<ConstantDataSequential>(c)) {
    unsigned elementSize =
      targetData->getTypeStoreSize(cds->getElementType());
    for (unsigned i=0, e=cds->getNumElements(); i != e; ++i)
      initializeGlobalObject(state, os, cds->getElementAsConstant(i),
                             offset + i*elementSize);
  } else if (!isa<UndefValue>(c)) {
    unsigned StoreBits = targetData->getTypeStoreSizeInBits(c->getType());
    ref<ConstantExpr> C = evalConstant(c);

    // Extend the constant if necessary;
    assert(StoreBits >= C->getWidth() && "Invalid store size!");
    if (StoreBits > C->getWidth())
      C = C->ZExt(StoreBits);

    os->write(offset, C);
  }
}

MemoryObject* Executor::addExternalObject(ExecutionState &state, 
                                           void *addr, unsigned size, 
                                           bool isReadOnly, const std::string& name, bool forTASE) {
  
  MemoryObject *mo = memory->allocateFixed((uint64_t) (unsigned long) addr, 
                                           size, 0);
  mo->setName(name);
  ObjectState *os = bindObjectInState(state, mo, false, NULL, forTASE);
  
  //printf("Mapping external buf: mo->address is 0x%lx, size is 0x%x \n", mo->address, size);
  os->concreteStore = (uint8_t *) mo->address;
  //for(unsigned i = 0; i < size; i++)
  // os->write8(i, ((uint8_t*)addr)[i]);
  if(isReadOnly)
    os->setReadOnly(true);  
  return mo;
}


bool Executor::addExternalObjectCheck(ExecutionState &state, 
                                           void *addr, unsigned size, 
                                           bool isReadOnly, const std::string& name, bool forTASE) {
  
  ObjectPair op;
  ref<ConstantExpr> CE = ConstantExpr::create((uint64_t) addr, Expr::Int64);
  if ( state.addressSpace.resolveOne(CE, op) ) {
    if ( taseDebug ) {
       std::cout << "mapped address resolved to MO: " << op.first->name << std::endl;
    }
    return false;
  }
  
  if ( taseDebug ) {
    std::cout << "no MO found, allocating" << std::endl;
  }
  
  MemoryObject *mo = memory->allocateFixed((uint64_t) (unsigned long) addr, 
                                           size, 0);
  mo->setName(name);
  ObjectState *os = bindObjectInState(state, mo, false, NULL, forTASE);
  
  os->concreteStore = (uint8_t *) mo->address;

  if(isReadOnly)
    os->setReadOnly(true);
  
  return true;
}

  

extern void *__dso_handle __attribute__ ((__weak__));

void Executor::initializeGlobals(ExecutionState &state) {
  Module *m = kmodule->module;

  if (m->getModuleInlineAsm() != "")
    klee_warning("executable has module level assembly (ignoring)");
  // represent function globals using the address of the actual llvm function
  // object. given that we use malloc to allocate memory in states this also
  // ensures that we won't conflict. we don't need to allocate a memory object
  // since reading/writing via a function pointer is unsupported anyway.
  for (Module::iterator i = m->begin(), ie = m->end(); i != ie; ++i) {
    Function *f = &*i;
    ref<ConstantExpr> addr(0);

    // If the symbol has external weak linkage then it is implicitly
    // not defined in this module; if it isn't resolvable then it
    // should be null.
    if (f->hasExternalWeakLinkage() && 
        !externalDispatcher->resolveSymbol(f->getName())) {
      addr = Expr::createPointer(0);
    } else {
      addr = Expr::createPointer((unsigned long) (void*) f);
      legalFunctions.insert((uint64_t) (unsigned long) (void*) f);
    }
    
    globalAddresses.insert(std::make_pair(f, addr));
  }

  // Disabled, we don't want to promote use of live externals.
#ifdef HAVE_CTYPE_EXTERNALS
#ifndef WINDOWS
#ifndef DARWIN
  /* From /usr/include/errno.h: it [errno] is a per-thread variable. */
  int *errno_addr = __errno_location();
  addExternalObject(state, (void *)errno_addr, sizeof *errno_addr, false);

  /* from /usr/include/ctype.h:
       These point into arrays of 384, so they can be indexed by any `unsigned
       char' value [0,255]; by EOF (-1); or by any `signed char' value
       [-128,-1).  ISO C requires that the ctype functions work for `unsigned */
  const uint16_t **addr = __ctype_b_loc();
  addExternalObject(state, const_cast<uint16_t*>(*addr-128),
                    384 * sizeof **addr, true);
  addExternalObject(state, addr, sizeof(*addr), true);
    
  const int32_t **lower_addr = __ctype_tolower_loc();
  addExternalObject(state, const_cast<int32_t*>(*lower_addr-128),
                    384 * sizeof **lower_addr, true);
  addExternalObject(state, lower_addr, sizeof(*lower_addr), true);
  
  const int32_t **upper_addr = __ctype_toupper_loc();
  addExternalObject(state, const_cast<int32_t*>(*upper_addr-128),
                    384 * sizeof **upper_addr, true);
  addExternalObject(state, upper_addr, sizeof(*upper_addr), true);
#endif
#endif
#endif

  // allocate and initialize globals, done in two passes since we may
  // need address of a global in order to initialize some other one.

  // allocate memory objects for all globals
  for (Module::const_global_iterator i = m->global_begin(),
         e = m->global_end();
       i != e; ++i) {
    const GlobalVariable *v = &*i;
    size_t globalObjectAlignment = getAllocationAlignment(v);
    if (i->isDeclaration()) {
      // FIXME: We have no general way of handling unknown external
      // symbols. If we really cared about making external stuff work
      // better we could support user definition, or use the EXE style
      // hack where we check the object file information.

      Type *ty = i->getType()->getElementType();
      uint64_t size = 0;
      if (ty->isSized()) {
	size = kmodule->targetData->getTypeStoreSize(ty);
      } else {
        klee_warning("Type for %.*s is not sized", (int)i->getName().size(),
			i->getName().data());
      }

      // XXX - DWD - hardcode some things until we decide how to fix.
#ifndef WINDOWS
      if (i->getName() == "_ZTVN10__cxxabiv117__class_type_infoE") {
        size = 0x2C;
      } else if (i->getName() == "_ZTVN10__cxxabiv120__si_class_type_infoE") {
        size = 0x2C;
      } else if (i->getName() == "_ZTVN10__cxxabiv121__vmi_class_type_infoE") {
        size = 0x2C;
      }
#endif

      if (size == 0) {
        klee_warning("Unable to find size for global variable: %.*s (use will result in out of bounds access)",
			(int)i->getName().size(), i->getName().data());
      }

      MemoryObject *mo = memory->allocate(size, /*isLocal=*/false,
                                          /*isGlobal=*/true, /*allocSite=*/v,
                                          /*alignment=*/globalObjectAlignment);
      ObjectState *os = bindObjectInState(state, mo, false);
      globalObjects.insert(std::make_pair(v, mo));
      globalAddresses.insert(std::make_pair(v, mo->getBaseExpr()));

      // Program already running = object already initialized.  Read
      // concrete value and write it to our copy.
      if (size) {
        void *addr;
        if (i->getName() == "__dso_handle") {
          addr = &__dso_handle; // wtf ?
        } else {
          addr = externalDispatcher->resolveSymbol(i->getName());
        }
        if (!addr)
          klee_error("unable to load symbol(%s) while initializing globals.", 
                     i->getName().data());

        for (unsigned offset=0; offset<mo->size; offset++){
          os->write8(offset, ((unsigned char*)addr)[offset]);
	}
      }
    } else {
      Type *ty = i->getType()->getElementType();
      uint64_t size = kmodule->targetData->getTypeStoreSize(ty);
      MemoryObject *mo = memory->allocate(size, /*isLocal=*/false,
                                          /*isGlobal=*/true, /*allocSite=*/v,
                                          /*alignment=*/globalObjectAlignment);
      if (!mo)
        llvm::report_fatal_error("out of memory");
      ObjectState *os = bindObjectInState(state, mo, false);
      globalObjects.insert(std::make_pair(v, mo));
      globalAddresses.insert(std::make_pair(v, mo->getBaseExpr()));

      if (!i->hasInitializer())
          os->initializeToRandom();
    }
  }
  
  // link aliases to their definitions (if bound)
  for (Module::alias_iterator i = m->alias_begin(), ie = m->alias_end(); 
       i != ie; ++i) {
    // Map the alias to its aliasee's address. This works because we have
    // addresses for everything, even undefined functions. 
    globalAddresses.insert(std::make_pair(&*i, evalConstant(i->getAliasee())));
  }

  // once all objects are allocated, do the actual initialization
  for (Module::const_global_iterator i = m->global_begin(),
         e = m->global_end();
       i != e; ++i) {
    if (i->hasInitializer()) {
      const GlobalVariable *v = &*i;
      MemoryObject *mo = globalObjects.find(v)->second;
      const ObjectState *os = state.addressSpace.findObject(mo);
      assert(os);
      ObjectState *wos = state.addressSpace.getWriteable(mo, os);
      
      initializeGlobalObject(state, wos, i->getInitializer(), 0);
      // if(i->isConstant()) os->setReadOnly(true);
    }
  }
}

void Executor::branch(ExecutionState &state, 
                      const std::vector< ref<Expr> > &conditions,
                      std::vector<ExecutionState*> &result) {
  TimerStatIncrementer timer(stats::forkTime);
  unsigned N = conditions.size();
  assert(N);

  if (MaxForks!=~0u && stats::forks >= MaxForks) {
    unsigned next = theRNG.getInt32() % N;
    for (unsigned i=0; i<N; ++i) {
      if (i == next) {
        result.push_back(&state);
      } else {
        result.push_back(NULL);
      }
    }
  } else {
    stats::forks += N-1;

    // XXX do proper balance or keep random?
    result.push_back(&state);
    for (unsigned i=1; i<N; ++i) {
      ExecutionState *es = result[theRNG.getInt32() % i];
      ExecutionState *ns = es->branch();
      addedStates.push_back(ns);
      result.push_back(ns);
      es->ptreeNode->data = 0;
      std::pair<PTree::Node*,PTree::Node*> res = 
        processTree->split(es->ptreeNode, ns, es);
      ns->ptreeNode = res.first;
      es->ptreeNode = res.second;
    }
  }

  // If necessary redistribute seeds to match conditions, killing
  // states if necessary due to OnlyReplaySeeds (inefficient but
  // simple).
  
  std::map< ExecutionState*, std::vector<SeedInfo> >::iterator it = 
    seedMap.find(&state);
  if (it != seedMap.end()) {
    std::vector<SeedInfo> seeds = it->second;
    seedMap.erase(it);

    // Assume each seed only satisfies one condition (necessarily true
    // when conditions are mutually exclusive and their conjunction is
    // a tautology).
    for (std::vector<SeedInfo>::iterator siit = seeds.begin(), 
           siie = seeds.end(); siit != siie; ++siit) {
      unsigned i;
      for (i=0; i<N; ++i) {
        ref<ConstantExpr> res;
        bool success = 
          solver->getValue(state, siit->assignment.evaluate(conditions[i]), 
                           res);
        assert(success && "FIXME: Unhandled solver failure");
        (void) success;
        if (res->isTrue())
          break;
      }
      
      // If we didn't find a satisfying condition randomly pick one
      // (the seed will be patched).
      if (i==N)
        i = theRNG.getInt32() % N;

      // Extra check in case we're replaying seeds with a max-fork
      if (result[i])
        seedMap[result[i]].push_back(*siit);
    }

    if (OnlyReplaySeeds) {
      for (unsigned i=0; i<N; ++i) {
        if (result[i] && !seedMap.count(result[i])) {
          terminateState(*result[i]);
          result[i] = NULL;
        }
      } 
    }
  }

  for (unsigned i=0; i<N; ++i)
    if (result[i])
      addConstraint(*result[i], conditions[i]);
}

Executor::StatePair 
Executor::fork(ExecutionState &current, ref<Expr> condition, bool isInternal) {  
  //printf("Entering regular klee fork() \n");
  Solver::Validity res;
  std::map< ExecutionState*, std::vector<SeedInfo> >::iterator it = 
    seedMap.find(&current);
  bool isSeeding = it != seedMap.end();
  double timeout = coreSolverTimeout;
  if (isSeeding)
    timeout *= it->second.size();
  solver->setTimeout(timeout);
  if (taseDebug) {
    if (!isa<ConstantExpr> (condition)) {
      printf("DEBUG:FORK ctx is \n");
      printCtx();
      forkSolverCalls++;
    }
  }
  bool success = solver->evaluate(current, condition, res);
  solver->setTimeout(0);
  if (!success) {
    printf("INTERPRETER: QUERY TIMEOUT \n");
    current.pc = current.prevPC;
    terminateStateEarly(current, "Query timed out (fork).");
    return StatePair(0, 0);
  }

  //Either condition is always true, always false, or we need to fork.
  if (res==Solver::True) {
    if (taseDebug) {
      printf("DEBUG:FORK - Solver returned true \n");
      std::cout.flush();
    }
    return StatePair(&current, 0);
  } else if (res==Solver::False) {
    if (taseDebug) {
      printf("Debug:FORK - Solver returned false \n");
      std::cout.flush();
    }
    return StatePair(0, &current);
  } else {
    TimerStatIncrementer timer(stats::forkTime);
    ExecutionState *falseState, *trueState = &current;
    ++stats::forks;

    if ( taseDebug ) {
      std::cout << "tase_fork call in Executor::fork" << std::endl;
    }
    //ABH: This, along with "forkOnPossibleRIPValues", is one of
    //the two places we fork during path exploration in TASE.
    int parentPID = getpid();
    uint64_t rip = target_ctx_gregs[GREG_RIP].u64;
    //    int pid  = tase_fork(parentPID, rip);
    int pid = worker_fork(Stopped, Running);

    if (pid == 0 ) {      
      addConstraint(*GlobalExecutionStatePtr, Expr::createIsZero(condition));
    } else {
      addConstraint(*GlobalExecutionStatePtr, condition);
    }

   
    
    //Call to solver to make sure we're legit on this branch.
    printf("Calling solver for sanity check \n");
    std::vector< std::vector<unsigned char> > values;
    std::vector<const Array*> objects;
    for (unsigned i = 0; i != GlobalExecutionStatePtr->symbolics.size(); ++i)
      objects.push_back(GlobalExecutionStatePtr->symbolics[i].second);
    bool success = solver->getInitialValues(*GlobalExecutionStatePtr, objects, values);
    if (success)
      printf("Solver checked sanity \n");
    else {
      printf("Solver found invalid path \n");
      printCtx();
    }
    if (pid == 0)  {
      return StatePair(0, GlobalExecutionStatePtr);
    } else {
      return StatePair(GlobalExecutionStatePtr,0);
    }
  }
}

void Executor::addConstraint(ExecutionState &state, ref<Expr> condition) {

  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(condition)) {
    if (!CE->isTrue())
      llvm::report_fatal_error("attempt to add invalid constraint");
    return;
  }

  // Check to see if this constraint violates seeds.
  std::map< ExecutionState*, std::vector<SeedInfo> >::iterator it = 
    seedMap.find(&state);
  if (it != seedMap.end()) {
    bool warn = false;
    for (std::vector<SeedInfo>::iterator siit = it->second.begin(), 
           siie = it->second.end(); siit != siie; ++siit) {
      bool res;
      bool success = 
        solver->mustBeFalse(state, siit->assignment.evaluate(condition), res);
      assert(success && "FIXME: Unhandled solver failure");
      (void) success;
      if (res) {
        siit->patchSeed(state, condition, solver);
        warn = true;
      }
    }
    if (warn)
      klee_warning("seeds patched for violating constraint"); 
  }


  state.addConstraint(condition);

  
  if (ivcEnabled)
    doImpliedValueConcretization(state, condition, 
                                 ConstantExpr::alloc(1, Expr::Bool));


}

const Cell& Executor::eval(KInstruction *ki, unsigned index, 
                           ExecutionState &state) const {
  assert(index < ki->inst->getNumOperands());
  int vnumber = ki->operands[index];

  assert(vnumber != -1 &&
         "Invalid operand to eval(), not a value or constant!");

  // Determine if this is a constant or not.
  if (vnumber < 0) {
    unsigned index = -vnumber - 2;
    return kmodule->constantTable[index];
  } else {
    unsigned index = vnumber;
    StackFrame &sf = state.stack.back();
    return sf.locals[index];
  }
}

void Executor::bindLocal(KInstruction *target, ExecutionState &state, 
                         ref<Expr> value) {
  getDestCell(state, target).value = value;
}

void Executor::bindArgument(KFunction *kf, unsigned index, 
                            ExecutionState &state, ref<Expr> value) {
  getArgumentCell(state, kf, index).value = value;
}

ref<Expr> Executor::toUnique(const ExecutionState &state, 
                             ref<Expr> &e) {

  //e->dump();
  ref<Expr> result = e;

  if (!isa<ConstantExpr>(e)) {
    ref<ConstantExpr> value;
    bool isTrue = false;

    solver->setTimeout(coreSolverTimeout);      
    if (solver->getValue(state, e, value) &&
        solver->mustBeTrue(state, EqExpr::create(e, value), isTrue) &&
        isTrue)
      result = value;
    solver->setTimeout(0);
  }
  
  return result;
}


/* Concretize the given expression, and return a possible constant value. 
   'reason' is just a documentation string stating the reason for concretization. */
ref<klee::ConstantExpr> 
Executor::toConstant(ExecutionState &state, 
                     ref<Expr> e,
                     const char *reason) {
  e = state.constraints.simplifyExpr(e);
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(e))
    return CE;

  ref<ConstantExpr> value;
  bool success = solver->getValue(state, e, value);
  assert(success && "FIXME: Unhandled solver failure");
  (void) success;

  std::string str;
  llvm::raw_string_ostream os(str);
  os << "silently concretizing (reason: " << reason << ") expression " << e
     << " to value " << value << " (" << (*(state.pc)).info->file << ":"
     << (*(state.pc)).info->line << ")";

  if (AllExternalWarnings)
    klee_warning(reason, os.str().c_str());
  else
    klee_warning_once(reason, "%s", os.str().c_str());

  addConstraint(state, EqExpr::create(e, value));
    
  return value;
}

void Executor::executeGetValue(ExecutionState &state,
                               ref<Expr> e,
                               KInstruction *target) {
  e = state.constraints.simplifyExpr(e);
  std::map< ExecutionState*, std::vector<SeedInfo> >::iterator it = 
    seedMap.find(&state);
  if (it==seedMap.end() || isa<ConstantExpr>(e)) {
    ref<ConstantExpr> value;
    bool success = solver->getValue(state, e, value);
    assert(success && "FIXME: Unhandled solver failure");
    (void) success;
    bindLocal(target, state, value);
  } else {
    std::set< ref<Expr> > values;
    for (std::vector<SeedInfo>::iterator siit = it->second.begin(), 
           siie = it->second.end(); siit != siie; ++siit) {
      ref<ConstantExpr> value;
      bool success = 
        solver->getValue(state, siit->assignment.evaluate(e), value);
      assert(success && "FIXME: Unhandled solver failure");
      (void) success;
      values.insert(value);
    }
    
    std::vector< ref<Expr> > conditions;
    for (std::set< ref<Expr> >::iterator vit = values.begin(), 
           vie = values.end(); vit != vie; ++vit)
      conditions.push_back(EqExpr::create(e, *vit));

    std::vector<ExecutionState*> branches;
    branch(state, conditions, branches);
    
    std::vector<ExecutionState*>::iterator bit = branches.begin();
    for (std::set< ref<Expr> >::iterator vit = values.begin(), 
           vie = values.end(); vit != vie; ++vit) {
      ExecutionState *es = *bit;
      if (es)
        bindLocal(target, *es, *vit);
      ++bit;
    }
  }
}

void Executor::printDebugInstructions(ExecutionState &state) {
  // check do not print
  if (DebugPrintInstructions.getBits() == 0)
	  return;

  llvm::raw_ostream *stream = 0;
  if (DebugPrintInstructions.isSet(STDERR_ALL) ||
      DebugPrintInstructions.isSet(STDERR_SRC) ||
      DebugPrintInstructions.isSet(STDERR_COMPACT))
    stream = &llvm::errs();
  else
    stream = &debugLogBuffer;

  if (!DebugPrintInstructions.isSet(STDERR_COMPACT) &&
      !DebugPrintInstructions.isSet(FILE_COMPACT)) {
    (*stream) << "     ";
    state.pc->printFileLine(*stream);
    (*stream) << ":";
  }

  (*stream) << state.pc->info->assemblyLine;

  if (DebugPrintInstructions.isSet(STDERR_ALL) ||
      DebugPrintInstructions.isSet(FILE_ALL))
    (*stream) << ":" << *(state.pc->inst);
  (*stream) << "\n";

  if (DebugPrintInstructions.isSet(FILE_ALL) ||
      DebugPrintInstructions.isSet(FILE_COMPACT) ||
      DebugPrintInstructions.isSet(FILE_SRC)) {
    debugLogBuffer.flush();
    (*debugInstFile) << debugLogBuffer.str();
    debugBufferString = "";
  }
}

void Executor::stepInstruction(ExecutionState &state) {
  ++stats::instructions;
  state.prevPC = state.pc;
  ++state.pc;

  if (stats::instructions==StopAfterNInstructions)
    haltExecution = true;
}

void Executor::executeCall(ExecutionState &state, 
                           KInstruction *ki,
                           Function *f,
                           std::vector< ref<Expr> > &arguments) {
  Instruction *i = ki->inst;
  if (f && f->isDeclaration()) {
    switch(f->getIntrinsicID()) {
    case Intrinsic::not_intrinsic:
      // state may be destroyed by this call, cannot touch
      callExternalFunction(state, ki, f, arguments);
      break;
        
      // va_arg is handled by caller and intrinsic lowering, see comment for
      // ExecutionState::varargs
      case Intrinsic::vastart:  {
      printf("va_start encountered\n");
      fflush(stdout);
      StackFrame &sf = state.stack.back();

      // varargs can be zero if no varargs were provided
      if (!sf.varargs)
        return;

      // FIXME: This is really specific to the architecture, not the pointer
      // size. This happens to work for x86-32 and x86-64, however.
      Expr::Width WordSize = Context::get().getPointerWidth();
      if (WordSize == Expr::Int32) {
        executeMemoryOperation(state, true, arguments[0], 
                               sf.varargs->getBaseExpr(), 0, "vastart 0, instCtr: " + std::to_string(instCtr));
      } else {
        assert(WordSize == Expr::Int64 && "Unknown word size!");

        // x86-64 has quite complicated calling convention. However,
        // instead of implementing it, we can do a simple hack: just
        // make a function believe that all varargs are on stack.
        executeMemoryOperation(state, true, arguments[0],
                               ConstantExpr::create(48, 32), 0, "vastart 1, instCtr: " + std::to_string(instCtr)); // gp_offset
        executeMemoryOperation(state, true,
                               AddExpr::create(arguments[0], 
                                               ConstantExpr::create(4, 64)),
                               ConstantExpr::create(304, 32), 0, "vastart 2, instCtr: " + std::to_string(instCtr)); // fp_offset
        executeMemoryOperation(state, true,
                               AddExpr::create(arguments[0], 
                                               ConstantExpr::create(8, 64)),
                               sf.varargs->getBaseExpr(), 0, "vastart 3, instCtr: " + std::to_string(instCtr)); // overflow_arg_area
        executeMemoryOperation(state, true,
                               AddExpr::create(arguments[0], 
                                               ConstantExpr::create(16, 64)),
                               ConstantExpr::create(0, 64), 0, "vastart 4, instCtr: " + std::to_string(instCtr)); // reg_save_area
      }
      break;
    }
    case Intrinsic::vaend:
      // va_end is a noop for the interpreter.
      //
      // FIXME: We should validate that the target didn't do something bad
      // with va_end, however (like call it twice).
      break;
        
    case Intrinsic::vacopy:
      // va_copy should have been lowered.
      //
      // FIXME: It would be nice to check for errors in the usage of this as
      // well.
    default:
      klee_error("unknown intrinsic: %s", f->getName().data());
    }

    if (InvokeInst *ii = dyn_cast<InvokeInst>(i))
      transferToBasicBlock(ii->getNormalDest(), i->getParent(), state);
  } else {
    // FIXME: I'm not really happy about this reliance on prevPC but it is ok, I
    // guess. This just done to avoid having to pass KInstIterator everywhere
    // instead of the actual instruction, since we can't make a KInstIterator
    // from just an instruction (unlike LLVM).
    KFunction *kf = kmodule->functionMap[f];
    state.pushFrame(state.prevPC, kf);
    state.pc = kf->instructions;

    if (statsTracker)
      statsTracker->framePushed(state, &state.stack[state.stack.size()-2]);

     // TODO: support "byval" parameter attribute
     // TODO: support zeroext, signext, sret attributes

    unsigned callingArgs = arguments.size();
    unsigned funcArgs = f->arg_size();
    if (!f->isVarArg()) {
      if (callingArgs > funcArgs) {
        klee_warning_once(f, "calling %s with extra arguments.", 
                          f->getName().data());
      } else if (callingArgs < funcArgs) {
        terminateStateOnError(state, "calling function with too few arguments",
                              User);
        return;
      }
    } else {
      printf("setting up varargs\n");
      fflush(stdout);
      Expr::Width WordSize = Context::get().getPointerWidth();

      if (callingArgs < funcArgs) {
        terminateStateOnError(state, "calling function with too few arguments",
                              User);
        return;
      }

      StackFrame &sf = state.stack.back();
      unsigned size = 0;
      bool requires16ByteAlignment = false;
      for (unsigned i = funcArgs; i < callingArgs; i++) {
        // FIXME: This is really specific to the architecture, not the pointer
        // size. This happens to work for x86-32 and x86-64, however.
        if (WordSize == Expr::Int32) {
          size += Expr::getMinBytesForWidth(arguments[i]->getWidth());
        } else {
          Expr::Width argWidth = arguments[i]->getWidth();
          // AMD64-ABI 3.5.7p5: Step 7. Align l->overflow_arg_area upwards to a
          // 16 byte boundary if alignment needed by type exceeds 8 byte
          // boundary.
          //
          // Alignment requirements for scalar types is the same as their size
          if (argWidth > Expr::Int64) {
             size = llvm::RoundUpToAlignment(size, 16);
             requires16ByteAlignment = true;
          }
          size += llvm::RoundUpToAlignment(argWidth, WordSize) / 8;
        }
      }

      MemoryObject *mo = sf.varargs =
          memory->allocate(size, true, false, state.prevPC->inst,
                           (requires16ByteAlignment ? 16 : 8));
      if (!mo && size) {
        terminateStateOnExecError(state, "out of memory (varargs)");
        return;
      }

      if (mo) {
        if ((WordSize == Expr::Int64) && (mo->address & 15) &&
            requires16ByteAlignment) {
          // Both 64bit Linux/Glibc and 64bit MacOSX should align to 16 bytes.
          klee_warning_once(
              0, "While allocating varargs: malloc did not align to 16 bytes.");
        }

        ObjectState *os = bindObjectInState(state, mo, true);
        unsigned offset = 0;
        for (unsigned i = funcArgs; i < callingArgs; i++) {
          // FIXME: This is really specific to the architecture, not the pointer
          // size. This happens to work for x86-32 and x86-64, however.
          if (WordSize == Expr::Int32) {
            os->write(offset, arguments[i]);
            offset += Expr::getMinBytesForWidth(arguments[i]->getWidth());
          } else {
            assert(WordSize == Expr::Int64 && "Unknown word size!");

            Expr::Width argWidth = arguments[i]->getWidth();
            if (argWidth > Expr::Int64) {
              offset = llvm::RoundUpToAlignment(offset, 16);
            }
            os->write(offset, arguments[i]);
            offset += llvm::RoundUpToAlignment(argWidth, WordSize) / 8;
          }
        }
      }
    }

    unsigned numFormals = f->arg_size();
    for (unsigned i=0; i<numFormals; ++i) 
      bindArgument(kf, i, state, arguments[i]);
  }
}

void Executor::transferToBasicBlock(BasicBlock *dst, BasicBlock *src, 
                                    ExecutionState &state) {
  // Note that in general phi nodes can reuse phi values from the same
  // block but the incoming value is the eval() result *before* the
  // execution of any phi nodes. this is pathological and doesn't
  // really seem to occur, but just in case we run the PhiCleanerPass
  // which makes sure this cannot happen and so it is safe to just
  // eval things in order. The PhiCleanerPass also makes sure that all
  // incoming blocks have the same order for each PHINode so we only
  // have to compute the index once.
  //
  // With that done we simply set an index in the state so that PHI
  // instructions know which argument to eval, set the pc, and continue.
  
  // XXX this lookup has to go ?
  KFunction *kf = state.stack.back().kf;
  unsigned entry = kf->basicBlockEntry[dst];
  state.pc = &kf->instructions[entry];


  if (state.pc->inst->getOpcode() == Instruction::PHI) {
    PHINode *first = static_cast<PHINode*>(state.pc->inst);
    state.incomingBBIndex = first->getBasicBlockIndex(src);
  }
}

/// Compute the true target of a function call, resolving LLVM and KLEE aliases
/// and bitcasts.
Function* Executor::getTargetFunction(Value *calledVal, ExecutionState &state) {
  SmallPtrSet<const GlobalValue*, 3> Visited;

  Constant *c = dyn_cast<Constant>(calledVal);
  if (!c)
    return 0;

  while (true) {
    if (GlobalValue *gv = dyn_cast<GlobalValue>(c)) {
#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 6)
      if (!Visited.insert(gv).second)
        return 0;
#else
      if (!Visited.insert(gv))
        return 0;
#endif
      std::string alias = state.getFnAlias(gv->getName());
      if (alias != "") {
        llvm::Module* currModule = kmodule->module;
        GlobalValue *old_gv = gv;
        gv = currModule->getNamedValue(alias);
        if (!gv) {
          klee_error("Function %s(), alias for %s not found!\n", alias.c_str(),
                     old_gv->getName().str().c_str());
        }
      }
     
      if (Function *f = dyn_cast<Function>(gv))
        return f;
      else if (GlobalAlias *ga = dyn_cast<GlobalAlias>(gv))
        c = ga->getAliasee();
      else
        return 0;
    } else if (llvm::ConstantExpr *ce = dyn_cast<llvm::ConstantExpr>(c)) {
      if (ce->getOpcode()==Instruction::BitCast)
        c = ce->getOperand(0);
      else
        return 0;
    } else
      return 0;
  }
}

/// TODO remove?
static bool isDebugIntrinsic(const Function *f, KModule *KM) {
  return false;
}

static inline const llvm::fltSemantics * fpWidthToSemantics(unsigned width) {
  switch(width) {
  case Expr::Int32:
    return &llvm::APFloat::IEEEsingle;
  case Expr::Int64:
    return &llvm::APFloat::IEEEdouble;
  case Expr::Fl80:
    return &llvm::APFloat::x87DoubleExtended;
  default:
    return 0;
  }
}

void Executor::executeInstruction(ExecutionState &state, KInstruction *ki) {

  instCtr++;
  Instruction *i = ki->inst;
  switch (i->getOpcode()) {
    // Control flow
  case Instruction::Ret: {
    ReturnInst *ri = cast<ReturnInst>(i);
    KInstIterator kcaller = state.stack.back().caller;
    Instruction *caller = kcaller ? kcaller->inst : 0;
    bool isVoidReturn = (ri->getNumOperands() == 0);
    ref<Expr> result = ConstantExpr::alloc(0, Expr::Bool);
    
    if (!isVoidReturn) {
      result = eval(ki, 0, state).value;
    }
    
    if (state.stack.size() <= 1) {
      assert(!caller && "caller set on initial stack frame");
      //printf("Choosing not to call terminateStateOnExit(state) \n \n ");
      //terminateStateOnExit(state);
      state.popFrame();
      
      haltExecution = true;
      break;
    } else {
      state.popFrame();

      if (statsTracker)
        statsTracker->framePopped(state);
      if (InvokeInst *ii = dyn_cast<InvokeInst>(caller)) {
        transferToBasicBlock(ii->getNormalDest(), caller->getParent(), state);
      } else {
        state.pc = kcaller;
        ++state.pc;
      }

      if (!isVoidReturn) {
        Type *t = caller->getType();
        if (t != Type::getVoidTy(i->getContext())) {
          // may need to do coercion due to bitcasts
          Expr::Width from = result->getWidth();
          Expr::Width to = getWidthForLLVMType(t);
            
          if (from != to) {
            CallSite cs = (isa<InvokeInst>(caller) ? CallSite(cast<InvokeInst>(caller)) : 
                           CallSite(cast<CallInst>(caller)));

            // XXX need to check other param attrs ?
      bool isSExt = cs.paramHasAttr(0, llvm::Attribute::SExt);
            if (isSExt) {
              result = SExtExpr::create(result, to);
            } else {
              result = ZExtExpr::create(result, to);
            }
          }

          bindLocal(kcaller, state, result);
        }
      } else {
        // We check that the return value has no users instead of
        // checking the type, since C defaults to returning int for
        // undeclared functions.
        if (!caller->use_empty()) {
          terminateStateOnExecError(state, "return void when caller expected a result");
        }
      }
    }
    
    break;
  }
  case Instruction::Br: {

    //printf("Hit br inst \n");
    BranchInst *bi = cast<BranchInst>(i);
    if (bi->isUnconditional()) {
      transferToBasicBlock(bi->getSuccessor(0), bi->getParent(), state);
    } else {
      // FIXME: Find a way that we don't have this hidden dependency.
      assert(bi->getCondition() == bi->getOperand(0) &&
             "Wrong operand index!");
      ref<Expr> cond = eval(ki, 0, state).value;

      //ABH: Within TASE, we're unix forking within Executor::fork
      // when a branch instruction depends on symbolic data.  Currently
      // only ever returning 1 state in "branches" becuase of this.
      Executor::StatePair branches = fork(state, cond, false);

      // NOTE: There is a hidden dependency here, markBranchVisited
      // requires that we still be in the context of the branch
      // instruction (it reuses its statistic id). Should be cleaned
      // up with convenient instruction specific data.
      if (statsTracker && state.stack.back().kf->trackCoverage)
        statsTracker->markBranchVisited(branches.first, branches.second);

      if (branches.second) {
        transferToBasicBlock(bi->getSuccessor(1), bi->getParent(), *branches.second);	
      }
      if (branches.first) {
        transferToBasicBlock(bi->getSuccessor(0), bi->getParent(), *branches.first);
      }
            
    }
    break;
  }
  case Instruction::Switch: {
    SwitchInst *si = cast<SwitchInst>(i);
    ref<Expr> cond = eval(ki, 0, state).value;
    BasicBlock *bb = si->getParent();

    cond = toUnique(state, cond);
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(cond)) {
      // Somewhat gross to create these all the time, but fine till we
      // switch to an internal rep.
      llvm::IntegerType *Ty = cast<IntegerType>(si->getCondition()->getType());
      ConstantInt *ci = ConstantInt::get(Ty, CE->getZExtValue());
      unsigned index = si->findCaseValue(ci).getSuccessorIndex();
      transferToBasicBlock(si->getSuccessor(index), si->getParent(), state);
    } else {
      // Handle possible different branch targets

      // We have the following assumptions:
      // - each case value is mutual exclusive to all other values including the
      //   default value
      // - order of case branches is based on the order of the expressions of
      //   the scase values, still default is handled last
      std::vector<BasicBlock *> bbOrder;
      std::map<BasicBlock *, ref<Expr> > branchTargets;

      std::map<ref<Expr>, BasicBlock *> expressionOrder;

      // Iterate through all non-default cases and order them by expressions
      for (SwitchInst::CaseIt i = si->case_begin(), e = si->case_end(); i != e;
           ++i) {
        ref<Expr> value = evalConstant(i.getCaseValue());

        BasicBlock *caseSuccessor = i.getCaseSuccessor();
        expressionOrder.insert(std::make_pair(value, caseSuccessor));
      }

      // Track default branch values
      ref<Expr> defaultValue = ConstantExpr::alloc(1, Expr::Bool);

      // iterate through all non-default cases but in order of the expressions
      for (std::map<ref<Expr>, BasicBlock *>::iterator
               it = expressionOrder.begin(),
               itE = expressionOrder.end();
           it != itE; ++it) {
        ref<Expr> match = EqExpr::create(cond, it->first);

        // Make sure that the default value does not contain this target's value
        defaultValue = AndExpr::create(defaultValue, Expr::createIsZero(match));

        // Check if control flow could take this case
        bool result;
        bool success = solver->mayBeTrue(state, match, result);
        assert(success && "FIXME: Unhandled solver failure");
        (void) success;
        if (result) {
          BasicBlock *caseSuccessor = it->second;

          // Handle the case that a basic block might be the target of multiple
          // switch cases.
          // Currently we generate an expression containing all switch-case
          // values for the same target basic block. We spare us forking too
          // many times but we generate more complex condition expressions
          // TODO Add option to allow to choose between those behaviors
          std::pair<std::map<BasicBlock *, ref<Expr> >::iterator, bool> res =
              branchTargets.insert(std::make_pair(
                  caseSuccessor, ConstantExpr::alloc(0, Expr::Bool)));

          res.first->second = OrExpr::create(match, res.first->second);

          // Only add basic blocks which have not been target of a branch yet
          if (res.second) {
            bbOrder.push_back(caseSuccessor);
          }
        }
      }

      // Check if control could take the default case
      bool res;
      bool success = solver->mayBeTrue(state, defaultValue, res);
      assert(success && "FIXME: Unhandled solver failure");
      (void) success;
      if (res) {
        std::pair<std::map<BasicBlock *, ref<Expr> >::iterator, bool> ret =
            branchTargets.insert(
                std::make_pair(si->getDefaultDest(), defaultValue));
        if (ret.second) {
          bbOrder.push_back(si->getDefaultDest());
        }
      }

      // Fork the current state with each state having one of the possible
      // successors of this switch
      std::vector< ref<Expr> > conditions;
      for (std::vector<BasicBlock *>::iterator it = bbOrder.begin(),
                                               ie = bbOrder.end();
           it != ie; ++it) {
        conditions.push_back(branchTargets[*it]);
      }
      std::vector<ExecutionState*> branches;
      branch(state, conditions, branches);

      std::vector<ExecutionState*>::iterator bit = branches.begin();
      for (std::vector<BasicBlock *>::iterator it = bbOrder.begin(),
                                               ie = bbOrder.end();
           it != ie; ++it) {
        ExecutionState *es = *bit;
        if (es)
          transferToBasicBlock(*it, bb, *es);
        ++bit;
      }
    }
    break;
 }
  case Instruction::Unreachable:
    // Note that this is not necessarily an internal bug, llvm will
    // generate unreachable instructions in cases where it knows the
    // program will crash. So it is effectively a SEGV or internal
    // error.
    terminateStateOnExecError(state, "reached \"unreachable\" instruction");
    break;

  case Instruction::Invoke:
  case Instruction::Call: {
    CallSite cs(i);

    unsigned numArgs = cs.arg_size();
    Value *fp = cs.getCalledValue();
    Function *f = getTargetFunction(fp, state);

    // Skip debug intrinsics, we can't evaluate their metadata arguments.
    if (f && isDebugIntrinsic(f, kmodule))
      break;

    if (isa<InlineAsm>(fp)) {
      terminateStateOnExecError(state, "inline assembly is unsupported");
      break;
    }
    // evaluate arguments
    std::vector< ref<Expr> > arguments;
    arguments.reserve(numArgs);

    for (unsigned j=0; j<numArgs; ++j)
      arguments.push_back(eval(ki, j+1, state).value);

    if (f) {
      const FunctionType *fType = 
        dyn_cast<FunctionType>(cast<PointerType>(f->getType())->getElementType());
      const FunctionType *fpType =
        dyn_cast<FunctionType>(cast<PointerType>(fp->getType())->getElementType());

      // special case the call with a bitcast case
      if (fType != fpType) {
        assert(fType && fpType && "unable to get function type");

        // XXX check result coercion

        // XXX this really needs thought and validation
        unsigned i=0;
        for (std::vector< ref<Expr> >::iterator
               ai = arguments.begin(), ie = arguments.end();
             ai != ie; ++ai) {
          Expr::Width to, from = (*ai)->getWidth();
            
          if (i<fType->getNumParams()) {
            to = getWidthForLLVMType(fType->getParamType(i));

            if (from != to) {
              // XXX need to check other param attrs ?
              bool isSExt = cs.paramHasAttr(i+1, llvm::Attribute::SExt);
              if (isSExt) {
                arguments[i] = SExtExpr::create(arguments[i], to);
              } else {
                arguments[i] = ZExtExpr::create(arguments[i], to);
              }
            }
          }
            
          i++;
        }
      }

      executeCall(state, ki, f, arguments);
    } else {
      ref<Expr> v = eval(ki, 0, state).value;

      ExecutionState *free = &state;
      bool hasInvalid = false, first = true;

      /* XXX This is wasteful, no need to do a full evaluate since we
         have already got a value. But in the end the caches should
         handle it for us, albeit with some overhead. */
      do {
        ref<ConstantExpr> value;
        bool success = solver->getValue(*free, v, value);
        assert(success && "FIXME: Unhandled solver failure");
        (void) success;
        StatePair res = fork(*free, EqExpr::create(v, value), true);
        if (res.first) {
          uint64_t addr = value->getZExtValue();
          if (legalFunctions.count(addr)) {
            f = (Function*) addr;

            // Don't give warning on unique resolution
            if (res.second || !first)
              klee_warning_once((void*) (unsigned long) addr, 
                                "resolved symbolic function pointer to: %s",
                                f->getName().data());

            executeCall(*res.first, ki, f, arguments);
          } else {
            if (!hasInvalid) {
              terminateStateOnExecError(state, "invalid function pointer");
              hasInvalid = true;
            }
          }
        }

        first = false;
        free = res.second;
      } while (free);
    }
    break;
  }
  case Instruction::PHI: {
    ref<Expr> result = eval(ki, state.incomingBBIndex, state).value;
    bindLocal(ki, state, result);
    break;
  }

    // Special instructions
  case Instruction::Select: {
    // NOTE: It is not required that operands 1 and 2 be of scalar type.
    ref<Expr> cond = eval(ki, 0, state).value;
    ref<Expr> tExpr = eval(ki, 1, state).value;
    ref<Expr> fExpr = eval(ki, 2, state).value;
    ref<Expr> result = SelectExpr::create(cond, tExpr, fExpr);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::VAArg:
    terminateStateOnExecError(state, "unexpected VAArg instruction");
    break;

    // Arithmetic / logical

  case Instruction::Add: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    bindLocal(ki, state, AddExpr::create(left, right));
    break;
  }

  case Instruction::Sub: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    bindLocal(ki, state, SubExpr::create(left, right));
    break;
  }
 
  case Instruction::Mul: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    bindLocal(ki, state, MulExpr::create(left, right));
    break;
  }

  case Instruction::UDiv: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = UDivExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::SDiv: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = SDivExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::URem: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = URemExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::SRem: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = SRemExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::And: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = AndExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::Or: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = OrExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::Xor: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = XorExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::Shl: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = ShlExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::LShr: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = LShrExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::AShr: {
    ref<Expr> left = eval(ki, 0, state).value;
    ref<Expr> right = eval(ki, 1, state).value;
    ref<Expr> result = AShrExpr::create(left, right);
    bindLocal(ki, state, result);
    break;
  }

    // Compare

  case Instruction::ICmp: {
    CmpInst *ci = cast<CmpInst>(i);
    ICmpInst *ii = cast<ICmpInst>(ci);

    switch(ii->getPredicate()) {
    case ICmpInst::ICMP_EQ: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = EqExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_NE: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = NeExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_UGT: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = UgtExpr::create(left, right);
      bindLocal(ki, state,result);
      break;
    }

    case ICmpInst::ICMP_UGE: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = UgeExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_ULT: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = UltExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_ULE: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = UleExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_SGT: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = SgtExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_SGE: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = SgeExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_SLT: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = SltExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    case ICmpInst::ICMP_SLE: {
      ref<Expr> left = eval(ki, 0, state).value;
      ref<Expr> right = eval(ki, 1, state).value;
      ref<Expr> result = SleExpr::create(left, right);
      bindLocal(ki, state, result);
      break;
    }

    default:
      terminateStateOnExecError(state, "invalid ICmp predicate");
    }
    break;
  }
 
    // Memory instructions...
  case Instruction::Alloca: {
    AllocaInst *ai = cast<AllocaInst>(i);
    unsigned elementSize = 
      kmodule->targetData->getTypeStoreSize(ai->getAllocatedType());
    ref<Expr> size = Expr::createPointer(elementSize);
    if (ai->isArrayAllocation()) {
      ref<Expr> count = eval(ki, 0, state).value;
      count = Expr::createZExtToPointerWidth(count);
      size = MulExpr::create(size, count);
    }
    executeAlloc(state, size, true, ki);
    break;
  }

  case Instruction::Load: {
    double T0;
    if (measureTime)
      T0= util::getWallTime();
    ref<Expr> base = eval(ki, 0, state).value;
    if (measureTime)
      mem_op_eval_time += util::getWallTime() - T0;
    executeMemoryOperation(state, false, base, 0, ki, "load 0, instCtr: " + std::to_string(instCtr));
    if (measureTime)
      run_mem_op_time += (util::getWallTime() - T0);
    break;
  }
  case Instruction::Store: {
    double T0;
    if (measureTime)
      T0 = util::getWallTime();
    ref<Expr> base = eval(ki, 1, state).value;
    ref<Expr> value = eval(ki, 0, state).value;
    if (measureTime) 
      mem_op_eval_time += util::getWallTime() - T0;
    executeMemoryOperation(state, true, base, value, 0, "store 0, instCtr: " + std::to_string(instCtr));
    if (measureTime)
      run_mem_op_time += (util::getWallTime() - T0);
    break;
  }

  case Instruction::GetElementPtr: {
    KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(ki);
    ref<Expr> base = eval(ki, 0, state).value;

    for (std::vector< std::pair<unsigned, uint64_t> >::iterator 
           it = kgepi->indices.begin(), ie = kgepi->indices.end(); 
         it != ie; ++it) {
      uint64_t elementSize = it->second;
      ref<Expr> index = eval(ki, it->first, state).value;
      base = AddExpr::create(base,
                             MulExpr::create(Expr::createSExtToPointerWidth(index),
                                             Expr::createPointer(elementSize)));
    }
    if (kgepi->offset)
      base = AddExpr::create(base,
                             Expr::createPointer(kgepi->offset));
    bindLocal(ki, state, base);
    break;
  }

    // Conversion
  case Instruction::Trunc: {
    CastInst *ci = cast<CastInst>(i);
    ref<Expr> result = ExtractExpr::create(eval(ki, 0, state).value,
                                           0,
                                           getWidthForLLVMType(ci->getType()));
    bindLocal(ki, state, result);
    break;
  }
  case Instruction::ZExt: {
    CastInst *ci = cast<CastInst>(i);
    ref<Expr> result = ZExtExpr::create(eval(ki, 0, state).value,
                                        getWidthForLLVMType(ci->getType()));
    bindLocal(ki, state, result);
    break;
  }
  case Instruction::SExt: {
    CastInst *ci = cast<CastInst>(i);
    ref<Expr> result = SExtExpr::create(eval(ki, 0, state).value,
                                        getWidthForLLVMType(ci->getType()));
    bindLocal(ki, state, result);
    break;
  }

  case Instruction::IntToPtr: {
    CastInst *ci = cast<CastInst>(i);
    Expr::Width pType = getWidthForLLVMType(ci->getType());
    ref<Expr> arg = eval(ki, 0, state).value;
    bindLocal(ki, state, ZExtExpr::create(arg, pType));
    break;
  }
  case Instruction::PtrToInt: {
    CastInst *ci = cast<CastInst>(i);
    Expr::Width iType = getWidthForLLVMType(ci->getType());
    ref<Expr> arg = eval(ki, 0, state).value;
    bindLocal(ki, state, ZExtExpr::create(arg, iType));
    break;
  }

  case Instruction::BitCast: {
    ref<Expr> result = eval(ki, 0, state).value;
    bindLocal(ki, state, result);
    break;
  }

    // Floating point instructions

  case Instruction::FAdd: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state).value,
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FAdd operation");

    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.add(APFloat(*fpWidthToSemantics(right->getWidth()),right->getAPValue()), APFloat::rmNearestTiesToEven);
    bindLocal(ki, state, ConstantExpr::alloc(Res.bitcastToAPInt()));
    break;
  }

  case Instruction::FSub: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state).value,
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FSub operation");
    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.subtract(APFloat(*fpWidthToSemantics(right->getWidth()), right->getAPValue()), APFloat::rmNearestTiesToEven);
    bindLocal(ki, state, ConstantExpr::alloc(Res.bitcastToAPInt()));
    break;
  }

  case Instruction::FMul: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state).value,
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FMul operation");

    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.multiply(APFloat(*fpWidthToSemantics(right->getWidth()), right->getAPValue()), APFloat::rmNearestTiesToEven);
    bindLocal(ki, state, ConstantExpr::alloc(Res.bitcastToAPInt()));
    break;
  }

  case Instruction::FDiv: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state).value,
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FDiv operation");

    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.divide(APFloat(*fpWidthToSemantics(right->getWidth()), right->getAPValue()), APFloat::rmNearestTiesToEven);
    bindLocal(ki, state, ConstantExpr::alloc(Res.bitcastToAPInt()));
    break;
  }

  case Instruction::FRem: {
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state).value,
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FRem operation");
    llvm::APFloat Res(*fpWidthToSemantics(left->getWidth()), left->getAPValue());
    Res.mod(APFloat(*fpWidthToSemantics(right->getWidth()),right->getAPValue()),
            APFloat::rmNearestTiesToEven);
    bindLocal(ki, state, ConstantExpr::alloc(Res.bitcastToAPInt()));
    break;
  }

  case Instruction::FPTrunc: {
    FPTruncInst *fi = cast<FPTruncInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state).value,
                                       "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || resultType > arg->getWidth())
      return terminateStateOnExecError(state, "Unsupported FPTrunc operation");

    llvm::APFloat Res(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    bool losesInfo = false;
    Res.convert(*fpWidthToSemantics(resultType),
                llvm::APFloat::rmNearestTiesToEven,
                &losesInfo);
    bindLocal(ki, state, ConstantExpr::alloc(Res));
    break;
  }

  case Instruction::FPExt: {
    FPExtInst *fi = cast<FPExtInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || arg->getWidth() > resultType)
      return terminateStateOnExecError(state, "Unsupported FPExt operation");
    llvm::APFloat Res(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    bool losesInfo = false;
    Res.convert(*fpWidthToSemantics(resultType),
                llvm::APFloat::rmNearestTiesToEven,
                &losesInfo);
    bindLocal(ki, state, ConstantExpr::alloc(Res));
    break;
  }

  case Instruction::FPToUI: {
    FPToUIInst *fi = cast<FPToUIInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state).value,
                                       "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || resultType > 64)
      return terminateStateOnExecError(state, "Unsupported FPToUI operation");

    llvm::APFloat Arg(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());
    uint64_t value = 0;
    bool isExact = true;
    Arg.convertToInteger(&value, resultType, false,
                         llvm::APFloat::rmTowardZero, &isExact);
    bindLocal(ki, state, ConstantExpr::alloc(value, resultType));
    break;
  }

  case Instruction::FPToSI: {
    FPToSIInst *fi = cast<FPToSIInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state).value,
                                       "floating point");
    if (!fpWidthToSemantics(arg->getWidth()) || resultType > 64)
      return terminateStateOnExecError(state, "Unsupported FPToSI operation");
    llvm::APFloat Arg(*fpWidthToSemantics(arg->getWidth()), arg->getAPValue());

    uint64_t value = 0;
    bool isExact = true;
    Arg.convertToInteger(&value, resultType, true,
                         llvm::APFloat::rmTowardZero, &isExact);
    bindLocal(ki, state, ConstantExpr::alloc(value, resultType));
    break;
  }

  case Instruction::UIToFP: {
    UIToFPInst *fi = cast<UIToFPInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state).value,
                                       "floating point");
    const llvm::fltSemantics *semantics = fpWidthToSemantics(resultType);
    if (!semantics)
      return terminateStateOnExecError(state, "Unsupported UIToFP operation");
    llvm::APFloat f(*semantics, 0);
    f.convertFromAPInt(arg->getAPValue(), false,
                       llvm::APFloat::rmNearestTiesToEven);

    bindLocal(ki, state, ConstantExpr::alloc(f));
    break;
  }

  case Instruction::SIToFP: {
    SIToFPInst *fi = cast<SIToFPInst>(i);
    Expr::Width resultType = getWidthForLLVMType(fi->getType());
    ref<ConstantExpr> arg = toConstant(state, eval(ki, 0, state).value,
                                       "floating point");
    const llvm::fltSemantics *semantics = fpWidthToSemantics(resultType);
    if (!semantics)
      return terminateStateOnExecError(state, "Unsupported SIToFP operation");
    llvm::APFloat f(*semantics, 0);
    f.convertFromAPInt(arg->getAPValue(), true,
                       llvm::APFloat::rmNearestTiesToEven);

    bindLocal(ki, state, ConstantExpr::alloc(f));
    break;
  }

  case Instruction::FCmp: {
    FCmpInst *fi = cast<FCmpInst>(i);
    ref<ConstantExpr> left = toConstant(state, eval(ki, 0, state).value,
                                        "floating point");
    ref<ConstantExpr> right = toConstant(state, eval(ki, 1, state).value,
                                         "floating point");
    if (!fpWidthToSemantics(left->getWidth()) ||
        !fpWidthToSemantics(right->getWidth()))
      return terminateStateOnExecError(state, "Unsupported FCmp operation");

    APFloat LHS(*fpWidthToSemantics(left->getWidth()),left->getAPValue());
    APFloat RHS(*fpWidthToSemantics(right->getWidth()),right->getAPValue());
    APFloat::cmpResult CmpRes = LHS.compare(RHS);

    bool Result = false;
    switch( fi->getPredicate() ) {
      // Predicates which only care about whether or not the operands are NaNs.
    case FCmpInst::FCMP_ORD:
      Result = CmpRes != APFloat::cmpUnordered;
      break;

    case FCmpInst::FCMP_UNO:
      Result = CmpRes == APFloat::cmpUnordered;
      break;

      // Ordered comparisons return false if either operand is NaN.  Unordered
      // comparisons return true if either operand is NaN.
    case FCmpInst::FCMP_UEQ:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
    case FCmpInst::FCMP_OEQ:
      Result = CmpRes == APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_UGT:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
    case FCmpInst::FCMP_OGT:
      Result = CmpRes == APFloat::cmpGreaterThan;
      break;

    case FCmpInst::FCMP_UGE:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
    case FCmpInst::FCMP_OGE:
      Result = CmpRes == APFloat::cmpGreaterThan || CmpRes == APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_ULT:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
    case FCmpInst::FCMP_OLT:
      Result = CmpRes == APFloat::cmpLessThan;
      break;

    case FCmpInst::FCMP_ULE:
      if (CmpRes == APFloat::cmpUnordered) {
        Result = true;
        break;
      }
    case FCmpInst::FCMP_OLE:
      Result = CmpRes == APFloat::cmpLessThan || CmpRes == APFloat::cmpEqual;
      break;

    case FCmpInst::FCMP_UNE:
      Result = CmpRes == APFloat::cmpUnordered || CmpRes != APFloat::cmpEqual;
      break;
    case FCmpInst::FCMP_ONE:
      Result = CmpRes != APFloat::cmpUnordered && CmpRes != APFloat::cmpEqual;
      break;

    default:
      assert(0 && "Invalid FCMP predicate!");
    case FCmpInst::FCMP_FALSE:
      Result = false;
      break;
    case FCmpInst::FCMP_TRUE:
      Result = true;
      break;
    }

    bindLocal(ki, state, ConstantExpr::alloc(Result, Expr::Bool));
    break;
  }
  case Instruction::InsertValue: {
    KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(ki);

    ref<Expr> agg = eval(ki, 0, state).value;
    ref<Expr> val = eval(ki, 1, state).value;

    ref<Expr> l = NULL, r = NULL;
    unsigned lOffset = kgepi->offset*8, rOffset = kgepi->offset*8 + val->getWidth();

    if (lOffset > 0)
      l = ExtractExpr::create(agg, 0, lOffset);
    if (rOffset < agg->getWidth())
      r = ExtractExpr::create(agg, rOffset, agg->getWidth() - rOffset);

    ref<Expr> result;
    if (!l.isNull() && !r.isNull())
      result = ConcatExpr::create(r, ConcatExpr::create(val, l));
    else if (!l.isNull())
      result = ConcatExpr::create(val, l);
    else if (!r.isNull())
      result = ConcatExpr::create(r, val);
    else
      result = val;

    bindLocal(ki, state, result);
    break;
  }
  case Instruction::ExtractValue: {
    KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(ki);

    ref<Expr> agg = eval(ki, 0, state).value;

    ref<Expr> result = ExtractExpr::create(agg, kgepi->offset*8, getWidthForLLVMType(i->getType()));

    bindLocal(ki, state, result);
    break;
  }
  case Instruction::Fence: {
    // Ignore for now
    break;
  }
  case Instruction::InsertElement: {
    InsertElementInst *iei = cast<InsertElementInst>(i);
    ref<Expr> vec = eval(ki, 0, state).value;
    ref<Expr> newElt = eval(ki, 1, state).value;
    ref<Expr> idx = eval(ki, 2, state).value;
    
    ConstantExpr *cIdx = dyn_cast<ConstantExpr>(idx);
    if (cIdx == NULL) {
      terminateStateOnError(
          state, "InsertElement, support for symbolic index not implemented",
          Unhandled);
      return;
    }
    uint64_t iIdx = cIdx->getZExtValue();
    const llvm::VectorType *vt = iei->getType();
    unsigned EltBits = getWidthForLLVMType(vt->getElementType());

    //printf("\n Calling InsertElement at idx %lu \n", iIdx);

    if (iIdx >= vt->getNumElements()) {
      // Out of bounds write
      terminateStateOnError(state, "Out of bounds write when inserting element",
                            BadVectorAccess);
      return;
    }

    const unsigned elementCount = vt->getNumElements();
    llvm::SmallVector<ref<Expr>, 8> elems;
    elems.reserve(elementCount);
    for (unsigned i = 0; i < elementCount; ++i) {
      // evalConstant() will use ConcatExpr to build vectors with the
      // zero-th element leftmost (most significant bits), followed
      // by the next element (second leftmost) and so on. This means
      // that we have to adjust the index so we read left to right
      // rather than right to left.
      unsigned bitOffset = EltBits * (elementCount - i - 1);
      //printf("bitOffset is %u \n\n",bitOffset);
      if (i == iIdx) {
	//printf("Found insert index at %u \n\n", bitOffset);
      }
      elems.push_back(i == iIdx ? newElt
                                : ExtractExpr::create(vec, bitOffset, EltBits));
    }

    ref<Expr> Result = ConcatExpr::createN(elementCount, elems.data());
    bindLocal(ki, state, Result);
    break;
  }
  case Instruction::ExtractElement: {
    ExtractElementInst *eei = cast<ExtractElementInst>(i);
    ref<Expr> vec = eval(ki, 0, state).value;
    ref<Expr> idx = eval(ki, 1, state).value;

    ConstantExpr *cIdx = dyn_cast<ConstantExpr>(idx);
    if (cIdx == NULL) {
      terminateStateOnError(
          state, "ExtractElement, support for symbolic index not implemented",
          Unhandled);
      return;
    }
    uint64_t iIdx = cIdx->getZExtValue();
    const llvm::VectorType *vt = eei->getVectorOperandType();
    unsigned EltBits = getWidthForLLVMType(vt->getElementType());

    if (iIdx >= vt->getNumElements()) {
      // Out of bounds read
      terminateStateOnError(state, "Out of bounds read when extracting element",
                            BadVectorAccess);
      return;
    }

    // evalConstant() will use ConcatExpr to build vectors with the
    // zero-th element left most (most significant bits), followed
    // by the next element (second left most) and so on. This means
    // that we have to adjust the index so we read left to right
    // rather than right to left.
    unsigned bitOffset = EltBits*(vt->getNumElements() - iIdx -1);
    ref<Expr> Result = ExtractExpr::create(vec, bitOffset, EltBits);
    bindLocal(ki, state, Result);
    break;
  }
  case Instruction::ShuffleVector:
    // Should never happen due to Scalarizer pass removing ShuffleVector
    // instructions.
    terminateStateOnExecError(state, "Unexpected ShuffleVector instruction");
    break;
  // Other instructions...
  // Unhandled
  default:
    terminateStateOnExecError(state, "illegal instruction");
    break;
  }
}

void Executor::updateStates(ExecutionState *current) {
  if (searcher) {
    searcher->update(current, addedStates, removedStates);
    searcher->update(nullptr, continuedStates, pausedStates);
    pausedStates.clear();
    continuedStates.clear();
  }
  
  states.insert(addedStates.begin(), addedStates.end());
  addedStates.clear();

  for (std::vector<ExecutionState *>::iterator it = removedStates.begin(),
                                               ie = removedStates.end();
       it != ie; ++it) {
    ExecutionState *es = *it;
    std::set<ExecutionState*>::iterator it2 = states.find(es);
    assert(it2!=states.end());
    states.erase(it2);
    std::map<ExecutionState*, std::vector<SeedInfo> >::iterator it3 = 
      seedMap.find(es);
    if (it3 != seedMap.end())
      seedMap.erase(it3);
    processTree->remove(es->ptreeNode);
    delete es;
  }
  removedStates.clear();
}

template <typename TypeIt>
void Executor::computeOffsets(KGEPInstruction *kgepi, TypeIt ib, TypeIt ie) {
  ref<ConstantExpr> constantOffset =
    ConstantExpr::alloc(0, Context::get().getPointerWidth());
  uint64_t index = 1;
  for (TypeIt ii = ib; ii != ie; ++ii) {
    if (StructType *st = dyn_cast<StructType>(*ii)) {
      const StructLayout *sl = kmodule->targetData->getStructLayout(st);
      const ConstantInt *ci = cast<ConstantInt>(ii.getOperand());
      uint64_t addend = sl->getElementOffset((unsigned) ci->getZExtValue());
      constantOffset = constantOffset->Add(ConstantExpr::alloc(addend,
                                                               Context::get().getPointerWidth()));
    } else {
      const SequentialType *set = cast<SequentialType>(*ii);
      uint64_t elementSize = 
        kmodule->targetData->getTypeStoreSize(set->getElementType());
      Value *operand = ii.getOperand();
      if (Constant *c = dyn_cast<Constant>(operand)) {
        ref<ConstantExpr> index = 
          evalConstant(c)->SExt(Context::get().getPointerWidth());
        ref<ConstantExpr> addend = 
          index->Mul(ConstantExpr::alloc(elementSize,
                                         Context::get().getPointerWidth()));
        constantOffset = constantOffset->Add(addend);
      } else {
        kgepi->indices.push_back(std::make_pair(index, elementSize));
      }
    }
    index++;
  }
  kgepi->offset = constantOffset->getZExtValue();
}

void Executor::bindInstructionConstants(KInstruction *KI) {
  KGEPInstruction *kgepi = static_cast<KGEPInstruction*>(KI);

  if (GetElementPtrInst *gepi = dyn_cast<GetElementPtrInst>(KI->inst)) {
    computeOffsets(kgepi, gep_type_begin(gepi), gep_type_end(gepi));
  } else if (InsertValueInst *ivi = dyn_cast<InsertValueInst>(KI->inst)) {
    computeOffsets(kgepi, iv_type_begin(ivi), iv_type_end(ivi));
    assert(kgepi->indices.empty() && "InsertValue constant offset expected");
  } else if (ExtractValueInst *evi = dyn_cast<ExtractValueInst>(KI->inst)) {
    computeOffsets(kgepi, ev_type_begin(evi), ev_type_end(evi));
    assert(kgepi->indices.empty() && "ExtractValue constant offset expected");
  }
}

void Executor::bindModuleConstants() {
  for (std::vector<KFunction*>::iterator it = kmodule->functions.begin(), 
         ie = kmodule->functions.end(); it != ie; ++it) {
    KFunction *kf = *it;
    for (unsigned i=0; i<kf->numInstructions; ++i)
      bindInstructionConstants(kf->instructions[i]);
  }
  
  kmodule->constantTable = new Cell[kmodule->constants.size()];
  for (unsigned i=0; i<kmodule->constants.size(); ++i) {    
    Cell &c = kmodule->constantTable[i];
    // assert(c);
    assert(kmodule->constants[i]);
    c.value = evalConstant(kmodule->constants[i]);
  }
}

void Executor::checkMemoryUsage() {
  if (!MaxMemory)
    return;
  if ((stats::instructions & 0xFFFF) == 0) {
    // We need to avoid calling GetTotalMallocUsage() often because it
    // is O(elts on freelist). This is really bad since we start
    // to pummel the freelist once we hit the memory cap.
    unsigned mbs = (util::GetTotalMallocUsage() >> 20) +
                   (memory->getUsedDeterministicSize() >> 20);

    if (mbs > MaxMemory) {
      if (mbs > MaxMemory + 100) {
        // just guess at how many to kill
        unsigned numStates = states.size();
        unsigned toKill = std::max(1U, numStates - numStates * MaxMemory / mbs);
        klee_warning("killing %d states (over memory cap)", toKill);
        std::vector<ExecutionState *> arr(states.begin(), states.end());
        for (unsigned i = 0, N = arr.size(); N && i < toKill; ++i, --N) {
          unsigned idx = rand() % N;
          // Make two pulls to try and not hit a state that
          // covered new code.
          if (arr[idx]->coveredNew)
            idx = rand() % N;

          std::swap(arr[idx], arr[N - 1]);
          terminateStateEarly(*arr[N - 1], "Memory limit exceeded.");
        }
      }
      atMemoryLimit = true;
    } else {
      atMemoryLimit = false;
    }
  }
}

void Executor::doDumpStates() {
  if (!DumpStatesOnHalt || states.empty())
    return;
  klee_message("halting execution, dumping remaining states");
  for (std::set<ExecutionState *>::iterator it = states.begin(),
                                            ie = states.end();
       it != ie; ++it) {
    ExecutionState &state = **it;
    stepInstruction(state); // keep stats rolling
    terminateStateEarly(state, "Execution halting.");
  }
  updateStates(0);
}

//Slightly modified for TASE because we don't use the
//KLEE state tracking structures.
void Executor::run(ExecutionState  & initialState) {
  if (usingSeeds) {
    printf("ERROR: Seeds not supported in TASE \n");
    std::exit(EXIT_FAILURE);
  }
  haltExecution = false;
  while ( !haltExecution) {
    KInstruction *ki = GlobalExecutionStatePtr->pc;
    stepInstruction(*GlobalExecutionStatePtr);
    executeInstruction(*GlobalExecutionStatePtr, ki);
  }
}

std::string Executor::getAddressInfo(ExecutionState &state, 
                                     ref<Expr> address) const{
  std::string Str;
  llvm::raw_string_ostream info(Str);
  info << "\taddress: " << address << "\n";
  uint64_t example;
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(address)) {
    example = CE->getZExtValue();
  } else {
    ref<ConstantExpr> value;
    bool success = solver->getValue(state, address, value);
    assert(success && "FIXME: Unhandled solver failure");
    (void) success;
    example = value->getZExtValue();
    info << "\texample: " << example << "\n";
    std::pair< ref<Expr>, ref<Expr> > res = solver->getRange(state, address);
    info << "\trange: [" << res.first << ", " << res.second <<"]\n";
  }
  
  MemoryObject hack((unsigned) example);    
  MemoryMap::iterator lower = state.addressSpace.objects.upper_bound(&hack);
  info << "\tnext: ";
  if (lower==state.addressSpace.objects.end()) {
    info << "none\n";
  } else {
    const MemoryObject *mo = lower->first;
    std::string alloc_info;
    mo->getAllocInfo(alloc_info);
    info << "object at " << mo->address
         << " of size " << mo->size << "\n"
         << "\t\t" << alloc_info << "\n";
  }
  if (lower!=state.addressSpace.objects.begin()) {
    --lower;
    info << "\tprev: ";
    if (lower==state.addressSpace.objects.end()) {
      info << "none\n";
    } else {
      const MemoryObject *mo = lower->first;
      std::string alloc_info;
      mo->getAllocInfo(alloc_info);
      info << "object at " << mo->address 
           << " of size " << mo->size << "\n"
           << "\t\t" << alloc_info << "\n";
    }
  }

  return info.str();
}

void Executor::pauseState(ExecutionState &state){
  auto it = std::find(continuedStates.begin(), continuedStates.end(), &state);
  // If the state was to be continued, but now gets paused again
  if (it != continuedStates.end()){
    // ...just don't continue it
    std::swap(*it, continuedStates.back());
    continuedStates.pop_back();
  } else {
    pausedStates.push_back(&state);
  }
}

void Executor::continueState(ExecutionState &state){
  auto it = std::find(pausedStates.begin(), pausedStates.end(), &state);
  // If the state was to be paused, but now gets continued again
  if (it != pausedStates.end()){
    // ...don't pause it
    std::swap(*it, pausedStates.back());
    pausedStates.pop_back();
  } else {
    continuedStates.push_back(&state);
  }
}

void Executor::terminateState(ExecutionState &state) {
  if (replayKTest && replayPosition!=replayKTest->numObjects) {
    klee_warning_once(replayKTest,
                      "replay did not consume all objects in test input.");
  }

  interpreterHandler->incPathsExplored();

  std::vector<ExecutionState *>::iterator it =
      std::find(addedStates.begin(), addedStates.end(), &state);
  if (it==addedStates.end()) {
    state.pc = state.prevPC;

    removedStates.push_back(&state);
  } else {
    // never reached searcher, just delete immediately
    std::map< ExecutionState*, std::vector<SeedInfo> >::iterator it3 = 
      seedMap.find(&state);
    if (it3 != seedMap.end())
      seedMap.erase(it3);
    addedStates.erase(it);
    processTree->remove(state.ptreeNode);
    printf(" WOULD NORMAL DELETE STATE HERE \n \n \n \n ");
    delete &state;
  }
}

void Executor::terminateStateEarly(ExecutionState &state, 
                                   const Twine &message) {
  if (!OnlyOutputStatesCoveringNew || state.coveredNew ||
      (AlwaysOutputSeeds && seedMap.count(&state)))
    interpreterHandler->processTestCase(state, (message + "\n").str().c_str(),
                                        "early");
  terminateState(state);
}

void Executor::terminateStateOnExit(ExecutionState &state) {
  if (!OnlyOutputStatesCoveringNew || state.coveredNew || 
      (AlwaysOutputSeeds && seedMap.count(&state)))
    interpreterHandler->processTestCase(state, 0, 0);
  terminateState(state);
}

const InstructionInfo & Executor::getLastNonKleeInternalInstruction(const ExecutionState &state,
    Instruction ** lastInstruction) {
  // unroll the stack of the applications state and find
  // the last instruction which is not inside a KLEE internal function
  ExecutionState::stack_ty::const_reverse_iterator it = state.stack.rbegin(),
      itE = state.stack.rend();

  // don't check beyond the outermost function (i.e. main())
  itE--;

  const InstructionInfo * ii = 0;
  if (kmodule->internalFunctions.count(it->kf->function) == 0){
    ii =  state.prevPC->info;
    *lastInstruction = state.prevPC->inst;
    //  Cannot return yet because even though
    //  it->function is not an internal function it might of
    //  been called from an internal function.
  }

  // Wind up the stack and check if we are in a KLEE internal function.
  // We visit the entire stack because we want to return a CallInstruction
  // that was not reached via any KLEE internal functions.
  for (;it != itE; ++it) {
    // check calling instruction and if it is contained in a KLEE internal function
    const Function * f = (*it->caller).inst->getParent()->getParent();
    if (kmodule->internalFunctions.count(f)){
      ii = 0;
      continue;
    }
    if (!ii){
      ii = (*it->caller).info;
      *lastInstruction = (*it->caller).inst;
    }
  }

  if (!ii) {
    // something went wrong, play safe and return the current instruction info
    *lastInstruction = state.prevPC->inst;
    return *state.prevPC->info;
  }
  return *ii;
}

bool Executor::shouldExitOn(enum TerminateReason termReason) {
  std::vector<TerminateReason>::iterator s = ExitOnErrorType.begin();
  std::vector<TerminateReason>::iterator e = ExitOnErrorType.end();

  for (; s != e; ++s)
    if (termReason == *s)
      return true;

  return false;
}

void Executor::terminateStateOnError(ExecutionState &state,
                                     const llvm::Twine &messaget,
                                     enum TerminateReason termReason,
                                     const char *suffix,
                                     const llvm::Twine &info) {
  std::string message = messaget.str();
  static std::set< std::pair<Instruction*, std::string> > emittedErrors;
  Instruction * lastInst;
  const InstructionInfo &ii = getLastNonKleeInternalInstruction(state, &lastInst);
  
  if (EmitAllErrors ||
      emittedErrors.insert(std::make_pair(lastInst, message)).second) {
    if (ii.file != "") {
      klee_message("ERROR: %s:%d: %s", ii.file.c_str(), ii.line, message.c_str());
    } else {
      klee_message("ERROR: (location information missing) %s", message.c_str());
    }
    if (!EmitAllErrors)
      klee_message("NOTE: now ignoring this error at this location");

    std::string MsgString;
    llvm::raw_string_ostream msg(MsgString);
    msg << "Error: " << message << "\n";
    if (ii.file != "") {
      msg << "File: " << ii.file << "\n";
      msg << "Line: " << ii.line << "\n";
      msg << "assembly.ll line: " << ii.assemblyLine << "\n";
    }
    msg << "Stack: \n";
    state.dumpStack(msg);

    std::string info_str = info.str();
    if (info_str != "")
      msg << "Info: \n" << info_str;

    std::string suffix_buf;
    if (!suffix) {
      suffix_buf = TerminateReasonNames[termReason];
      suffix_buf += ".err";
      suffix = suffix_buf.c_str();
    }

    interpreterHandler->processTestCase(state, msg.str().c_str(), suffix);
  }
    
  terminateState(state);

  if (shouldExitOn(termReason))
    haltExecution = true;
}

// XXX shoot me
static const char *okExternalsList[] = { "printf", 
                                         "fprintf", 
                                         "puts",
                                         "getpid" };
static std::set<std::string> okExternals(okExternalsList,
                                         okExternalsList + 
                                         (sizeof(okExternalsList)/sizeof(okExternalsList[0])));

void Executor::callExternalFunction(ExecutionState &state,
                                    KInstruction *target,
                                    Function *function,
                                    std::vector< ref<Expr> > &arguments) {
  // check if specialFunctionHandler wants it
  if (specialFunctionHandler->handle(state, function, target, arguments))
    return;
  
  if (NoExternals && !okExternals.count(function->getName())) {
    klee_warning("Disallowed call to external function: %s\n",
               function->getName().str().c_str());
    terminateStateOnError(state, "externals disallowed", User);
    return;
  }

  // normal external function handling path
  // allocate 128 bits for each argument (+return value) to support fp80's;
  // we could iterate through all the arguments first and determine the exact
  // size we need, but this is faster, and the memory usage isn't significant.
  uint64_t *args = (uint64_t*) alloca(2*sizeof(*args) * (arguments.size() + 1));
  memset(args, 0, 2 * sizeof(*args) * (arguments.size() + 1));
  unsigned wordIndex = 2;
  for (std::vector<ref<Expr> >::iterator ai = arguments.begin(), 
       ae = arguments.end(); ai!=ae; ++ai) {
    if (AllowExternalSymCalls) { // don't bother checking uniqueness
      ref<ConstantExpr> ce;
      bool success = solver->getValue(state, *ai, ce);
      assert(success && "FIXME: Unhandled solver failure");
      (void) success;
      ce->toMemory(&args[wordIndex]);
      wordIndex += (ce->getWidth()+63)/64;
    } else {
      ref<Expr> arg = toUnique(state, *ai);
      if (ConstantExpr *ce = dyn_cast<ConstantExpr>(arg)) {
        // XXX kick toMemory functions from here
        ce->toMemory(&args[wordIndex]);
        wordIndex += (ce->getWidth()+63)/64;
      } else {
        terminateStateOnExecError(state, 
                                  "external call with symbolic argument: " + 
                                  function->getName());
        return;
      }
    }
  }

  state.addressSpace.copyOutConcretes();

  if (!SuppressExternalWarnings) {

    std::string TmpStr;
    llvm::raw_string_ostream os(TmpStr);
    os << "calling external: " << function->getName().str() << "(";
    for (unsigned i=0; i<arguments.size(); i++) {
      os << arguments[i];
      if (i != arguments.size()-1)
	os << ", ";
    }
    os << ") at ";
    state.pc->printFileLine(os);
    
    if (AllExternalWarnings)
      klee_warning("%s", os.str().c_str());
    else
      klee_warning_once(function, "%s", os.str().c_str());
  }
  bool success = externalDispatcher->executeCall(function, target->inst, args);
  if (!success) {
    terminateStateOnError(state, "failed external call: " + function->getName(),
                          External);
    return;
  }

  if (!state.addressSpace.copyInConcretes()) {
    terminateStateOnError(state, "external modified read-only object",
                          External);
    return;
  }

  Type *resultType = target->inst->getType();
  if (resultType != Type::getVoidTy(function->getContext())) {
    ref<Expr> e = ConstantExpr::fromMemory((void*) args, 
                                           getWidthForLLVMType(resultType));
    bindLocal(target, state, e);
  }
}

ref<Expr> Executor::replaceReadWithSymbolic(ExecutionState &state, 
                                            ref<Expr> e) {
  unsigned n = interpreterOpts.MakeConcreteSymbolic;
  if (!n || replayKTest || replayPath)
    return e;

  // right now, we don't replace symbolics (is there any reason to?)
  if (!isa<ConstantExpr>(e))
    return e;

  if (n != 1 && random() % n)
    return e;

  // create a new fresh location, assert it is equal to concrete value in e
  // and return it.
  
  static unsigned id;
  const Array *array =
      arrayCache.CreateArray("rrws_arr" + llvm::utostr(++id),
                             Expr::getMinBytesForWidth(e->getWidth()));
  ref<Expr> res = Expr::createTempRead(array, e->getWidth());
  ref<Expr> eq = NotOptimizedExpr::create(EqExpr::create(e, res));
  llvm::errs() << "Making symbolic: " << eq << "\n";
  state.addConstraint(eq);
  return res;
}

ObjectState *Executor::bindObjectInState(ExecutionState &state, 
                                         const MemoryObject *mo,
                                         bool isLocal,
                                         const Array *array,
					 bool forTASE ) {
  ObjectState *os = array ? new ObjectState(mo, array,forTASE) : new ObjectState(mo, forTASE);
  state.addressSpace.bindObject(mo, os);

  
  
  // Its possible that multiple bindings of the same mo in the state
  // will put multiple copies on this list, but it doesn't really
  // matter because all we use this list for is to unbind the object
  // on function return.
  if (isLocal)
    state.stack.back().allocas.push_back(mo);

  return os;
}

void Executor::executeAlloc(ExecutionState &state,
                            ref<Expr> size,
                            bool isLocal,
                            KInstruction *target,
                            bool zeroMemory,
                            const ObjectState *reallocFrom) {
  size = toUnique(state, size);
  if (ConstantExpr *CE = dyn_cast<ConstantExpr>(size)) {
    const llvm::Value *allocSite = state.prevPC->inst;
    size_t allocationAlignment = getAllocationAlignment(allocSite);
    MemoryObject *mo =
        memory->allocate(CE->getZExtValue(), isLocal, /*isGlobal=*/false,
                         allocSite, allocationAlignment);
    if (!mo) {
      bindLocal(target, state, 
                ConstantExpr::alloc(0, Context::get().getPointerWidth()));
    } else {
      ObjectState *os = bindObjectInState(state, mo, isLocal);
      if (zeroMemory) {
        os->initializeToZero();
      } else {
        os->initializeToRandom();
      }
      bindLocal(target, state, mo->getBaseExpr());
      
      if (reallocFrom) {
        unsigned count = std::min(reallocFrom->size, os->size);
        for (unsigned i=0; i<count; i++)
          os->write(i, reallocFrom->read8(i));
        state.addressSpace.unbindObject(reallocFrom->getObject());
      }
    }
  } else {
    // XXX For now we just pick a size. Ideally we would support
    // symbolic sizes fully but even if we don't it would be better to
    // "smartly" pick a value, for example we could fork and pick the
    // min and max values and perhaps some intermediate (reasonable
    // value).
    // 
    // It would also be nice to recognize the case when size has
    // exactly two values and just fork (but we need to get rid of
    // return argument first). This shows up in pcre when llvm
    // collapses the size expression with a select.

    ref<ConstantExpr> example;
    bool success = solver->getValue(state, size, example);
    assert(success && "FIXME: Unhandled solver failure");
    (void) success;
    
    // Try and start with a small example.
    Expr::Width W = example->getWidth();
    while (example->Ugt(ConstantExpr::alloc(128, W))->isTrue()) {
      ref<ConstantExpr> tmp = example->LShr(ConstantExpr::alloc(1, W));
      bool res;
      bool success = solver->mayBeTrue(state, EqExpr::create(tmp, size), res);
      assert(success && "FIXME: Unhandled solver failure");      
      (void) success;
      if (!res)
        break;
      example = tmp;
    }

    StatePair fixedSize = fork(state, EqExpr::create(example, size), true);
    
    if (fixedSize.second) { 
      // Check for exactly two values
      ref<ConstantExpr> tmp;
      bool success = solver->getValue(*fixedSize.second, size, tmp);
      assert(success && "FIXME: Unhandled solver failure");      
      (void) success;
      bool res;
      success = solver->mustBeTrue(*fixedSize.second, 
                                   EqExpr::create(tmp, size),
                                   res);
      assert(success && "FIXME: Unhandled solver failure");      
      (void) success;
      if (res) {
        executeAlloc(*fixedSize.second, tmp, isLocal,
                     target, zeroMemory, reallocFrom);
      } else {
        // See if a *really* big value is possible. If so assume
        // malloc will fail for it, so lets fork and return 0.
        StatePair hugeSize = 
          fork(*fixedSize.second, 
               UltExpr::create(ConstantExpr::alloc(1U<<31, W), size),
               true);
        if (hugeSize.first) {
          klee_message("NOTE: found huge malloc, returning 0");
          bindLocal(target, *hugeSize.first, 
                    ConstantExpr::alloc(0, Context::get().getPointerWidth()));
        }
        
        if (hugeSize.second) {

          std::string Str;
          llvm::raw_string_ostream info(Str);
          ExprPPrinter::printOne(info, "  size expr", size);
          info << "  concretization : " << example << "\n";
          info << "  unbound example: " << tmp << "\n";
          terminateStateOnError(*hugeSize.second, "concretized symbolic size",
                                Model, NULL, info.str());
        }
      }
    }

    if (fixedSize.first) // can be zero when fork fails
      executeAlloc(*fixedSize.first, example, isLocal, 
                   target, zeroMemory, reallocFrom);
  }
}

void Executor::executeFree(ExecutionState &state,
                           ref<Expr> address,
                           KInstruction *target) {
  StatePair zeroPointer = fork(state, Expr::createIsZero(address), true);
  if (zeroPointer.first) {
    if (target)
      bindLocal(target, *zeroPointer.first, Expr::createPointer(0));
  }
  if (zeroPointer.second) { // address != 0
    ExactResolutionList rl;
    resolveExact(*zeroPointer.second, address, rl, "free");
    
    for (Executor::ExactResolutionList::iterator it = rl.begin(), 
           ie = rl.end(); it != ie; ++it) {
      const MemoryObject *mo = it->first.first;
      if (mo->isLocal) {
        terminateStateOnError(*it->second, "free of alloca", Free, NULL,
                              getAddressInfo(*it->second, address));
      } else if (mo->isGlobal) {
        terminateStateOnError(*it->second, "free of global", Free, NULL,
                              getAddressInfo(*it->second, address));
      } else {
        it->second->addressSpace.unbindObject(mo);
        if (target)
          bindLocal(target, *it->second, Expr::createPointer(0));
      }
    }
  }
}

void Executor::resolveExact(ExecutionState &state,
                            ref<Expr> p,
                            ExactResolutionList &results, 
                            const std::string &name) {
  // XXX we may want to be capping this?
  ResolutionList rl;
  state.addressSpace.resolve(state, solver, p, rl);
  
  ExecutionState *unbound = &state;
  for (ResolutionList::iterator it = rl.begin(), ie = rl.end(); 
       it != ie; ++it) {
    ref<Expr> inBounds = EqExpr::create(p, it->first->getBaseExpr());
    
    StatePair branches = fork(*unbound, inBounds, true);
    
    if (branches.first)
      results.push_back(std::make_pair(*it, branches.first));

    unbound = branches.second;
    if (!unbound) // Fork failure
      break;
  }

  if (unbound) {
    terminateStateOnError(*unbound, "memory error: invalid pointer: " + name,
                          Ptr, NULL, getAddressInfo(*unbound, p));
  }
}

void Executor::executeMemoryOperation(ExecutionState &state,
                                      bool isWrite,
                                      ref<Expr> address,
                                      ref<Expr> value /* undef if read */,
                                      KInstruction *target /* undef if write */,
                                      const std::string& reason) {

  Expr::Width type = (isWrite ? value->getWidth() : 
                      getWidthForLLVMType(target->inst->getType()));
  unsigned bytes = Expr::getMinBytesForWidth(type);
  
  if (SimplifySymIndices) {
    if (!isa<ConstantExpr>(address))
      address = state.constraints.simplifyExpr(address);
    if (isWrite && !isa<ConstantExpr>(value))
      value = state.constraints.simplifyExpr(value);
  }

  ObjectPair op;
  bool success;
  
//Fast path for TASE where offset is concrete
  ConstantExpr * CE = dyn_cast<ConstantExpr> (address);
  if (CE) {
    success = state.addressSpace.resolveOne(CE, op);
  } else {
    
    // fast path: single in-bounds resolution
    solver->setTimeout(coreSolverTimeout);
    if (!state.addressSpace.resolveOne(state, solver, address, op, success)) {
      std::cout << "resolveOne failure! " << std::endl;
      fflush(stdout);
      address = toConstant(state, address, "resolveOne failure");
      success = state.addressSpace.resolveOne(cast<ConstantExpr>(address), op);
    }
    solver->setTimeout(0);
    
  }
  
  if (!success) {
    std::string ss;
    llvm::raw_string_ostream tmp(ss);
    address->print(tmp);
    std::cout << "Could not resolve address to MO: " << std::hex << std::stoull(tmp.str().c_str(), nullptr, 0) << ", RIP: " << target_ctx_gregs[GREG_RIP].u64 << std::dec <<  "\n";
    std::cout << "Reason: " << reason << "\n";
    std::cout << "address was " << (CE ? "" : "not ") << "a ConstExpr" << std::endl;
  } else {
    const MemoryObject *mo = op.first;

    //------------------New fast path:
    //Cherry-pick in-bounds reads at constant offsets.

    if (optimizeConstMemOps && CE->getZExtValue() + bytes <= mo->address + mo->size) {
      unsigned offset = CE->getZExtValue() - mo->address;
      const ObjectState *os = op.second;
      
      if (isWrite) {
        if (os->readOnly) {
          terminateStateOnError(state, "memory error: object read only", ReadOnly);
        } else {
          //Todo: Implement for writes.  Need to actually apply psn, and
          //add new method for applying poison on write for constant
          //offsets.  For now, it's OK to leave this blank as we fall-through to the
          //older slower logic.
	  
          //ObjectState *wos = state.addressSpace.getWriteable(mo, os);
          //wos->write(offset, value);
          //wos->applyPsnOnWrite(offset,value);
        }
      } else {
        ref<Expr> result = os->read(offset, type);

        //ObjectState *wos = state.addressSpace.getWriteable(mo, os);
        //wos->applyPsnOnRead(offset); //Not needed for const offset
        if (interpreterOpts.MakeConcreteSymbolic)
          result = replaceReadWithSymbolic(state, result);
        bindLocal(target, state, result);
        return;
      }
    }
    //------------------End new fast path
    
    if (MaxSymArraySize && mo->size>=MaxSymArraySize) {
      address = toConstant(state, address, "max-sym-array-size");
    }
    ref<Expr> offset;
    if (CE) {
      if (  CE->getZExtValue() + bytes <= mo->address + mo->size) {
        offset = ConstantExpr::create( CE->getZExtValue() - mo->address, Context::get().getPointerWidth());
      } else  {
        std::string ss;
        llvm::raw_string_ostream tmp(ss);
        address->print(tmp);
        std::cout << "Could not resolve address to MO: " << tmp.str() << "\n";
        std::cout << "Illegal offset in execute memory operation in " << reason << std::endl;
        std::exit(EXIT_FAILURE);
      }
    } else {
      offset = mo->getOffsetExpr(address);
    }
    bool inBounds;
    //Fast path for TASE where offset is concrete
    if (CE) {
      //Should inequality be strictly less than?
      if (  CE->getZExtValue() + bytes <= mo->address + mo->size) {

        inBounds = true;
      } else { 
        //Code duplication: Remove me
        solver->setTimeout(coreSolverTimeout);
        bool success = solver->mustBeTrue(state,
                                          mo->getBoundsCheckOffset(offset, bytes),
                                          inBounds);
        solver->setTimeout(0);
        if (!success) {
          state.pc = state.prevPC;
          terminateStateEarly(state, "Query timed out (bounds check).");
          return;
        }
      }
    } else {
      solver->setTimeout(coreSolverTimeout);
      bool success = solver->mustBeTrue(state, 
                                        mo->getBoundsCheckOffset(offset, bytes),
                                        inBounds);
      solver->setTimeout(0);
      if (!success) {
        state.pc = state.prevPC;
        terminateStateEarly(state, "Query timed out (bounds check).");
        return;
      }
    }
    
    if (inBounds) {
      const ObjectState *os = op.second;
      if (isWrite) {
        if (os->readOnly) {
          terminateStateOnError(state, "memory error: object read only",
                                ReadOnly);
        } else {
          ObjectState *wos = state.addressSpace.getWriteable(mo, os);
          wos->write(offset, value);
          wos->applyPsnOnWrite(offset,value);
        }          
      } else {
        ref<Expr> result = os->read(offset, type);
        ObjectState *wos = state.addressSpace.getWriteable(mo, os);
        wos->applyPsnOnRead(offset);
        if (interpreterOpts.MakeConcreteSymbolic)
          result = replaceReadWithSymbolic(state, result);
        bindLocal(target, state, result);
      }
      
      return;
    }
  } 
  
  // we are on an error path (no resolution, multiple resolution, one
  // resolution with out of bounds)
  
  ResolutionList rl;  
  solver->setTimeout(coreSolverTimeout);
  bool incomplete = state.addressSpace.resolve(state, solver, address, rl,
                                               0, coreSolverTimeout);
  solver->setTimeout(0);

  // XXX there is some query wasteage here. who cares?
  ExecutionState *unbound = &state;
  
  for (ResolutionList::iterator i = rl.begin(), ie = rl.end(); i != ie; ++i) {
    const MemoryObject *mo = i->first;
    const ObjectState *os = i->second;
    ref<Expr> inBounds = mo->getBoundsCheckPointer(address, bytes);
    
    StatePair branches = fork(*unbound, inBounds, true);
    ExecutionState *bound = branches.first;

    // bound can be 0 on failure or overlapped 
    if (bound) {
      if (isWrite) {
        if (os->readOnly) {
          terminateStateOnError(*bound, "memory error: object read only",
                                ReadOnly);
        } else {
          ObjectState *wos = bound->addressSpace.getWriteable(mo, os);
          wos->write(mo->getOffsetExpr(address), value);
          wos->applyPsnOnWrite(mo->getOffsetExpr(address), value);
        }
      } else {
        ref<Expr> result = os->read(mo->getOffsetExpr(address), type);
        ObjectState *wos = bound->addressSpace.getWriteable(mo, os);
        wos->applyPsnOnRead(mo->getOffsetExpr(address));  //Todo: ABH - should this be on os instead of wos?
        bindLocal(target, *bound, result);
      }
    }

    unbound = branches.second;
    if (!unbound)
      break;
  }

  // XXX should we distinguish out of bounds and overlapped cases?
  if (unbound) {
    if (incomplete) {
      terminateStateEarly(*unbound, "Query timed out (resolve).");
    } else {
      terminateStateOnError(*unbound, "memory error: out of bound pointer", Ptr,
                            NULL, getAddressInfo(*unbound, address));
      std::exit(EXIT_FAILURE);
    }
  }
}

void Executor::concretizeGPRArgs( unsigned int argNum, const char * reason) {

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);
  ref<Expr> arg4Expr = target_ctx_gregs_OS->read(GREG_RCX * 8, Expr::Int64);
  ref<Expr> arg5Expr = target_ctx_gregs_OS->read(GREG_R8 * 8, Expr::Int64);
  ref<Expr> arg6Expr = target_ctx_gregs_OS->read(GREG_R9 * 8, Expr::Int64);

  printf("WARNING: Concretizing GPR args for %s \n", reason);
  
  if (argNum == 0 || argNum > 7) {
    printf("ERROR -- concretizeGPRArgs called with invalid number of args %u ", argNum);
    //    worker_exit(EXIT_FAILURE);
    exit(1);
  }

  if (argNum >= 1 ) {
    if (!isa<ConstantExpr>(arg1Expr)) {
      ref<Expr> ConcArg1Expr = toConstant(*GlobalExecutionStatePtr, arg1Expr, reason);
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RDI].u64,ConcArg1Expr); 
    }
  }

  if (argNum >= 2 ) {
    if (!isa<ConstantExpr>(arg2Expr)) {
      ref<Expr> ConcArg2Expr = toConstant(*GlobalExecutionStatePtr, arg2Expr, reason);
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RSI].u64,ConcArg2Expr);
    }
  }

  if (argNum >= 3 ) {
    if (!isa<ConstantExpr>(arg3Expr)) {
      ref<Expr> ConcArg3Expr = toConstant(*GlobalExecutionStatePtr, arg3Expr, reason);
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RDX].u64,ConcArg3Expr);
    }
  }

  if (argNum >= 4 ) {
    if (!isa<ConstantExpr>(arg4Expr)) {
      ref<Expr> ConcArg4Expr = toConstant(*GlobalExecutionStatePtr, arg4Expr, reason);
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RCX].u64,ConcArg4Expr);
    }
  }
  
  if (argNum >= 5 ) {
    if (!isa<ConstantExpr>(arg5Expr)) {
      ref<Expr> ConcArg5Expr = toConstant(*GlobalExecutionStatePtr, arg5Expr, reason);
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_R8].u64,ConcArg5Expr);
    }
  }

  if (argNum >= 6 ) {
    if (!isa<ConstantExpr>(arg6Expr)) {
      ref<Expr> ConcArg6Expr = toConstant(*GlobalExecutionStatePtr, arg6Expr, reason);
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_R9].u64,ConcArg6Expr);
    }
  }
  
  
}

void Executor::executeMakeSymbolic(ExecutionState &state, 
                                   const MemoryObject *mo,
                                   const std::string &name) {

  static int executeMakeSymbolicCalls = 0;  
  executeMakeSymbolicCalls++;

#ifdef TASE_OPENSSL
  if ( executeMakeSymbolicCalls == 1 ) {
    //Bootstrap multipass here for the very first round
    //before we hit a concretized writesocket call
    if (modelDebug) {
      printf("Calling multipass_reset_round and multipass_start_round for first time \n");
      fflush(stdout);
    }

    
    multipass_reset_round(true);
    multipass_start_round(this, false);
  }
#endif
  
  if(modelDebug) {
    printf("Calling executeMakeSymbolic on name %s \n", name.c_str());
    std::cout.flush();
  }

  bool unnamed = (name == "unnamed") ? true : false;
  // Create a new object state for the memory object (instead of a copy).
  
  std::string array_name = get_unique_array_name(name);
  if (modelDebug) {
    printf("DBG executeMakeSymbolic: Encountered unique name for %s \n", array_name.c_str());
    std::cout.flush();
  }
  //See if we have an assignment
  const klee::Array *array = NULL;
  if (!unnamed && prevMPA.bindings.size() != 0) {
    array = prevMPA.getArray(array_name);
  }
  
  bool multipass = false;
  if (array != NULL) {
    if (taseDebug) {
      printf("Found concretization \n");
      std::cout.flush();
    }
    //CVDEBUG("Multi-pass: Concretization found for " << array_name);
    multipass = true;
  } else {
    if (!unnamed && taseDebug) {
      printf("Didn't find concretization \n");
      std::cout.flush();
    }
    array = arrayCache.CreateArray(array_name, mo->size);
    round_symbolics.push_back(array);
  }
  
  bindObjectInState(state, mo, false, array, true /*forTase*/);

  std::vector<unsigned char> *bindings = NULL;
  if (pass_count > 0 && multipass) {
    bindings = prevMPA.getBindings(array_name);
    
    if (!bindings || bindings->size() != mo->size) {

      printf("Bindings mismatch in executeMakeSymbolic; terminating \n");
      //      worker_exit();
      exit(1);
    } else {
      const klee::ObjectState *os = state.addressSpace.findObject(mo);
      klee::ObjectState *wos = state.addressSpace.getWriteable(mo, os);
      assert(wos && "Writeable object is NULL!");
      unsigned idx = 0;
      for (std::vector<unsigned char>::iterator it = bindings->begin();  it!= bindings->end(); it++) {
	wos->write8(idx, *it);
	idx++;
      }
    }
  } else {
    if (modelDebug) {
      printf("DBG executeMakeSymbolic: Created symbolic var for %s \n", name.c_str());
      std::cout.flush();
    }
    state.addSymbolic(mo, array);
    multipass_symbolic_vars++;
  }

}

extern "C" void target_exit() {
  printf("Target exited \n");
  printf("Executed %lu total interp instructions \n", instCtr);
  printf("Execution State has stack size %lu \n", GlobalExecutionStatePtr->stack.size());
  printf("Found %d calls to solver in fork \n", forkSolverCalls);
  
  //  tase_exit();
  exit(0);
}


uint64_t init_trap_RIP = 0;

extern "C" void klee_interp () {
  uint64_t interpCtr_init = interpCtr;
  init_trap_RIP = target_ctx_gregs[GREG_RIP].u64;
  
  if (measureTime) 
    interp_enter_time = util::getWallTime();
  
  if (taseDebug) {
    std::cout << "---------------ENTERING KLEE_INTERP ---------------------- " << std::endl;
  }

  abort_info.classify_and_count();
  if( modelDebug ) {
    abort_info.print();
  }

  ex_state.update( abort_info.type );
  
  if( ex_state == EXECUTION_STATE::BOUNCEBACK ) {
    if (taseDebug){
      std::cout << "Attempting to bounceback to native execution at RIP " << std::hex
		<< target_ctx_gregs[GREG_RIP].u64 << std::dec << " and tran_max = " << tran_max << std::endl;
    }
    return;
  }
  if (taseDebug) {
    printf("Not attempting to bounceback to native execution at RIP 0x%lx \n", target_ctx_gregs[GREG_RIP].u64);
  }
  
  target_ctx_gregs[GREG_R14].u64 = 0; //Kill r14 because it's only used for instrumentation
  GlobalInterpreter->klee_interp_internal();
  
  if (measureTime) 
    measure_interp_time(interpCtr_init, target_ctx_gregs[GREG_R15].u64);

  tran_max = tranMaxArg;
  return; //Returns to loop in main
}


bool Executor::resumeNativeExecution() {
  return false;
}


//This function's purpose is to take a context from native execution 
//and return an llvm function for interpretation.  May interpret through
//a single instruction, or a full basic block.
KFunction * findInterpFunction (tase_greg_t * registers, KModule * kmod) {

  if (taseDebug) {
    printf("Attempting to find interp function \n");
    fflush(stdout);
  }

  KFunction * KInterpFunction;
  uint64_t nativePC = registers[GREG_RIP].u64;

  auto it = IR_KF_Map.find(nativePC);
  if (it != IR_KF_Map.end()) {
    KInterpFunction = it->second;
  } else {
    std::stringstream converter;
    converter << std::hex << nativePC;
    llvm::Function * interpFn = interpModule->getFunction("interp_fn_" + converter.str());
    KInterpFunction = kmod->functionMap[interpFn];
    IR_KF_Map.insert(std::make_pair(nativePC, KInterpFunction));
  }
  
  if (!KInterpFunction) {
    printf("Unable to find interp function for entrypoint PC 0x%lx \n", nativePC);
    fflush(stdout);
    //    worker_exit(EXIT_FAILURE);
    exit(1);
  } else {
    if (taseDebug) {
      printf("Found interp function \n");
      fflush(stdout);
    }
  }
  return KInterpFunction;
}
//Here's the interface expected for the llvm interpretation function.

// void @interp_fn_PCValHere( %tase_greg_t * %target_ctx_ptr) {
//; Emulate the native code modeled in the function, including an
//; updated program counter.  %target_ctx_ptr will take the initial tase_greg_t ctx,
//; and the necessary interpretation will occur in-place at the ctx pointed to. Also need 
//; to perform loads and stores to main memory.  By this point, an llvm load/store to
//; a given address in the interpreter will result in a load/store from/to the actual
//; native memory address.
// }

//Just an external trap for making a byte symbolic
void Executor::make_byte_symbolic_model() {
  printf("Hit make_byte_symbolic_model on addr 0x%lx \n", target_ctx_gregs[GREG_RDI].u64);
  fflush(stdout);
  
  uint64_t addr = target_ctx_gregs[GREG_RDI].u64;
  tase_make_symbolic_internal(addr, 1, "external_request");

  //fake a ret
  uint64_t retAddr = *((uint64_t *) target_ctx_gregs[GREG_RSP].u64);
  target_ctx_gregs[GREG_RIP].u64 = retAddr;
  target_ctx_gregs[GREG_RSP].u64 += 8;

}

//Populate a buffer at addr with len bytes of unconstrained symbolic data.
//We make the symbolic memory object at a malloc'd address and write the bytes to addr.
void Executor::tase_make_symbolic_internal(uint64_t addr, uint64_t len, const char * name)  {

  if (addr %2 != 0)
    printf("WARNING: tase_make_symbolic_internal called on unaligned object \n");

  void * buf = malloc(len);
  MemoryObject * bufMO = memory->allocateFixed((uint64_t) buf, len,  NULL);
  std::string nameString = name;
  bufMO->name = nameString;
  executeMakeSymbolic(*GlobalExecutionStatePtr, bufMO, name);
  const ObjectState * constBufOS = GlobalExecutionStatePtr->addressSpace.findObject(bufMO);
  ObjectState * bufOS = GlobalExecutionStatePtr->addressSpace.getWriteable(bufMO, constBufOS);

  for (uint64_t i = 0; i < len; i++) {
    tase_helper_write(addr + i, bufOS->read(i, Expr::Int8));
  }


}

void Executor::tase_make_symbolic_bytewise(uint64_t addr, uint64_t len, const char * name) {
  for (uint64_t i = 0; i < len; i++) {
    std::string s = "_byte_" + std::to_string(i); 
    std::string varName = std::string(name) + s;

    tase_make_symbolic_internal(addr + i, 1, varName.c_str());
  }		       
}

/*void Executor::model_sb_disabled() {
  target_ctx_gregs[GREG_RIP].u64 = target_ctx_gregs[GREG_R15].u64;
  }*/

void Executor::model_sb_reopen() {
  target_ctx_gregs[GREG_RIP].u64 = target_ctx_gregs[GREG_RAX].u64;
}



//Todo: Double check the edge cases and make sure we handle
//buffers allocated at odd addresses.
bool Executor::isBufferEntirelyConcrete (uint64_t addr, int size) {

  //Fast path
  bool definitelyConcrete = !tase_buf_has_taint((void *) addr, size);
  if (definitelyConcrete)
    return true;
  
  //Slow path
  uint64_t byteItr = (uint64_t) addr;
  for (int i = 0; i < size; i++) {
    ref<Expr> byteExpr = tase_helper_read ( byteItr +i , 1);
    if (!(isa<ConstantExpr> (byteExpr)))
      return false;	
  }
  return true;
}
  
//Todo:  Make fastpath lookup where we leverage poison checking instead of
//full lookup.
//tase_helper_read: This func exists because reads and writes to
//object states in klee need to happen as offsets to buffers that have
//already been allocatated.  
ref<Expr> Executor::tase_helper_read (uint64_t addr, uint8_t byteWidth) {

  ref<Expr> addrExpr = ConstantExpr::create(addr, Expr::Int64);

  //Find a better way to do this.  Using logic from Executor::executeMemoryOperation
  //begin gross------------------
  ObjectPair op;
  bool success;
  
  if (!(GlobalExecutionStatePtr->addressSpace.resolveOne(*GlobalExecutionStatePtr,solver,addrExpr, op, success))) {
    printf("ERROR in tase_helper_read: Couldn't resolve addr to fixed value \n");
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }
  ref<Expr> offset = op.first->getOffsetExpr(addrExpr);
  //end gross----------------------------


  ref<Expr> returnVal;

  ObjectState *wos = GlobalExecutionStatePtr->addressSpace.getWriteable(op.first, op.second);

  
   switch (byteWidth) {
  case 1:
    returnVal = wos->read(offset, Expr::Int8);
    break;
  case 2:
    returnVal = wos->read(offset, Expr::Int16);
    break;
  case 4:
    returnVal = wos->read(offset, Expr::Int32);
    break;
  case 8:
    returnVal = wos->read(offset, Expr::Int64);
    break;
  default:
    printf("Unrecognized byteWidth in tase_helper_read: %u \n", byteWidth);
    std::cout.flush();
    std::exit(EXIT_FAILURE);
    } 


  wos->applyPsnOnRead(offset);
  return returnVal;
}


template<typename T1, typename T2, typename... Ts>
bool Executor::tase_map(const std::string& name, T1 t1, T2 t2, Ts... ts){
  bool a = tase_map(t1, name) & tase_map(name, t2, ts...);
  if ( !a ) {
    std::cout << "error mapping buffer: " << name << std::endl;
  }

  return a;
}

// for things like ptr into buffer for start/end/current position...
template<typename T>
bool Executor::tase_map(const T*& t, const std::string& name){
  bool a = tase_map_buf((uint64_t) &t, sizeof(T*), name);
  if( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

template<typename T>
bool Executor::tase_map(T* const & t, const size_t& size, const std::string& name){
  bool a = tase_map_buf((uint64_t) &t, sizeof(T*), name);
  bool b = tase_map_buf((uint64_t) t, sizeof(T) * size, name);
  if ( !a || !b ) {
    std::cout << "Error mapping buffer: " << name << " - generic pointer " << ( !a ? "a" : "b" ) << " size " << size * sizeof(T) << std::endl;
  }
  return a && b;
}

template bool Executor::tase_map<char>(char* const & t, const size_t& size, const std::string& name); // force instantiation
//template ObjectState *  Executor::tase_map<unsigned char>(unsigned char* const & t, const size_t& size); // force instantiation

// assume null-terminated
template<>
bool Executor::tase_map(char* const & t, const std::string& name){
  auto a = t == NULL ? tase_map_buf((uint64_t) &t, sizeof(char*), name) : tase_map(t, strlen(t)+1, name);
  if ( !a ) {
    std::cout << "Error mapping buffer: " << name << " - " << ( t == NULL ? "NULL" : "non-NULL" ) << std::endl;
  }
  return a;
}

template<>
bool Executor::tase_map(void* const & t, const std::string& name){
  bool a = tase_map_buf((uint64_t) &t, sizeof(void*), name);
  if ( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

typedef size_t (read_t)(FILE*, unsigned char*, size_t);
typedef size_t (write_t)(FILE*, const unsigned char *, size_t);
typedef off_t (seek_t)(FILE*, off_t, int);

template<>
bool Executor::tase_map(read_t* const& t, const std::string& name){
  bool a = tase_map_buf((uint64_t)&t, sizeof(read_t*), name);
  if ( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

template<>
bool Executor::tase_map(write_t* const& t, const std::string& name){
  bool a = tase_map_buf((uint64_t)&t, sizeof(write_t*), name);
    if ( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

template<>
bool Executor::tase_map(seek_t* const& t, const std::string& name){
  bool a = tase_map_buf((uint64_t) &t, sizeof(seek_t*), name);
    if ( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

// ptr default
//template<typename T>
/*bool Executor::tase_map(const T*& t){
  auto x = tase_map_buf((uint64_t) &t, sizeof(t));
  return t == NULL ? x : tase_map_buf((uint64_t) t, sizeof(T));
}*/

// default
template<typename T>
bool Executor::tase_map(const T& t, const std::string& name){
  bool a = tase_map_buf((uint64_t) &t, sizeof(t), name);
    if ( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

template bool Executor::tase_map<uint64_t>(const uint64_t&, const std::string& name);


template<>
bool Executor::tase_map(FILE* const & t, const std::string& name){
  bool a = tase_map_buf((uint64_t) &t, sizeof(FILE*), name);
    a &= (t == NULL ? a : tase_map(*t, name));
  if ( !a ) {
    std::cout << "Error mapping buffer: " << name << std::endl;
  }
  return a;
}

/*template<>
bool Executor::tase_map(const FILE& t){
  tase_map(t.flags, t.rpos, t.rend, t.close, t.wend, t.wpos, t.mustbezero_1, t.wbase, t.read, t.write, t.seek);
  tase_map(t.buf, t.buf_size);
  return tase_map(t.prev, t.next, t.fd, t.pipe_pid, t.lockcount, t.mode, t.lock, t.lbf, t.cookie, t.off, t.getln_buf,
           t.mustbezero_2, t.shend, t.shlim, t.shcnt, t.prev_locked, t.next_locked, t.locale);
}*/

//Todo -- make this play nice with our alignment requirements
 bool Executor::tase_map_buf(uint64_t addr, size_t size, const std::string& name) {
   return addExternalObjectCheck(*GlobalExecutionStatePtr, (void *) addr, size, false, name, true);
   //MemoryObject * MOres = addExternalObject(*GlobalExecutionStatePtr, (void *) addr, size, false, name, true);
  //  const ObjectState * OSConst = GlobalExecutionStatePtr->addressSpace.findObject(MOres);
  //  ObjectState * OSres = GlobalExecutionStatePtr->addressSpace.getWriteable(MOres, OSConst);
  //  OSres->concreteStore = (uint8_t *) addr;
   // return OSres;
}

//Todo: See if we can make this faster with the poison checking scheme.
//tase_helper_write: Helper function similar to tase_helper_read
//that helps us avoid rewriting the offset-lookup logic over and over.
void Executor::tase_helper_write(uint64_t addr, ref<Expr> val) {
  
  ref<Expr> addrExpr = ConstantExpr::create(addr, Expr::Int64);

  //Find a better way to do this.  Using logic from Executor::executeMemoryOperation
  //begin gross------------------
  ObjectPair op;
  bool success;
  if (! GlobalExecutionStatePtr->addressSpace.resolveOne(*GlobalExecutionStatePtr,solver,addrExpr, op, success) ) {
    printf("ERROR in tase_helper_write: Couldn't resolve addr to fixed value \n");
    fflush(stdout);
    std::exit(EXIT_FAILURE);
  }

  ref<Expr> offset = op.first->getOffsetExpr(addrExpr);
  //end gross----------------------------
  ObjectState *wos = GlobalExecutionStatePtr->addressSpace.getWriteable(op.first, op.second);

  wos->write(offset, val);
  wos->applyPsnOnWrite(offset, val);

}


//CAUTION: This only works if the pc points to the beginning of the cartridge 
bool cartridgeHasFlagsDead(uint64_t pc) {
  return (cartridges_with_flags_live.find(pc) == cartridges_with_flags_live.end());
}


void Executor::model_tase_make_symbolic() {

  ref<Expr> arg1Expr = target_ctx_gregs_OS->read(GREG_RDI * 8, Expr::Int64);
  ref<Expr> arg2Expr = target_ctx_gregs_OS->read(GREG_RSI * 8, Expr::Int64);
  ref<Expr> arg3Expr = target_ctx_gregs_OS->read(GREG_RDX * 8, Expr::Int64);

  if  (
       (isa<ConstantExpr>(arg1Expr)) &&
       (isa<ConstantExpr>(arg2Expr)) &&
       (isa<ConstantExpr>(arg3Expr))
       ){


    uint64_t addr = target_ctx_gregs[GREG_RDI].u64;
    uint64_t size = target_ctx_gregs[GREG_RSI].u64; //Todo -- impose a reasonable upper limit on size.
    char * name = (char *) target_ctx_gregs[GREG_RDX].u64;
    
    tase_make_symbolic_internal(addr, size, name);
    do_ret();
    
    
  } else {

    printf("ERROR in model_tase_make_symbolic -- args must all be concrete. \n");
    std::cout.flush();
    std::exit(EXIT_FAILURE);
  }

}

//static int model_malloc_calls = 0;



/*Macro for (M)odeled (F)unction (E)ntries
#define MFE(x, y) fnModelMap.insert(std::make_pair( (uint64_t)   &x , &klee::Executor::y )); \
fnModelMap.insert(std::make_pair( (uint64_t)   &x  + trap_off, &klee::Executor::y ))*/

/* Example for MFE(calloc_tase, model_calloc):
fnModelMap.insert(std::make_pair( (uint64_t) &calloc_tase , &klee::Executor::model_calloc ));
fnModelMap.insert(std::make_pair( (uint64_t) &calloc_tase + trap_off, &klee::Executor::model_calloc ));
*/

void Executor::loadFnModelMap() {
  fnModelMap = {
		//  {(uint64_t) &calloc_tase_shim, &Executor::model_calloc},
  {(uint64_t) &calloc_tase,  &Executor::model_calloc},
  {(uint64_t) &__ctype_b_loc,  &Executor::model___ctype_b_loc},
  {(uint64_t) &__ctype_tolower_loc,  &Executor::model___ctype_tolower_loc},
  {(uint64_t) &__errno_location,  &Executor::model___errno_location},
  {(uint64_t) &exit_tase_shim,  &Executor::model_exit_tase_success},
  {(uint64_t) &exit_tase,  &Executor::model_exit_tase},
  {(uint64_t) &fclose,  &Executor::model_fclose},
  {(uint64_t) &fcntl,  &Executor::model_fcntl},
  {(uint64_t) &feof_tase,  &Executor::model_feof},
  {(uint64_t) &ferror_tase,  &Executor::model_ferror},
  {(uint64_t) &fflush_tase,  &Executor::model_fflush},
  //  {(uint64_t) &fflush_tase_shim, &Executor::model_fflush},
  {(uint64_t) &fflush_unlocked, &Executor::model_fflush_unlocked},
  {(uint64_t) &fgets,  &Executor::model_fgets},
  {(uint64_t) &fileno_tase,  &Executor::model_fileno},
  {(uint64_t) &fopen_tase,  &Executor::model_fopen},
  {(uint64_t) &fopen64,  &Executor::model_fopen64},
  {(uint64_t) &fclose_tase, &Executor::model_fclose},
  {(uint64_t) &fread_tase,  &Executor::model_fread},
  {(uint64_t) &fread_unlocked_tase,  &Executor::model_fread_unlocked}, //double check against model_fread
  //  {(uint64_t) &free,  &Executor::model_free},
  {(uint64_t) &free_tase,  &Executor::model_free},
  //  {(uint64_t) &free_tase_shim, &Executor::model_free},
  {(uint64_t) &freopen_tase,  &Executor::model_freopen},
  {(uint64_t) &fseek_tase,  &Executor::model_fseek},
  {(uint64_t) &ftell_tase,  &Executor::model_ftell},
  {(uint64_t) &fwrite_tase,  &Executor::model_fwrite},
  {(uint64_t) &fwrite_unlocked,  &Executor::model_fwrite}, //double check against model_fwrite
  //  {(uint64_t) &getc_unlocked,  &Executor::model_getc_unlocked},
  {(uint64_t) &getc_unlocked_tase,  &Executor::model_getc_unlocked},
  //  {(uint64_t) &getc_unlocked_tase_shim, &Executor::model_getc_unlocked},
  {(uint64_t) &getegid,  &Executor::model_getegid},
  {(uint64_t) &getenv,  &Executor::model_getenv},
  {(uint64_t) &geteuid,  &Executor::model_geteuid},
  {(uint64_t) &getgid,  &Executor::model_getgid},
  {(uint64_t) &gethostbyname,  &Executor::model_gethostbyname},
  {(uint64_t) &getpid,  &Executor::model_getpid},
  {(uint64_t) &gettimeofday,  &Executor::model_gettimeofday},
  {(uint64_t) &getuid,  &Executor::model_getuid},
  {(uint64_t) &gmtime,  &Executor::model_gmtime},
  {(uint64_t) &isatty_tase,  &Executor::model_isatty},
  {(uint64_t) &__isoc99_sscanf,  &Executor::model___isoc99_sscanf},
  {(uint64_t) &make_byte_symbolic,  &Executor::make_byte_symbolic_model},
  //  {(uint64_t) &malloc,  &Executor::model_malloc},
  {(uint64_t) &malloc_tase,  &Executor::model_malloc},
  //  {(uint64_t) &malloc_tase_shim, &Executor::model_malloc},
  //  {(uint64_t) &memcpy,  &Executor::model_memcpy_tase},
  {(uint64_t) &memcpy_tase,  &Executor::model_memcpy_tase},
  {(uint64_t) &posix_fadvise_tase,  &Executor::model_posix_fadvise},
  {(uint64_t) &putchar_tase,  &Executor::model_putchar},
  {(uint64_t) &printf_tase,  &Executor::model_printf},
  {(uint64_t) &vsnprintf_tase, &Executor::model_vsnprintf},
  {(uint64_t) &vasprintf_tase, &Executor::model_vasprintf},
  {(uint64_t) &mbsrtowcs, &Executor::model_mbsrtowcs},
  {(uint64_t) &setlocale, &Executor::model_setlocale},
  {(uint64_t) &sigemptyset, &Executor::model_sigemptyset},
  {(uint64_t) &sigaddset, &Executor::model_sigaddset},
  {(uint64_t) &sigprocmask, &Executor::model_sigprocmask},
  {(uint64_t) &sigaction, &Executor::model_sigaction},
  {(uint64_t) &__printf_chk,  &Executor::model___printf_chk},
  {(uint64_t) &puts_tase,  &Executor::model_puts},
  //  {(uint64_t) &puts_tase_shim, &Executor::model_puts},

  //{(uint64_t) &read,  &Executor::model_read}, //Model doesn't exist yet.  fixme.
  //  {(uint64_t) &realloc,  &Executor::model_realloc},
  {(uint64_t) &realloc_tase,  &Executor::model_realloc},
  //  {(uint64_t) &realloc_tase_shim, &Executor::model_realloc},
  {(uint64_t) &rewind_tase,  &Executor::model_rewind},
  //  {(uint64_t) &sb_disabled,  &Executor::model_sb_disabled},
  {(uint64_t) &sb_reopen,  &Executor::model_sb_reopen},
  //  {(uint64_t) &sb_eject, &Executor::model_sb_eject},
  {(uint64_t) &sprintf_tase,  &Executor::model_sprintf},
  {(uint64_t) &sscanf,  &Executor::model___isoc99_sscanf},//Check to make sure it's OK to model with C99
  {(uint64_t) &stat,  &Executor::model_stat},
  //{(uint64_t) &target_exit,  &Executor::model_target_exit}, //Special case. fixme. //Doesn't look like we call target_exit anymore.
  {(uint64_t) &time,  &Executor::model_time},
  {(uint64_t) &fprintf_tase, &Executor::model_fprintf},
//  {(uint64_t) &vfprintf,  &Executor::model_vfprintf},
  {(uint64_t) &write_tase,  &Executor::model_write},
  {(uint64_t) &ioctl, &Executor::model_ioctl},
  {(uint64_t) &strtof_tase,  &Executor::model_strtof},
  {(uint64_t) &strtod_tase,  &Executor::model_strtod},
  {(uint64_t) &strtold_tase,  &Executor::model_strtold},
  {(uint64_t) &strtol_tase,  &Executor::model_strtol},
  {(uint64_t) &strtoll_tase,  &Executor::model_strtoll},
  {(uint64_t) &strtoul_tase,  &Executor::model_strtoul},
  {(uint64_t) &strtoull_tase,  &Executor::model_strtoull},
  {(uint64_t) &strtoimax_tase,  &Executor::model_strtoimax},
  {(uint64_t) &strtoumax_tase,  &Executor::model_strtoumax},

  {(uint64_t) &wcstof_tase,  &Executor::model_wcstof},
  {(uint64_t) &wcstod_tase,  &Executor::model_wcstod},
  {(uint64_t) &wcstold_tase,  &Executor::model_wcstold},
  {(uint64_t) &wcstol_tase,  &Executor::model_wcstol},
  {(uint64_t) &wcstoll_tase,  &Executor::model_wcstoll},
  {(uint64_t) &wcstoul_tase,  &Executor::model_wcstoul},
  {(uint64_t) &wcstoull_tase,  &Executor::model_wcstoull},
  {(uint64_t) &wcstoumax_tase,  &Executor::model_wcstoumax},
  {(uint64_t) &wcstoimax_tase,  &Executor::model_wcstoimax},

 // {(uint64_t) &a_ctz_64_tase,  &Executor::model_a_ctz_64},
 // {(uint64_t) &a_clz_64_tase,  &Executor::model_a_clz_64},

  {(uint64_t) &tase_make_symbolic,  &Executor::model_tase_make_symbolic},

  {(uint64_t) &__pthread_self_tase,  &Executor::model___pthread_self},

#ifdef TASE_OPENSSL
  {(uint64_t) &AES_encrypt,  &Executor::model_AES_encrypt},
  {(uint64_t) &BIO_printf,  &Executor::model_BIO_printf},
  {(uint64_t) &ECDH_compute_key,  &Executor::model_ECDH_compute_key},
  {(uint64_t) &EC_KEY_generate_key,  &Executor::model_EC_KEY_generate_key},
  {(uint64_t) &EC_POINT_point2oct,  &Executor::model_EC_POINT_point2oct},
  {(uint64_t) &gcm_ghash_4bit,  &Executor::model_gcm_ghash_4bit},
  {(uint64_t) &gcm_gmult_4bit,  &Executor::model_gcm_gmult_4bit},
  {(uint64_t) &ktest_connect,  &Executor::model_ktest_connect},
  {(uint64_t) &ktest_master_secret,  &Executor::model_ktest_master_secret},
  {(uint64_t) &ktest_RAND_bytes,  &Executor::model_ktest_RAND_bytes},
  {(uint64_t) &ktest_RAND_pseudo_bytes,  &Executor::model_ktest_RAND_pseudo_bytes},
  {(uint64_t) &ktest_raw_read_stdin,  &Executor::model_ktest_raw_read_stdin},
  {(uint64_t) &ktest_readsocket,  &Executor::model_ktest_readsocket},
  {(uint64_t) &ktest_select,  &Executor::model_ktest_select},
  {(uint64_t) &ktest_start,  &Executor::model_ktest_start},
  {(uint64_t) &ktest_writesocket,  &Executor::model_ktest_writesocket},
  {(uint64_t) &OpenSSLDie,  &Executor::model_OpenSSLDie},
  {(uint64_t) &RAND_add,  &Executor::model_RAND_add},
  {(uint64_t) &RAND_load_file,  &Executor::model_RAND_load_file},
  {(uint64_t) &RAND_poll,  &Executor::model_RAND_poll},
  {(uint64_t) &SHA1_Final,  &Executor::model_SHA1_Final},
  {(uint64_t) &SHA1_Update,  &Executor::model_SHA1_Update},
  {(uint64_t) &SHA256_Final,  &Executor::model_SHA256_Final},
  {(uint64_t) &SHA256_Update,  &Executor::model_SHA256_Update},
  {(uint64_t) &select,  &Executor::model_select}, //Maybe move to generic section?
  {(uint64_t) &setsockopt,  &Executor::model_setsockopt},
  {(uint64_t) &shutdown,  &Executor::model_shutdown},
  {(uint64_t) &signal,  &Executor::model_signal},
  {(uint64_t) &socket,  &Executor::model_socket},
  {(uint64_t) &tls1_generate_master_secret,  &Executor::model_tls1_generate_master_secret},
#endif

  {(uint64_t) &__addsf3_tase, &Executor::model__addsf3},
  {(uint64_t) &__adddf3_tase, &Executor::model__adddf3},
  {(uint64_t) &__subsf3_tase, &Executor::model__subsf3},
  {(uint64_t) &__subdf3_tase, &Executor::model__subdf3},
  {(uint64_t) &__mulsf3_tase, &Executor::model__mulsf3},
  {(uint64_t) &__muldf3_tase, &Executor::model__muldf3},
  {(uint64_t) &__divsf3_tase, &Executor::model__divsf3},
  {(uint64_t) &__divdf3_tase, &Executor::model__divdf3},
  {(uint64_t) &__negsf2_tase, &Executor::model__negsf2},
  {(uint64_t) &__negdf2_tase, &Executor::model__negdf2},
  {(uint64_t) &__extendsfdf2_tase, &Executor::model__extendsfdf2},
  {(uint64_t) &__truncdfsf2_tase, &Executor::model__truncdfsf2},
  {(uint64_t) &__fixsfsi_tase, &Executor::model__fixsfsi},
  {(uint64_t) &__fixdfsi_tase, &Executor::model__fixdfsi},
  {(uint64_t) &__fixsfdi_tase, &Executor::model__fixsfdi},
  {(uint64_t) &__fixdfdi_tase, &Executor::model__fixdfdi},
  {(uint64_t) &__fixsfti_tase, &Executor::model__fixsfti},
  {(uint64_t) &__fixdfti_tase, &Executor::model__fixdfti},
  {(uint64_t) &__fixunssfsi_tase, &Executor::model__fixunssfsi},
  {(uint64_t) &__fixunsdfsi_tase, &Executor::model__fixunsdfsi},
  {(uint64_t) &__fixunssfdi_tase, &Executor::model__fixunssfdi},
  {(uint64_t) &__fixunsdfdi_tase, &Executor::model__fixunsdfdi},
  {(uint64_t) &__fixunssfti_tase, &Executor::model__fixunssfti},
  {(uint64_t) &__fixunsdfti_tase, &Executor::model__fixunsdfti},
  {(uint64_t) &__floatsisf_tase, &Executor::model__floatsisf},
  {(uint64_t) &__floatsidf_tase, &Executor::model__floatsidf},
  {(uint64_t) &__floatdisf_tase, &Executor::model__floatdisf},
  {(uint64_t) &__floatdidf_tase, &Executor::model__floatdidf},
  {(uint64_t) &__floattisf_tase, &Executor::model__floattisf},
  {(uint64_t) &__floattidf_tase, &Executor::model__floattidf},
  {(uint64_t) &__floatunsisf_tase, &Executor::model__floatunsisf},
  {(uint64_t) &__floatunsidf_tase, &Executor::model__floatunsidf},
  {(uint64_t) &__floatundisf_tase, &Executor::model__floatundisf},
  {(uint64_t) &__floatundidf_tase, &Executor::model__floatundidf},
  {(uint64_t) &__floatuntisf_tase, &Executor::model__floatuntisf},
  {(uint64_t) &__floatuntidf_tase, &Executor::model__floatuntidf},
  {(uint64_t) &__cmpsf2_tase, &Executor::model__cmpsf2},
  {(uint64_t) &__cmpdf2_tase, &Executor::model__cmpdf2},
  {(uint64_t) &__unordsf2_tase, &Executor::model__unordsf2},
  {(uint64_t) &__unorddf2_tase, &Executor::model__unorddf2},
  {(uint64_t) &__eqsf2_tase, &Executor::model__eqsf2},
  {(uint64_t) &__eqdf2_tase, &Executor::model__eqdf2},
  {(uint64_t) &__nesf2_tase, &Executor::model__nesf2},
  {(uint64_t) &__nedf2_tase, &Executor::model__nedf2},
  {(uint64_t) &__gesf2_tase, &Executor::model__gesf2},
  {(uint64_t) &__gedf2_tase, &Executor::model__gedf2},
  {(uint64_t) &__ltsf2_tase, &Executor::model__ltsf2},
  {(uint64_t) &__ltdf2_tase, &Executor::model__ltdf2},
  {(uint64_t) &__lesf2_tase, &Executor::model__lesf2},
  {(uint64_t) &__ledf2_tase, &Executor::model__ledf2},
  {(uint64_t) &__gtsf2_tase, &Executor::model__gtsf2},
  {(uint64_t) &__gtdf2_tase, &Executor::model__gtdf2},
  {(uint64_t) &__powisf2_tase, &Executor::model__powisf2},
  {(uint64_t) &__powidf2_tase, &Executor::model__powidf2},
  };

  std::map<uint64_t , void (Executor::*) (void) > cpy(fnModelMap);
  printf("fnModelMap locations:\n");
  for(auto& x : cpy){
    printf("maploc: %lx, %lx\n", x.first, x.first + trap_off);
    fnModelMap.insert({x.first + trap_off, x.second});
  }
  //printf("Loading float emulation models \n");
  //  loadFloatModelMap();
}


//Soft float entry macro
//#define SFE(x) fnModelMap.insert(std::make_pair( (uint64_t) &__ ## x ## _tase, &klee::Executor::model__ ## x )); \
//  fnModelMap.insert(std::make_pair( (uint64_t) &__ ## x ## _tase + trap_off, &klee::Executor::model__ ## x ))


/* Example for SFE(adddf3): 
  fnModelMap.insert(std::make_pair((uint64_t) &__adddf3_tase, &klee::Executor::model__adddf3));
  fnModelMap.insert(std::make_pair((uint64_t) &__adddf3_tase + trap_off, &klee::Executor::model__adddf3));
*/


/*void Executor::loadFloatModelMap() {
  for(auto x: floatModels){
    fnModelMap.insert(x);
    fnModelMap.insert({x.first + trap_off, x.second});
  }
  }*/

// void Executor::model_inst () {
//   run_model_count++;
//   uint64_t rip = target_ctx_gregs[GREG_RIP].u64;
//   if (taseDebug) {
//     printf("INTERPRETER: FOUND SPECIAL MODELED INST at rip 0x%lx \n", rip);
//     fflush(stdout);
//   }


//   auto it = fnModelMap.find(rip);
//   if (it != fnModelMap.end() ) {
//     void (klee::Executor::*fp)() = it->second;
//     (this->*fp)();
//   }

// }

// bool isSpecialInst (uint64_t rip ) {
//   if (taseDebug) {
//     printf("In isSpecialInst for rip 0x%lx \n", rip);
//     fflush(stdout);
//   }

//   auto it = fnModelMap.find(rip);
//   if (it != fnModelMap.end())
//     return true;
//   else
//     return false;

// }

void Executor::printDebugInterpHeader() {

  printf("------------------------------------------- \n");
  printf("Entering interpreter for time %lu and instCtr %lu \n \n \n", interpCtr, instCtr);
  uint64_t rip = target_ctx_gregs[GREG_RIP].u64;
  printf("RIP is %lu in decimal, 0x%lx in hex.\n", rip, rip);
  printf("Initial ctx BEFORE interpretation is \n");
  printCtx();
  printf("\n");
  std::cout.flush();
  
}

void Executor::printDebugInterpFooter() {
  printCtx();
  printf("Executor has %lu states \n", this->states.size() );
  printf("Finished round %lu of interpretation. \n", interpCtr);
  printf("-------------------------------------------\n");
}


// set based on poison_size and xmm vs zmm in main
extern "C" bool (*large_buf_has_taint)(const uint16_t*, const int);

//Fast-path check for poison tag in buffer.
//Todo -- double check the corner cases
extern "C" bool tase_buf_has_taint (const void * ptr, const int size) {
  // simpler construction: find end of original buffer
  // adjust start, loop until we're out of the original bounds, don't bother calculating size
  uint16_t * const end = (uint16_t*) (((uint64_t) ptr) + size);
  uint16_t * const start = ((uint64_t) ptr) % 2 == 1 ? (uint16_t *) ((uint64_t) ptr - 1 ) : (uint16_t *) ptr;

  if( target_ctx.poisonSize == 2 ) {
    for( uint16_t *x = start; end - x > 0; x++ ) {
      if( *x == poison_val )
	return true;
    }
  } else {
    for( uint16_t *x = start; end - x > 0; x += 2 ) {
      if( *x == poison_val && *(x+1) == poison_val )
	return true;
    }
  }
  return false;
  
  //  const int checkSize = ( ((uint8_t*) ptr + size + 1 ) - (uint8_t*) checkBase ) / 2;

  /*  if( checkSize > 4 ) { // more than 64 bits/8 btyes -> use xmm/ymms.  
    return large_buf_has_taint(checkBase, checkSize);
    }*/

  /*  if( target_ctx.poisonSize == 2 ) {
    for ( int i = 0; i < checkSize; i++ ) {
      if ( *( checkBase + i ) == poison_val )
        return true;
    }
  } else {
    for( int i = 0; i < checkSize/2; i++ ) {
      if( *( checkBase + 2*i ) == poison_val && *( checkBase + 2*i + 1 ) == poison_val )
	return true;
    }
  }
  
  return false;*/
}


// pretty much what we do in the springboard
extern "C" bool large_buf_has_taint_16_128(const uint16_t * checkBase, const int checkSize){
  const int x = checkSize % 8;
  const int rounds = x == 0 ? (checkSize / 8) : (checkSize / 8) + 1;

  // mask off leftovers
  const __m128i mask = _mm_setr_epi16( x == 0 ? 0xffff : 0x0000, x > 6 ? 0xffff : 0x0000, x > 5 ? 0xffff : 0x0000, x > 4 ? 0xffff : 0x0000,
				       x  > 3 ? 0xffff : 0x0000, x > 2 ? 0xffff : 0x0000, x > 1 ? 0xffff : 0x0000, x > 0 ? 0xffff : 0x0000  );

  __m128i acc;
  for(int i = 0; i < rounds; i++) {
    const __m128i a = _mm_load_si128((__m128i*)checkBase);
    const __m128i b = _mm_load_si128((__m128i*)&target_ctx.reference);
    
    __m128i cmp = _mm_cmpeq_epi16(a, b);
    if ( i == rounds - 1 ){
      cmp = _mm_or_si128(cmp, mask);
    }
    acc = _mm_or_si128(cmp, acc);
  }
  return _mm_testz_si128(acc, acc) != 0;
}


extern "C" bool large_buf_has_taint_32_128(const uint16_t * checkBase, const int checkSize){
  const int x = ( checkSize / 2 ) % 4;
  const int rounds = x == 0 ? (checkSize / 8) : (checkSize / 8) + 1;
  
  // mask off leftovers
  const __m128i mask = _mm_setr_epi32( x == 0 ? 0xffffffff : 0x00000000, x > 3 ? 0xffffffff : 0x00000000,
				       x > 2  ? 0xffffffff : 0x00000000, x > 1 ? 0xffffffff : 0x00000000 );

  __m128i acc;
  for(int i = 0; i < rounds; i++) {
    const __m128i a = _mm_load_si128((__m128i*)checkBase);
    const __m128i b = _mm_load_si128((__m128i*)&target_ctx.reference);
    __m128i cmp = _mm_cmpeq_epi32(a, b);
    if ( i == rounds - 1 ){
      cmp = _mm_or_si128(cmp, mask);
    }
    acc = _mm_or_si128(cmp, acc);
  }
  return _mm_testz_si128(acc, acc) != 0;
}



extern "C" bool large_buf_has_taint_32_256(const uint16_t * checkBase, const int checkSize){
  const int x = ( checkSize / 2 ) % 8;
  const int rounds = x ? (checkSize / 8) : (checkSize / 8) + 1;
  
  // mask off leftovers
  const __m256i mask = _mm256_setr_epi32( x == 0 ? 0xffffffff : 0x00000000, x > 7  ? 0xffffffff : 0x00000000,
					  x > 6  ? 0xffffffff : 0x00000000, x > 5  ? 0xffffffff : 0x00000000,
					  x > 4  ? 0xffffffff : 0x00000000, x > 3  ? 0xffffffff : 0x00000000,
					  x > 2  ? 0xffffffff : 0x00000000, x > 1  ? 0xffffffff : 0x00000000 );

  __m256i acc;
  for(int i = 0; i < rounds; i++) {
    const __m256i a = _mm256_load_si256((__m256i*)checkBase);
    const __m256i b = _mm256_load_si256((__m256i*)&target_ctx.reference);
    
    __m256i cmp = _mm256_cmpeq_epi32(a, b);
    if ( i == rounds - 1 ){
      cmp = _mm256_or_si256(cmp, mask);
    }
    acc = _mm256_or_si256(cmp, acc);
  }
  return _mm256_testz_si256(acc, acc) != 0;
}


extern "C" bool large_buf_has_taint_16_256(const uint16_t * checkBase, const int checkSize){
  const int rounds = checkSize % 16 == 0 ? (checkSize / 16) : (checkSize / 16) + 1;
  const int x = checkSize % 16;

  // mask off leftovers
  const __m256i mask = _mm256_setr_epi32( x == 0 ? 0xffffffff : (x > 15 ? 0x0000ffff : 0x00000000),
					  x > 14 ? 0xffffffff : (x > 13 ? 0x0000ffff : 0x00000000),
					  x > 12 ? 0xffffffff : (x > 11 ? 0x0000ffff : 0x00000000),
					  x > 10 ? 0xffffffff : (x > 9  ? 0x0000ffff : 0x00000000),
					  x > 8  ? 0xffffffff : (x > 7  ? 0x0000ffff : 0x00000000),
					  x > 6  ? 0xffffffff : (x > 5  ? 0x0000ffff : 0x00000000),
					  x > 4  ? 0xffffffff : (x > 3  ? 0x0000ffff : 0x00000000),
					  x > 2  ? 0xffffffff : (x > 1  ? 0x0000ffff : 0x00000000) );

  __m256i acc;
  for(int i = 0; i < rounds; i++) {
    const __m256i a = _mm256_load_si256((__m256i*)checkBase);  // 32-byte aligned load
    const __m256i b = _mm256_load_si256((__m256i*)&target_ctx.reference);
    __m256i cmp = _mm256_cmpeq_epi16(a, b);
    if ( i == rounds - 1 ){
      cmp = _mm256_or_si256(cmp, mask);
    }
    acc = _mm256_or_si256(cmp, acc);
  }
  return _mm256_testz_si256(acc, acc) != 0;
}


template < int I > inline int
scan (uint64_t target, uint64_t pattern, uint64_t mask) { return ((target >> 8 * I) & mask) == pattern ? I : -1; }

template<typename... Ts>
inline int scan(int I, Ts... ts) {
  switch(I){
  case 0: return scan<0>(ts...); break;
  case 1: return scan<1>(ts...); break;
  case 2: return scan<2>(ts...); break;
  case 3: return scan<3>(ts...); break;
  case 4: return scan<4>(ts...); break;
  case 5: return scan<5>(ts...); break;
  case 6: return scan<6>(ts...); break;
  case 7: return scan<7>(ts...); break;
  }
}

template <> inline int scan < 8 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }


template < int I, int J > inline int
scanleft (uint64_t target, uint64_t pattern, uint64_t mask);


template < int I > inline int
scanlefthelper (uint64_t target, uint64_t pattern, uint64_t mask) {
    return scanleft < I, I+1 > (target, pattern, mask);    
}

template <> inline int
scanlefthelper <8> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }

template < > inline int scanleft < 1, 0 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 2, 1 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 3, 2 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 4, 3 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 5, 4 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 6, 5 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 7, 6 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 8, 7 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }
template < > inline int scanleft < 8, 8 > (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; }

// scan full pattern, then scan partials
template < int I, int J > inline int
scanleft (uint64_t target, uint64_t pattern, uint64_t mask)
{
  return I < J ?
    ( scan < I > (target, pattern, mask) >=0 ? scan < I > (target, pattern, mask) : scanleft < I + 1, J > (target, pattern, mask) ) : 
     I == J && J < 8 ? scanlefthelper < I > (target, pattern << 8*I >> 8*I, mask << 8*I >> 8*I) : -1;
}


template< int I, int J > inline int
scanright (uint64_t target, uint64_t pattern, uint64_t mask);

template <> inline int scanright <1,1> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };
template <> inline int scanright <2,2> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };
template <> inline int scanright <3,3> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };
template <> inline int scanright <4,4> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };
template <> inline int scanright <5,5> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };
template <> inline int scanright <6,6> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };
template <> inline int scanright <7,7> (uint64_t target, uint64_t pattern, uint64_t mask) { return -1; };

template< int I, int J > inline int
scanright (uint64_t target, uint64_t pattern, uint64_t mask)
{
    return scan< 0 >(target, pattern, mask) == 0 ? I : scanright < I+1, J >(target, pattern >> 8, mask >> 8);
}


void Executor::single_step_match(uint64_t cc[2]){
  if( scan<0>(cc[0], 0x0000000000f6314d, 0x0000000000ffffff) >= 0 ) { 
    Executor::tase_helper_write((uint64_t) &(target_ctx_gregs[GREG_EFL].u64), klee::ConstantExpr::create(0, Expr::Int64));
    target_ctx_gregs[GREG_RIP].u64 += 3; // xor %r14,%r14
    if( modelDebug ) {
      std::cout << "Killing Flags (xor)" << std::endl;
    }
    ex_state = EXECUTION_STATE::SKIP;
  } else if ( cc[0] == 0x4566363c751101c4 && scan< 0 >(cc[1], 0x0000850fff17380f, 0x0000ffffffffffff) >= 0 ) { 
    target_ctx_gregs[GREG_RIP].u64 += 32; // vpcmpeqw/ptest/movq/leaq/jne
    if( modelDebug ){
      std::cout << "Skipping eager instrumentation (A)..." << std::endl;
    }
    ex_state = EXECUTION_STATE::SKIP;	
  } else if ( scan< 0 >(cc[0], 0x9e00000000008948, 0xff0000000000ffff) >= 0 ) { 
    // movq/lahf/movl/shrxq/vpcmpeqw/ptest/movq/leaq/jne/sahf/movq
    target_ctx_gregs[GREG_RIP].u64 += 53;
	
    if( modelDebug ){                                                                                       
      std::cout << "Skipping eager instrumentation (B)..." << std::endl;                                    
    }
    ex_state = EXECUTION_STATE::SKIP;	  
  } else if ( scan< 0 >(cc[0], 0x000000000000009f, 0x00000000000000ff) == 0 ) {
    // lahf/movl/shrxq/vpcmpeqw/ptest/movq/leaq/jne/sahf
    target_ctx_gregs[GREG_RIP].u64 += 37;
	
    if( modelDebug ){                                                                                       
      std::cout << "Skipping eager instrumentation (C)..." << std::endl;                                    
    }
    ex_state = EXECUTION_STATE::SKIP;	  
  } else if ( scan< 0 >(cc[0], 0x0000000000308d44, 0x00000000000038fff4) == 0 ) {
    // leaq -> r14
    // get length: https://wiki.osdev.org/X86-64_Instruction_Encoding#64-bit_addressing
    //        MOD
    //  B.RM       0.000-0.011(0-3) 0.100  0.101  0.110-1.011(6-11) 1.100 1.101  1.110-1.111(14-15)
    //         00        3           SIB      7         3             SIB     7        3
    //         01        4           SIB      4         4             SIB     4        4
    //         10        7           SIB      7         7             SIB     7        7
    // SIB -> check base. If mod == 00 and base is 0.101 or 1.101 then size = 8, o.w. size = 4. If mod == 10, size = 8. If mod == 01, size = 5.
	
    auto brm = ((0x00000000000001 & cc[0]) << 3) | ((0x0000000000070000 & cc[0]) >> 16);
    auto mod = (0x0000000000c00000 & cc[0]) >> 22;
    auto size = 0;
	
    if ( brm == 4 || brm == 12 ) {
      auto base = (0x00000007000000 & cc[0]) >> 6*4;
      size = mod == 0 ? ( base == 5 ? 8 : 4 ) : ( mod == 2 ? 8 : 5 );
    } else if ( brm == 5 || brm == 13 ) {
      size = mod < 1 ? 7 : mod < 2 ? 4 : 7;
    } else {
      size = mod < 1 ? 3 : mod < 2 ? 4 : 7;
    }

    uint64_t c = 0;

    if ( size < 8 ) {
      c = (cc[0] >> ((size)*8)) | (cc[1] << ((8-size)*8)); // skip past current instr
    } else {
      c = cc[1];
    }
	
    // check for potential next instrs, take earliest appearance
    auto shr = scanleft< 0, 7 >(c, 0x0000000000eed149, 0x0000000000ffffff); // shrq ( eflags dead and rax dead )
    auto mov = scanleft< 0, 7 >(c, 0x0000000000008948, 0x000000000000ffff); // movq ( eflags live and rax live )
    auto lah = scanleft< 0, 7 >(c, 0x000000000000009f, 0x00000000000000ff); // lahf ( eflags live only )
    shr = shr >= 0 ? shr : 8;
    mov = mov >= 0 ? mov : 8;
    lah = lah >= 0 ? lah : 8;

    auto update = shr < mov ? ( shr < lah ? 35 : 37 ) : ( mov < lah ? 51 : 37 );
    target_ctx_gregs[GREG_RIP].u64 += size + update;

    if ( modelDebug ) {
      std::cout << "Skipping eager instrumentation (D[" << size << "][" << shr << "][" << mov << "][" << lah << "])... " << std::hex << c << std::endl;
    }
    ex_state = EXECUTION_STATE::SKIP;	  
  } else {
    ex_state = EXECUTION_STATE::INTERP;
  }
}


  
void Executor::klee_interp_internal() {
  run_interp_traps++;
  double model_start_time;
  
  while (true) {
    run_interp_insts++;
    interpCtr++;
    if (taseDebug)
      printDebugInterpHeader();

    uint64_t rip = target_ctx_gregs[GREG_RIP].u64;
    uint64_t rip_init = rip;
    
    auto mod = fnModelMap.find(rip);
    void (klee::Executor::*fp)() = mod == fnModelMap.end() ? NULL : mod->second;
    
    if (modelDebug) {
      std::cout << "RIP at top of klee_interp_internal loop is " << std::hex << rip << std::dec << "\n";
      std::cout << "interp_state: " << ex_state.print_istate() << "\n";
      std::cout << "Model found: " << (mod == fnModelMap.end() ? "false" : "true") << std::endl;
    }

    // in the case we advance within the while loop to a model
    if( ex_state != EXECUTION_STATE::MODEL && ex_state != EXECUTION_STATE::PROHIB && mod != fnModelMap.end() ){
      if( modelDebug ) {
	std::cout << "Updating interp_state to " << ex_state.print_istate() << std::endl;
      }
      
      ex_state = EXECUTION_STATE::MODEL;
    }

    uint64_t cc[2] = {*(uint64_t*)target_ctx_gregs[GREG_RIP].u64, *(((uint64_t*)target_ctx_gregs[GREG_RIP].u64)+1)};
    
    switch( ex_state.istate ) {
    case EXECUTION_STATE::PROHIB_RESUME:
      if ( ex_state.is_concrete() && tran_max == 0 && target_ctx_gregs[GREG_RIP].u64 == init_trap_RIP ) {
	if (taseDebug) {
	  std::cout << "Repeated faults detected for prohib function" << std::endl;
	}
	ex_state = EXECUTION_STATE::PROHIB_FAULT;
      }
    case EXECUTION_STATE::RESUME:
      return;

    case EXECUTION_STATE::MODEL:
      if( measureTime )
	model_start_time = util::getWallTime();
      
      (this->*fp)();

      if( measureTime )
	run_model_time = util::getWallTime() - model_start_time;
      
      ex_state = ex_state.is_mixedMode() ? EXECUTION_STATE::RESUME : EXECUTION_STATE::INTERP;
      break;

    case EXECUTION_STATE::BOUNCEBACK:
      assert(false && "BOUNCEBACK state found in klee_interp_internal!");
      break;

    case EXECUTION_STATE::SKIP:
    case EXECUTION_STATE::FAULT:
    case EXECUTION_STATE::PROHIB_FAULT:
    case EXECUTION_STATE::PROHIB:
    case EXECUTION_STATE::INTERP:
      if( taseDebug ) {
	std::cout << "Checking for skippable instrs" << std::endl;
      }

      if( ex_state.is_singleStepping() ) {
	single_step_match(cc);
        if ( ex_state == EXECUTION_STATE::SKIP ) {
	  break;
	}
      } else {
	tryKillFlags();
	
	if( scan< 0 >(cc[0], 0x00000000053d8d4c, 0x00ffffffffffffff) >= 0 ){
	  target_ctx_gregs[GREG_RIP].u64 += trap_off; // lea/jmpq
	  if( modelDebug ) {
	    std::cout << "Skipping LEA and jmp..." << std::endl;
	  }
	  ex_state = EXECUTION_STATE::SKIP;
	  break;
	}
      }
      runCoreInterpreter(target_ctx_gregs);
      ex_state = EXECUTION_STATE::INTERP;
      break;
    }
      


    // fast check here is a subset of is_concrete's check and it's run already
    if( ex_state.is_concrete() || tase_buf_has_taint((void *) &(target_ctx_gregs[GREG_RIP].u64), 8) ) { // fast check for potential taint
      ref<Expr> RIPExpr = tase_helper_read((uint64_t) &(target_ctx_gregs[GREG_RIP].u64), 8); // full/slow check
      if (!(isa<ConstantExpr>(RIPExpr))) {
	if ( taseDebug ) {
	  std::cout << "Forking on possible RIP values in klee_interp_internal" << std::endl;
	}
        forkOnPossibleRIPValues(RIPExpr, rip_init);

        if ( taseDebug ) {
          ref<Expr> FinalRIPExpr = target_ctx_gregs_OS->read(GREG_RIP * 8, Expr::Int64);
          if ( !(isa<ConstantExpr>(FinalRIPExpr)) ) {
            std::cout << "ERROR: Failed to concretize RIP \n" << std::endl;
            std::exit(EXIT_FAILURE);
          }
        }
      }
    }

    ex_state.update();
  } // end while
  

  static int numReturns = 0;
  numReturns++;

  if (taseDebug) {
    std::cout << "Returning to native execution for time " << numReturns << "\n";
    std::cout << "Prior to return to native execution, ctx is ..." << "\n";
    printCtx();
    std::cout << "--------RETURNING TO TARGET --- ------------" << std::endl;
  }
  
  return;
}


void Executor::tryKillFlags() {
  if (killFlags) {
    uint64_t rip = target_ctx_gregs[GREG_RIP].u64;
    if ( ex_state.is_txn_start() ) {
      if (cartridgeHasFlagsDead(rip)) {
	if (taseDebug) {
	  printf("Killing flags \n");
	}
	uint64_t zeroFlags = 0x2; // reserved flag bit 1 is always set
	ref<ConstantExpr> zeroExpr = ConstantExpr::create(zeroFlags, Expr::Int64);
	tase_helper_write((uint64_t) &target_ctx_gregs[GREG_EFL], zeroExpr);
      }
    }
  }
}

void Executor::runCoreInterpreter(tase_greg_t * gregs) {
  run_bb_count++;
  double T0;
  if (measureTime) {
    T0 = util::getWallTime();
  }
  KFunction * interpFn = findInterpFunction (gregs, kmodule);

  //We have to manually push a frame on for the function we'll be
  //interpreting through.  At this point, no other frames should exist
  // on klee's interpretation "stack".

  GlobalExecutionStatePtr->pushFrameTASE(0,interpFn);

  GlobalExecutionStatePtr->pc = interpFn->instructions;
  GlobalExecutionStatePtr->prevPC = GlobalExecutionStatePtr->pc;
  
  //getArgumentCell(GlobalExecutionStatePtr,interpFn,0).value = arguments[0];
  (GlobalExecutionStatePtr->stack.back().locals[interpFn->getArgRegister(0)]).value = arguments[0];
  if (measureTime) {
    run_tmp_1_time += (util::getWallTime() - T0);
  }
  //bindArgument(interpFn, 0, *GlobalExecutionStatePtr, arguments[0]);

  
  run(*GlobalExecutionStatePtr);
  
  if (measureTime) {
    run_core_interp_time += (util::getWallTime() - T0);
  }
  
}


//This struct is to help the solver for basic blocks with only
//two possible successors (e.g., blocks ending in "jb", "je", etc).

typedef struct cartridgeDestHint {
  uint64_t blockTop;
  uint64_t dest1;
  uint64_t dest2;
} cartridgeSuccessorInfo;

extern std::map<uint64_t, cartridgeSuccessorInfo> knownCartridgeDests;

  
//Take an Expr and find all the possible concrete solutions.
//Hopefully there's a better builtin function in klee that we can
//use, but if not this should do the trick.  Intended to be used
//to help us get all possible concrete values of RIP (has dependency on RIP).

void Executor::forkOnPossibleRIPValues (ref <Expr> inputExpr, uint64_t initRIP) {

  //Fast Path -- Only two possible destinations when exiting basic block;
  std::map<uint64_t, cartridgeSuccessorInfo>::iterator it;
 
  it = knownCartridgeDests.find(initRIP);
  if (it != knownCartridgeDests.end()) {

    uint64_t d1 = it->second.dest1;
    uint64_t d2 = it->second.dest2;

    Solver::Validity rtmp;
    bool s = false;

    double t0 = util::getWallTime();
    s = solver->evaluate(*GlobalExecutionStatePtr, EqExpr::create(inputExpr, ConstantExpr::create(d1, Expr::Int64)), rtmp);
    
    if (!s) {
      printf("FATAL ERROR: Solver evaluate call failed in forkOnPossibleRIPValues! \n");  
      std::cout.flush();
      //      worker_exit(EXIT_FAILURE);
      exit(1);
    }

    double t1 = util::getWallTime();

    if (!noLog && taseDebug) {
      printf("Solver (forking) calls took %lf seconds in knownCartDests case \n", t1-t0);
    }
    run_solver_time += (t1-t0);
        
    if (rtmp == Solver::True) {
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RIP], ConstantExpr::create(d1,Expr::Int64));
      return;
    } else if (rtmp == Solver::False) {
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RIP], ConstantExpr::create(d2,Expr::Int64));
      return;
    } else {
      printf("Prior to fork, time since start is %lf \n", util::getWallTime() - target_start_time);
      //      int isTrueChild = tase_fork(getpid(), initRIP); //Returns 0 for false branch, 1 for true.  Not intuitive
      int isTrueChild = worker_fork(Stopped, Running);
      if (isTrueChild == 1) {
	addConstraint(*GlobalExecutionStatePtr,  EqExpr::create(inputExpr, ConstantExpr::create(d1, Expr::Int64)));
	tase_helper_write( (uint64_t) &target_ctx_gregs[GREG_RIP], ConstantExpr::create(d1,Expr::Int64));
	return;	
      } else {
	addConstraint(*GlobalExecutionStatePtr,  EqExpr::create(inputExpr, ConstantExpr::create(d2, Expr::Int64)));
	tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RIP], ConstantExpr::create(d2,Expr::Int64));
	return;	
      } 
    }
  } else {
    
    solver_start_time = util::getWallTime();
    
    ref <Expr> uniqueRIPExpr  = toUnique(*GlobalExecutionStatePtr,inputExpr);
    solver_end_time = util::getWallTime();
    solver_diff_time = solver_end_time - solver_start_time;

    if (!noLog) {
      printf("Elapsed solver time (RIP toUnique) is %lf at interpCtr %lu \n", solver_diff_time, interpCtr);
    }
    run_solver_time += solver_diff_time;
  
    if (isa<ConstantExpr> (uniqueRIPExpr)) {
      if (!noLog) {
	printf("Only one valid value for RIP \n");
      }
      tase_helper_write((uint64_t) &target_ctx_gregs[GREG_RIP], uniqueRIPExpr);
      return;
    
    } else {

      int maxSolutions = 2; //Completely arbitrary.  Should not be more than 2 for our use cases in TASE
      //or we're in trouble anyway.
    
      int numSolutions = 0;  
      while (true) {
	ref<ConstantExpr> solution;
	numSolutions++;
      
	if (numSolutions > maxSolutions) {
	  printf("IMPORTANT: control debug: Found too many symbolic values for next instruction after 0x%lx \n ", initRIP);
	  std::cout.flush();
	  //	  worker_exit(EXIT_FAILURE);
	  exit(1);
	}
      
	solver_start_time = util::getWallTime();
      
	bool success = solver->getValue(*GlobalExecutionStatePtr, inputExpr, solution);
	solver_end_time = util::getWallTime();
      
	solver_diff_time = solver_end_time - solver_start_time;
	printf("Elapsed solver time (forking) is %lf at interpCtr %lu \n", solver_diff_time, interpCtr);
      
	run_solver_time += solver_diff_time;
      
	if (!success) {
	  printf("ERROR: couldn't get initial value in forkOnPossibleRIPValues \n");  
	  std::cout.flush();
	  //	  worker_exit(EXIT_FAILURE);
	  exit(1);
	}
      
	//	int isTrueChild = tase_fork(getpid(), initRIP); //Returns 0 for false branch, 1 for true.  Not intuitive
	int isTrueChild = worker_fork(Stopped, Running);

	//ABH: Todo -- roll this back and support > 2 symbolic dests for things like indirect jumps
	if (isTrueChild == 0) { //Rule out latest solution and see if more exist
	  ref<Expr> notEqualsSolution = NotExpr::create(EqExpr::create(inputExpr,solution));
	  if (klee::ConstantExpr *CE = dyn_cast<klee::ConstantExpr>(notEqualsSolution)) {
	    if (CE->isFalse()) {
	      printf("IMPORTANT: forked child %d is not exploring a feasible path \n", getpid());
	      fflush(stdout);
	      //	      worker_exit();
	      exit(1);
	    }
	  }
	
	
	  addConstraint(*GlobalExecutionStatePtr, notEqualsSolution);

	  solver_start_time = util::getWallTime();
	  success = solver->getValue(*GlobalExecutionStatePtr, inputExpr, solution);
	  solver_end_time = util::getWallTime();
    
	  solver_diff_time = solver_end_time - solver_start_time;
	  run_solver_time += solver_diff_time;

	  if (!noLog) {
	    printf("Elapsed solver time (fork - path constraint) is %lf at interpCtr %lu \n", solver_diff_time, interpCtr);
	  }
      
	  if (!success) {
	    printf("ERROR: couldn't get RIP value in forkOnPossibleRIPValues for false child \n");
	    std::cout.flush();
	    //	    worker_exit(EXIT_FAILURE);
	    exit(1);
	  }
	  if (!noLog) {
	    printf("IMPORTANT: control debug: Found dest RIP 0x%lx on false branch in forkOnRip from RIP 0x%lx with pid %d \n", (uint64_t) solution->getZExtValue(), initRIP, getpid());
	  }
	  addConstraint(*GlobalExecutionStatePtr, EqExpr::create(inputExpr, solution));
	  target_ctx_gregs_OS->write(GREG_RIP*8, solution);
	  break;

	} else { // Take the concrete value of solution and explore that path.

	  if (!noLog) {
	    printf("IMPORTANT: control debug: Found dest RIP 0x%lx on true branch in forkOnRip from RIP 0x%lx with pid %d \n", (uint64_t) solution->getZExtValue(), initRIP, getpid());
	  }
	  addConstraint(*GlobalExecutionStatePtr, EqExpr::create(inputExpr, solution));
	  target_ctx_gregs_OS->write(GREG_RIP*8, solution);
	  break;
	}
      }
    }
  }
}

void Executor::printCtx() {
  printf("R8   : 0x%lx \n", target_ctx_gregs[GREG_R8].u64);
  printf("R9   : 0x%lx \n", target_ctx_gregs[GREG_R9].u64);
  printf("R10  : 0x%lx \n", target_ctx_gregs[GREG_R10].u64);
  printf("R11  : 0x%lx \n", target_ctx_gregs[GREG_R11].u64);
  printf("R12  : 0x%lx \n", target_ctx_gregs[GREG_R12].u64);
  printf("R13  : 0x%lx \n", target_ctx_gregs[GREG_R13].u64);
  printf("R14  : 0x%lx \n", target_ctx_gregs[GREG_R14].u64);
  printf("R15  : 0x%lx \n", target_ctx_gregs[GREG_R15].u64);
  printf("RDI  : 0x%lx \n", target_ctx_gregs[GREG_RDI].u64);
  printf("RSI  : 0x%lx \n", target_ctx_gregs[GREG_RSI].u64);
  printf("RBP  : 0x%lx \n", target_ctx_gregs[GREG_RBP].u64);
  printf("RBX  : 0x%lx \n", target_ctx_gregs[GREG_RBX].u64);
  printf("RDX  : 0x%lx \n", target_ctx_gregs[GREG_RDX].u64);
  printf("RAX  : 0x%lx \n", target_ctx_gregs[GREG_RAX].u64);
  printf("RCX  : 0x%lx \n", target_ctx_gregs[GREG_RCX].u64);
  printf("RSP  : 0x%lx \n", target_ctx_gregs[GREG_RSP].u64);
  printf("RIP  : 0x%lx \n", target_ctx_gregs[GREG_RIP].u64);
  printf("EFL  : 0x%lx [", target_ctx_gregs[GREG_EFL].u64);

  auto x = target_ctx_gregs[GREG_EFL].u64;
  if( x & 0x1 )
    printf(" CF");

  if( x & 0x4 )
    printf(" PF");

  if( x & 0x40 )
    printf(" ZF");

  if( x & 0x80 )
    printf(" SF ");
  
  printf(" ] \n");

  if( simdType == XMM ) {
    printf("XMM14 (acc) : [");
    printf(" 0x%lx,", target_ctx.accumulator.qword[0]);
    printf(" 0x%lx,", target_ctx.accumulator.qword[1]);
    printf(" ]\n");
    
    printf("XMM15 (data) : [");
    printf(" 0x%lx,", target_ctx.data.qword[0]);
    printf(" 0x%lx,", target_ctx.data.qword[1]);
    printf(" ]\n");
  } else {
    printf("YMM14 (acc)  : [");
    printf(" 0x%lx,", target_ctx.accumulator.qword[0]);
    printf(" 0x%lx,", target_ctx.accumulator.qword[1]);
    printf(" 0x%lx,", target_ctx.accumulator.qword[2]);
    printf(" 0x%lx", target_ctx.accumulator.qword[3]);
    printf(" ]\n");
    
    printf("YMM15 (data)  : [");
    printf(" 0x%lx,", target_ctx.data.qword[0]);
    printf(" 0x%lx,", target_ctx.data.qword[1]);
    printf(" 0x%lx,", target_ctx.data.qword[2]);
    printf(" 0x%lx", target_ctx.data.qword[3]);
    printf(" ]\n");
  }
  
  return;
}


void Executor::initializeInterpretationStructures (Function *f) {
  ufmanager = new UFManager();
  printf("INITIALIZING INTERPRETATION STRUCTURES \n");
  fflush(stdout);
  
  GlobalExecutorPtr = this;
  GlobalExecutionStatePtr = new ExecutionState(kmodule->functionMap[f]);

  uint64_t regAddr = (uint64_t) target_ctx_gregs;
  ref<ConstantExpr> regExpr = ConstantExpr::create(regAddr, Context::get().getPointerWidth());  
  arguments.push_back(regExpr);
  
  initializeGlobals(*GlobalExecutionStatePtr);
  //Set up the KLEE memory object for the stack, and back the concrete store with the actual stack.
  //Need to be careful here.  The buffer we allocate for the stack is char [X] target_stack. It
  // starts at address StackBase and covers up to StackBase + sizeof(target_stack) -1.

  
  uint64_t stackBase = (uint64_t) &target_ctx.target_stack;
  uint64_t stackSize = STACK_SIZE;
  tase_map_buf((uint64_t) stackBase, stackSize, "stack");
  std::cout << "Stack: " << std::hex << stackBase << " to " << (stackBase + stackSize + 1) << std::dec << std::endl;
  
  target_ctx_gregs_MO = addExternalObject(*GlobalExecutionStatePtr, (void *) target_ctx_gregs, TASE_NGREG * TASE_GREG_SIZE, false );
  const ObjectState *targetCtxOS = GlobalExecutionStatePtr->addressSpace.findObject(target_ctx_gregs_MO);
  target_ctx_gregs_OS = GlobalExecutionStatePtr->addressSpace.getWriteable(target_ctx_gregs_MO,targetCtxOS);
  target_ctx_gregs_OS->concreteStore = (uint8_t *) target_ctx_gregs;

  /*target_ctx_xmms_MO = addExternalObject(*GlobalExecutionStatePtr, (void*) target_ctx.xmm, 8*XMMREG_SIZE, false);
  const ObjectState *targetCtxXOS = GlobalExecutionStatePtr->addressSpace.findObject(target_ctx_xmm_MO);
  target_ctx_xmms_OS = GlobalExecutionStatePtr->addressSpace.getWritable(target_ctx_xmms_MO, targetCtxXOS);
  target_ctx_xmms_OS->concreteStore = (uint8_t *) target_ctx_xmms;
  */
  //Map in read-only globals
  //Todo -- find a less hacky way of getting the exact size of the .rodata section

  rodata_base_ptr = (void *) (&_IO_stdin_used);
  rodata_size = (uint64_t) ((uint64_t) (&__GNU_EH_FRAME_HDR) - (uint64_t) (&_IO_stdin_used)) ;

  rodata_size += (0x2949c + 0x2949c); //Hack to map in eh_frame_hdr and eh_frame also
  tase_map_buf((uint64_t) rodata_base_ptr, rodata_size, "rodata");

  //Map in special stdin libc symbol
  //tase_map_buf((uint64_t) &stdin, 8);
  
  //Map in special stdout libc symbol
  //tase_map_buf((uint64_t) &stdout, 8);
  //tase_map_buf((uint64_t) stdout, sizeof(FILE));
  //tase_map(stdout);
  //Map in special stderr libc symbol
  //tase_map_buf((uint64_t) &stderr, 8);
  //printf("stream globals: out/err/in  %lx/%lx/%lx", &stdout, &stderr, &stdin);
  //Map in initialized and uninitialized non-read only globals into klee from .vars file.
  std::string varsFileLocation = "./" + project + ".vars";
  std::ifstream externals(varsFileLocation);
  if(externals){
    std::string line;
    uint64_t addrVal;
    uint64_t sizeVal;
    int lines = 0;
    while(std::getline(externals, line)){
      std::istringstream ss(line);
      ss >> std::hex >> addrVal >> sizeVal;
      if(!ss){
        std::cout << "Error reading externals file within initializeInterpretationStructures() at line " << lines << std::endl;
	std::cout << '"' << line << '"' << std::endl;
	//        worker_exit(EXIT_FAILURE);
	exit(1);
      }
      if((uint64_t) addrVal != (uint64_t) &target_ctx_gregs){
        //if((sizeVal %2) == 1){
        if((sizeVal % 8) != 0){
          sizeVal += 8 - (sizeVal % 8);
          //++sizeVal;
          printf("rounding up sizeval to even number - %lu \n", sizeVal);
        }
        tase_map_buf(addrVal, sizeVal, "global");
        std::cout << "Global: " << std::hex << addrVal << " size " << std::dec << sizeVal << std::endl;
      }
      lines++;
    }
  } else {
    std::cout << "Error reading externals file within initializeInterpretationStructures()" << std::endl;
    //    worker_exit(EXIT_FAILURE);
    exit(1);
  }

  //Todo -- De-hackify this environ variable mapping
  char ** environPtr = environ;
  char * envStr = environ[0];
  size_t len = 0;
  char * latestEnvStr = NULL;
  
  while (envStr != NULL) {
    
    uint32_t size = strlen(envStr);
    printf("Found env var at 0x%lx with size 0x%x \n", (uint64_t) envStr, size);

    envStr = *(environPtr++);
    if (envStr != NULL) {
      len = strlen(envStr);
      latestEnvStr = envStr;
    }
  }

  uint64_t baseEnvAddr =  (uint64_t) environ[0];
  uint64_t endEnvAddr = (uint64_t) latestEnvStr + len + 1;
  uint64_t envSize = endEnvAddr - baseEnvAddr;

  if (envSize % 2 == 1)
    envSize++;

  tase_map_buf(baseEnvAddr, envSize, "env");
  
  //Add mappings for stderr and stdout
  //Todo -- remove dependency on _edata location
  //printf("Mapping edata at 0x%lx \n", (uint64_t) &edata);
  tase_map_buf((uint64_t) &edata, 16, "edata");

  //Get rid of the dummy function used for initialization
  GlobalExecutionStatePtr->popFrame();
  processTree = new PTree(GlobalExecutionStatePtr);
  GlobalExecutionStatePtr->ptreeNode = processTree->root;
  bindModuleConstants(); //Moved from "run"
  loadFnModelMap();
  
  FILE * stats = fopen("/proc/self/statm", "r");
  if (stats != NULL ) {
    printf("Couldn't open statm \n");
    fflush(stdout);
  } else {
    printf("Opened statm \n");
    fflush(stdout);
    uint64_t r1, r2, r3, r4, r5, r6, r7;
    fscanf (stats, "%lu %lu %lu %lu %lu %lu %lu", &r1, &r2, &r3, &r4, &r5, &r6, &r7);
    printf("STATM 3:  %lu %lu %lu %lu %lu %lu %lu \n", r1, r2, r3, r4, r5, r6, r7);
    fclose(stats);
    fflush(stdout);
  }
}
				   

void Executor::runFunctionAsMain(Function *f,
				 int argc,
				 char **argv,
				 char **envp) {
  std::vector<ref<Expr> > arguments;

  // force deterministic initialization of memory objects
  srand(1);
  srandom(1);
  
  MemoryObject *argvMO = 0;

  // In order to make uclibc happy and be closer to what the system is
  // doing we lay out the environments at the end of the argv array
  // (both are terminated by a null). There is also a final terminating
  // null that uclibc seems to expect, possibly the ELF header?

  int envc;
  for (envc=0; envp[envc]; ++envc) ;

  unsigned NumPtrBytes = Context::get().getPointerWidth() / 8;
  KFunction *kf = kmodule->functionMap[f];
  assert(kf);
  Function::arg_iterator ai = f->arg_begin(), ae = f->arg_end();
  if (ai!=ae) {
    arguments.push_back(ConstantExpr::alloc(argc, Expr::Int32));
    if (++ai!=ae) {
      Instruction *first = &*(f->begin()->begin());
      argvMO =
         memory->allocate((argc + 1 + envc + 1 + 1) * NumPtrBytes,
                           /*isLocal=*/false, /*isGlobal=*/true,
                           /*allocSite=*/first, /*alignment=*/8);

      if (!argvMO)
        klee_error("Could not allocate memory for function arguments");

      arguments.push_back(argvMO->getBaseExpr());

      if (++ai!=ae) {
        uint64_t envp_start = argvMO->address + (argc+1)*NumPtrBytes;
        arguments.push_back(Expr::createPointer(envp_start));

        if (++ai!=ae)
          klee_error("invalid main function (expect 0-3 arguments)");
      }
    }
  }

  ExecutionState *state = new ExecutionState(kmodule->functionMap[f]);
  
  if (pathWriter) 
    state->pathOS = pathWriter->open();
  if (symPathWriter) 
    state->symPathOS = symPathWriter->open();

  if (statsTracker)
    statsTracker->framePushed(*state, 0);

  assert(arguments.size() == f->arg_size() && "wrong number of arguments");
  for (unsigned i = 0, e = f->arg_size(); i != e; ++i)
    bindArgument(kf, i, *state, arguments[i]);

  if (argvMO) {
    ObjectState *argvOS = bindObjectInState(*state, argvMO, false);

    for (int i=0; i<argc+1+envc+1+1; i++) {
      if (i==argc || i>=argc+1+envc) {
        // Write NULL pointer
        argvOS->write(i * NumPtrBytes, Expr::createPointer(0));
      } else {
        char *s = i<argc ? argv[i] : envp[i-(argc+1)];
        int j, len = strlen(s);

        MemoryObject *arg =
            memory->allocate(len + 1, /*isLocal=*/false, /*isGlobal=*/true,
                             /*allocSite=*/state->pc->inst, /*alignment=*/8);
        if (!arg)
          klee_error("Could not allocate memory for function arguments");
        ObjectState *os = bindObjectInState(*state, arg, false);
        for (j=0; j<len+1; j++)
          os->write8(j, s[j]);

        // Write pointer to newly allocated and initialised argv/envp c-string
        argvOS->write(i * NumPtrBytes, arg->getBaseExpr());
      }
    }
  }
  
  initializeGlobals(*state);

  processTree = new PTree(state);
  state->ptreeNode = processTree->root;
  run(*state);
  delete processTree;
  processTree = 0;

  // hack to clear memory objects
  delete memory;
  memory = new MemoryManager(NULL);

  globalObjects.clear();
  globalAddresses.clear();

  if (statsTracker)
    statsTracker->done();
}

unsigned Executor::getPathStreamID(const ExecutionState &state) {
  assert(pathWriter);
  return state.pathOS.getID();
}

unsigned Executor::getSymbolicPathStreamID(const ExecutionState &state) {
  assert(symPathWriter);
  return state.symPathOS.getID();
}

void Executor::getConstraintLog(const ExecutionState &state, std::string &res,
                                Interpreter::LogType logFormat) {

  switch (logFormat) {
  case STP: {
    Query query(state.constraints, ConstantExpr::alloc(0, Expr::Bool));
    char *log = solver->getConstraintLog(query);
    res = std::string(log);
    free(log);
  } break;

  case KQUERY: {
    std::string Str;
    llvm::raw_string_ostream info(Str);
    ExprPPrinter::printConstraints(info, state.constraints);
    res = info.str();
  } break;

  case SMTLIB2: {
    std::string Str;
    llvm::raw_string_ostream info(Str);
    ExprSMTLIBPrinter printer;
    printer.setOutput(info);
    Query query(state.constraints, ConstantExpr::alloc(0, Expr::Bool));
    printer.setQuery(query);
    printer.generateOutput();
    res = info.str();
  } break;

  default:
    klee_warning("Executor::getConstraintLog() : Log format not supported!");
  }
}

bool Executor::getSymbolicSolution(const ExecutionState &state,
                                   std::vector< 
                                   std::pair<std::string,
                                   std::vector<unsigned char> > >
                                   &res) {
  solver->setTimeout(coreSolverTimeout);

  ExecutionState tmp(state);

  // Go through each byte in every test case and attempt to restrict
  // it to the constraints contained in cexPreferences.  (Note:
  // usually this means trying to make it an ASCII character (0-127)
  // and therefore human readable. It is also possible to customize
  // the preferred constraints.  See test/Features/PreferCex.c for
  // an example) While this process can be very expensive, it can
  // also make understanding individual test cases much easier.
  for (unsigned i = 0; i != state.symbolics.size(); ++i) {
    const MemoryObject *mo = state.symbolics[i].first;
    std::vector< ref<Expr> >::const_iterator pi = 
      mo->cexPreferences.begin(), pie = mo->cexPreferences.end();
    for (; pi != pie; ++pi) {
      bool mustBeTrue;
      // Attempt to bound byte to constraints held in cexPreferences
      bool success = solver->mustBeTrue(tmp, Expr::createIsZero(*pi),

					mustBeTrue);
      // If it isn't possible to constrain this particular byte in the desired
      // way (normally this would mean that the byte can't be constrained to
      // be between 0 and 127 without making the entire constraint list UNSAT)
      // then just continue on to the next byte.
      if (!success) break;
      // If the particular constraint operated on in this iteration through
      // the loop isn't implied then add it to the list of constraints.
      if (!mustBeTrue) tmp.addConstraint(*pi);
    }
    if (pi!=pie) break;
  }

  std::vector< std::vector<unsigned char> > values;
  std::vector<const Array*> objects;
  for (unsigned i = 0; i != state.symbolics.size(); ++i)
    objects.push_back(state.symbolics[i].second);
  bool success = solver->getInitialValues(tmp, objects, values);
  solver->setTimeout(0);
  if (!success) {
    klee_warning("unable to compute initial values (invalid constraints?)!");
    ExprPPrinter::printQuery(llvm::errs(), state.constraints,
                             ConstantExpr::alloc(0, Expr::Bool));
    return false;
  }
  
  for (unsigned i = 0; i != state.symbolics.size(); ++i)
    res.push_back(std::make_pair(state.symbolics[i].first->name, values[i]));
  return true;
}

void Executor::getCoveredLines(const ExecutionState &state,
                               std::map<const std::string*, std::set<unsigned> > &res) {
  res = state.coveredLines;
}

void Executor::doImpliedValueConcretization(ExecutionState &state,
                                            ref<Expr> e,
                                            ref<ConstantExpr> value) {
  abort(); // FIXME: Broken until we sort out how to do the write back.

  if (DebugCheckForImpliedValues)
    ImpliedValue::checkForImpliedValues(solver->solver, e, value);

  ImpliedValueList results;
  ImpliedValue::getImpliedValues(e, value, results);
  for (ImpliedValueList::iterator it = results.begin(), ie = results.end();
       it != ie; ++it) {
    ReadExpr *re = it->first.get();
    
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(re->index)) {
      // FIXME: This is the sole remaining usage of the Array object
      // variable. Kill me.
      const MemoryObject *mo = 0; //re->updates.root->object;
      const ObjectState *os = state.addressSpace.findObject(mo);

      if (!os) {
        // object has been free'd, no need to concretize (although as
        // in other cases we would like to concretize the outstanding
        // reads, but we have no facility for that yet)
      } else {
        assert(!os->readOnly && 
               "not possible? read only object with static read?");
        ObjectState *wos = state.addressSpace.getWriteable(mo, os);
        wos->write(CE, it->second);
      }
    }
  }
}

Expr::Width Executor::getWidthForLLVMType(llvm::Type *type) const {
  return kmodule->targetData->getTypeSizeInBits(type);
}

size_t Executor::getAllocationAlignment(const llvm::Value *allocSite) const {
  // FIXME: 8 was the previous default. We shouldn't hard code this
  // and should fetch the default from elsewhere.
  const size_t forcedAlignment = 8;
  size_t alignment = 0;
  llvm::Type *type = NULL;
  std::string allocationSiteName(allocSite->getName().str());
  if (const GlobalValue *GV = dyn_cast<GlobalValue>(allocSite)) {
    alignment = GV->getAlignment();
    if (const GlobalVariable *globalVar = dyn_cast<GlobalVariable>(GV)) {
      // All GlobalVariables's have pointer type
      llvm::PointerType *ptrType =
          dyn_cast<llvm::PointerType>(globalVar->getType());
      assert(ptrType && "globalVar's type is not a pointer");
      type = ptrType->getElementType();
    } else {
      type = GV->getType();
    }
  } else if (const AllocaInst *AI = dyn_cast<AllocaInst>(allocSite)) {
    alignment = AI->getAlignment();
    type = AI->getAllocatedType();
  } else if (isa<InvokeInst>(allocSite) || isa<CallInst>(allocSite)) {
    // FIXME: Model the semantics of the call to use the right alignment
    llvm::Value *allocSiteNonConst = const_cast<llvm::Value *>(allocSite);
    const CallSite cs = (isa<InvokeInst>(allocSiteNonConst)
                             ? CallSite(cast<InvokeInst>(allocSiteNonConst))
                             : CallSite(cast<CallInst>(allocSiteNonConst)));
    llvm::Function *fn =
        klee::getDirectCallTarget(cs, /*moduleIsFullyLinked=*/true);
    if (fn)
      allocationSiteName = fn->getName().str();

    klee_warning_once(fn != NULL ? fn : allocSite,
                      "Alignment of memory from call \"%s\" is not "
                      "modelled. Using alignment of %zu.",
                      allocationSiteName.c_str(), forcedAlignment);
    alignment = forcedAlignment;
  } else {
    llvm_unreachable("Unhandled allocation site");
  }

  if (alignment == 0) {
    assert(type != NULL);
    // No specified alignment. Get the alignment for the type.
    if (type->isSized()) {
      alignment = kmodule->targetData->getPrefTypeAlignment(type);
    } else {
      klee_warning_once(allocSite, "Cannot determine memory alignment for "
                                   "\"%s\". Using alignment of %zu.",
                        allocationSiteName.c_str(), forcedAlignment);
      alignment = forcedAlignment;
    }
  }

  // Currently we require alignment be a power of 2
  if (!bits64::isPowerOfTwo(alignment)) {
    klee_warning_once(allocSite, "Alignment of %zu requested for %s but this "
                                 "not supported. Using alignment of %zu",
                      alignment, allocSite->getName().str().c_str(),
                      forcedAlignment);
    alignment = forcedAlignment;
  }
  assert(bits64::isPowerOfTwo(alignment) &&
         "Returned alignment must be a power of two");
  return alignment;
}

void Executor::prepareForEarlyExit() {
  if (statsTracker) {
    // Make sure stats get flushed out
    statsTracker->done();
  }
}
///

Interpreter *Interpreter::create(LLVMContext &ctx, const InterpreterOptions &opts,
                                 InterpreterHandler *ih) {
  return new Executor(ctx, opts, ih);
}
