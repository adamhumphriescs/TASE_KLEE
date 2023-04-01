/*
 * This header groups command line options declarations and associated data
 * that is common for klee and kleaver.
 */

#ifndef KLEE_COMMANDLINE_H
#define KLEE_COMMANDLINE_H

#include "klee/Config/config.h"

#include "llvm/ADT/ArrayRef.h"
#include "llvm/Support/CommandLine.h"

//Tase Types
enum runType : int {INTERP_ONLY, MIXED};
enum TASETestType : int {EXPLORATION, VERIFICATION};
enum TASEExplorationType : int {DFS, BFS};
enum PoisonSize : uint8_t { WORD = 2, DWORD = 4 };
enum SIMDType : uint32_t { XMM = 128, YMM = 256, ZMM = 512 };

namespace klee {

  //TASE args
  extern llvm::cl::opt<runType> execMode;
  extern llvm::cl::opt<TASETestType> testType;
  extern llvm::cl::opt<TASEExplorationType> explorationType;
  extern llvm::cl::opt<PoisonSize> poisonSize;
  extern llvm::cl::opt<SIMDType> simdType;
  extern llvm::cl::opt<std::string> verificationLog;
  extern llvm::cl::opt<std::string> masterSecretFile;
  extern llvm::cl::opt<bool> skipFree;
  extern llvm::cl::opt<bool> killFlags;
  extern llvm::cl::opt<bool> taseManager;
  extern llvm::cl::opt<bool> tasePreProcess;
  extern llvm::cl::opt<bool> taseDebugArg;
  extern llvm::cl::opt<bool> modelDebugArg;
  extern llvm::cl::opt<bool> taseFloatDebug;
  extern llvm::cl::opt<std::string> logFileArg;
  extern llvm::cl::opt<bool> dontFork;
  extern llvm::cl::opt<bool> workerSelfTerminate;
  extern llvm::cl::opt<bool> UseLegacyIndependentSolver;
  extern llvm::cl::opt<bool> UseCanonicalization;
  extern llvm::cl::opt<bool> enableBounceback;
  //extern llvm::cl::opt<bool> dropS2C;
  extern llvm::cl::opt<bool> measureTime;
  extern llvm::cl::opt<bool> useCMS4;
  extern llvm::cl::opt<bool> useXOROpt;
  extern llvm::cl::opt<std::string> project;
  extern llvm::cl::opt<bool> disableSpringboard;
  extern llvm::cl::opt<int> retryMax;
  extern llvm::cl::opt<int> QRMaxWorkers;
  extern llvm::cl::opt<int> tranMaxArg;
  extern llvm::cl::opt<bool> optimizeOvershiftChecks;
  extern llvm::cl::opt<bool> optimizeConstMemOps;
    
extern llvm::cl::opt<bool> UseFastCexSolver;

extern llvm::cl::opt<bool> UseCexCache;

extern llvm::cl::opt<bool> UseCache;

extern llvm::cl::opt<bool> UseIndependentSolver; 

extern llvm::cl::opt<bool> DebugValidateSolver;
  
extern llvm::cl::opt<int> MinQueryTimeToLog;

extern llvm::cl::opt<double> MaxCoreSolverTime;

extern llvm::cl::opt<bool> UseForkedCoreSolver;

extern llvm::cl::opt<bool> CoreSolverOptimizeDivides;

extern llvm::cl::opt<bool> UseAssignmentValidatingSolver;

///The different query logging solvers that can switched on/off
enum QueryLoggingSolverType
{
    ALL_KQUERY,   ///< Log all queries (un-optimised) in .kquery (KQuery) format
    ALL_SMTLIB,   ///< Log all queries (un-optimised)  .smt2 (SMT-LIBv2) format
    SOLVER_KQUERY,///< Log queries passed to solver (optimised) in .kquery (KQuery) format
    SOLVER_SMTLIB ///< Log queries passed to solver (optimised) in .smt2 (SMT-LIBv2) format
};

extern llvm::cl::bits<QueryLoggingSolverType> queryLoggingOptions;

enum CoreSolverType {
  STP_SOLVER,
  METASMT_SOLVER,
  DUMMY_SOLVER,
  Z3_SOLVER,
  NO_SOLVER
};
extern llvm::cl::opt<CoreSolverType> CoreSolverToUse;

extern llvm::cl::opt<CoreSolverType> DebugCrossCheckCoreSolverWith;

#ifdef ENABLE_METASMT

enum MetaSMTBackendType
{
    METASMT_BACKEND_STP,
    METASMT_BACKEND_Z3,
    METASMT_BACKEND_BOOLECTOR,
    METASMT_BACKEND_CVC4,
    METASMT_BACKEND_YICES2
};

extern llvm::cl::opt<klee::MetaSMTBackendType> MetaSMTBackend;

#endif /* ENABLE_METASMT */

class KCommandLine {
public:
  /// Hide all options except the ones in the specified category
  static void HideUnrelatedOptions(llvm::cl::OptionCategory &Category);

  /// Hide all options except the ones in the specified categories
  static void HideUnrelatedOptions(
      llvm::ArrayRef<const llvm::cl::OptionCategory *> Categories);
};
}

#endif	/* KLEE_COMMANDLINE_H */
