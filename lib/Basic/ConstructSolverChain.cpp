//===-- ConstructSolverChain.cpp ------------------------------------++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

/*
 * This file groups declarations that are common to both KLEE and Kleaver.
 */
#include "klee/Common.h"
#include "klee/CommandLine.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "klee/CommandLine.h"

namespace klee {
Solver *constructSolverChain(Solver *coreSolver,
                             std::string querySMT2LogPath,
                             std::string baseSolverQuerySMT2LogPath,
                             std::string queryKQueryLogPath,
                             std::string baseSolverQueryKQueryLogPath) {
  Solver *solver = coreSolver;

  printf("Constructing Solver Chain:\n");
  
  if (queryLoggingOptions.isSet(SOLVER_KQUERY)) {
    printf("\tKQueryLoggingSolver\n");
    solver = createKQueryLoggingSolver(solver, baseSolverQueryKQueryLogPath,
                                   MinQueryTimeToLog);
    klee_message("Logging queries that reach solver in .kquery format to %s\n",
                 baseSolverQueryKQueryLogPath.c_str());
  }

  if (queryLoggingOptions.isSet(SOLVER_SMTLIB)) {
    printf("\tSMTLLIBLoggingSolver\n");
    solver = createSMTLIBLoggingSolver(solver, baseSolverQuerySMT2LogPath,
                                       MinQueryTimeToLog);
    klee_message("Logging queries that reach solver in .smt2 format to %s\n",
                 baseSolverQuerySMT2LogPath.c_str());
  }

  if (UseAssignmentValidatingSolver)
    printf("\tAssignmentValidatingSolver\n");
    solver = createAssignmentValidatingSolver(solver);

  if (UseFastCexSolver)
    printf("\tFastCexSolver\n");
    solver = createFastCexSolver(solver);

  if (UseCexCache)
    printf("\tCexCachingSolver\n");
    solver = createCexCachingSolver(solver);

  if (UseCache)
    printf("\tCachingSolver\n");
    solver = createCachingSolver(solver);

  /* if (UseTrivialEqSolver) 
    solver = createTrivialEqualitySolver(solver);
  */
  
  if (UseCanonicalization)
    printf("\tCononicalizationSolver\n");
    solver = createCanonicalSolver(solver);
  
  if (UseIndependentSolver) 
    printf("\tIndependetSolver\n");
    solver = createIndependentSolver(solver, UseLegacyIndependentSolver);

  if (DebugValidateSolver)
    printf("\tValidatingSolver\n");
    solver = createValidatingSolver(solver, coreSolver);

  if (queryLoggingOptions.isSet(ALL_KQUERY)) {
    printf("\tKQueryLoggingSolver\n");    
    solver = createKQueryLoggingSolver(solver, queryKQueryLogPath,
                                       MinQueryTimeToLog);
    klee_message("Logging all queries in .kquery format to %s\n",
                 queryKQueryLogPath.c_str());
  }

  if (queryLoggingOptions.isSet(ALL_SMTLIB)) {
    printf("\tSMTLIBLoggingSolver\n");    
    solver =
        createSMTLIBLoggingSolver(solver, querySMT2LogPath, MinQueryTimeToLog);
    klee_message("Logging all queries in .smt2 format to %s\n",
                 querySMT2LogPath.c_str());
  }
  if (DebugCrossCheckCoreSolverWith != NO_SOLVER) {
    printf("\tDebugCrossCheckCoreSolver\n");
    Solver *oracleSolver = createCoreSolver(DebugCrossCheckCoreSolverWith);
    solver = createValidatingSolver(/*s=*/solver, /*oracle=*/oracleSolver);
  }

  return solver;
}
}
