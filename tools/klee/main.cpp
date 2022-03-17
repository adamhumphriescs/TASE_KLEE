/* -*- mode: c++; c-basic-offset: 2; -*- */

//===-- main.cpp ------------------------------------------------*- C++ -*-===//
//
//                     The KLEE Symbolic Virtual Machine
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "klee/Config/Version.h"
#include "klee/ExecutionState.h"
#include "klee/Expr.h"
#include "klee/Internal/ADT/KTest.h"
#include "klee/Internal/ADT/TreeStream.h"
#include "klee/Internal/Support/Debug.h"
#include "klee/Internal/Support/ErrorHandling.h"
#include "klee/Internal/Support/FileHandling.h"
#include "klee/Internal/Support/ModuleUtil.h"
#include "klee/Internal/Support/PrintVersion.h"
#include "klee/Internal/System/Time.h"
#include "klee/Interpreter.h"
#include "klee/Statistics.h"

#include "llvm/IR/Constants.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/Support/Errno.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Bitcode/ReaderWriter.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/Signals.h"

#if LLVM_VERSION_CODE < LLVM_VERSION(3, 5)
#include "llvm/Support/system_error.h"
#endif

#include <dirent.h>
#include <signal.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cerrno>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>

#include <stdlib.h>

//---------------------------------------------------------
//AH: BEGINNING OF TASE ADDITIONS (not including .h files)

#include "klee/CVAssignment.h"
#include "klee/util/ExprUtil.h"
#include "tase/TASEControl.h"
//#include "../../../test/proj_defs.h"
#include <sys/prctl.h>
#include <sys/time.h>
#include <iostream>
#include <unordered_set>

#include <malloc.h>
#include <fcntl.h>

int trace_ID;
double target_start_time;
double target_end_time;
extern double run_start_time;
extern double last_message_verification_time;
#include "tase.h"
#include "tase_interp.h"
extern target_ctx_t target_ctx;
tase_greg_t * target_ctx_gregs = target_ctx.gregs;

uint64_t targetMemAddr;
int glob_argc;
char ** glob_argv;
char ** glob_envp;
extern KTestObjectVector ktov;
extern "C" void begin_target_inner(int argc, char** argv);
extern "C" void klee_interp();

std::unordered_set<uint64_t> cartridge_entry_points;
std::unordered_set<uint64_t> cartridges_with_flags_live;

#ifdef TASE_OPENSSL
extern "C" void s_client_main(int argc, char ** argv);
#endif

//This struct is to help the solver for basic blocks with only
//two possible successors (e.g., blocks ending in "jb", "je", etc).

typedef struct cartridgeDestHint {
  uint64_t blockTop;
  uint64_t dest1;
  uint64_t dest2;
} cartridgeSuccessorInfo;

std::map<uint64_t, cartridgeSuccessorInfo> knownCartridgeDests;
std::stringstream worker_ID_stream;
std::string prev_worker_ID;
klee::Interpreter * GlobalInterpreter;
llvm::Module * interpModule;

extern char * ktestModePtr;
extern char * ktestPathPtr;
extern char ktestMode[20];
extern char ktestPath[100];
extern uint64_t trap_off;
extern int * target_started_ptr;
extern std::map<uint64_t, KFunction *> IR_KF_Map;

int QR_MAX_WORKERS = 8;
int masterPID;
bool enableMultipass = false;
bool taseDebug;
bool dropS2C;
bool enableTimeSeries;
bool bufferGuard;
int orig_stdout_fd;

#ifdef TASE_BIGNUM
extern int symIndex;
extern int numEntries;
#endif
extern void  exit_tase();

//AH: END OF TASE ADDITIONS
//-----------------------------------

using namespace llvm;
using namespace klee;

namespace klee {

  cl::opt<runType>
  execMode("execMode", cl::desc("INTERP_ONLY or MIXED (native and interpretation)"),
	      cl::values(clEnumValN(INTERP_ONLY, "INTERP_ONLY", "only execute via interpreter"),
                  clEnumValN(MIXED, "MIXED", "execute natively in transactions and interpret ")
		  KLEE_LLVM_CL_VAL_END),

	      cl::init(MIXED));

  cl::opt<std::string>
  targetArgs("targetArgs", cl::desc("pass-through arguments"), cl::init(""));

  cl::opt<TASETestType>
  testType("testType", cl::desc("EXPLORATION or VERIFICATION"),
	      cl::values(clEnumValN(EXPLORATION, "EXPLORATION", "Just execute and don't try to verify or do multiple passes"),
                  clEnumValN(VERIFICATION, "VERIFICATION", "Mark certain functions as symbolic and attempt to verify against a message log with multipass ")
									     KLEE_LLVM_CL_VAL_END),
	      
	      cl::init(EXPLORATION));

  
  cl::opt<TASEExplorationType>
  explorationType("explorationType", cl::desc("BFS or DFS"),
		  cl::values(clEnumValN(DFS, "DFS", "Depth-first search"),
			     clEnumValN(BFS, "BFS", "Breadth-first search" )
			     KLEE_LLVM_CL_VAL_END),

		  cl::init(BFS));
  
  cl::opt<std::string> verificationLog("verificationLog", cl::desc("ktest file to verify against for OpenSSL "), cl::init("./ssl.ktest"));

  cl::opt<std::string> masterSecretFile("masterSecretFile", cl::desc("File containing master secret for OpenSSL verification"), cl::init("./ssl.mastersecret"));
  
  cl::opt<bool>
  skipFree("skipFree", cl::desc("Debugging option to skip frees"), cl::init(false));

  cl::opt<bool>
  bufferGuardArg("bufferGuard", cl::desc("Add poison zones around heap buffers"), cl::init(false));
  
  cl::opt<bool>
  killFlags("killFlags", cl::desc("Option to kill dead flags register after each jump to the springboard"), cl::init(true));  
  
  cl::opt<bool>
  taseManager("taseManager", cl::desc("Fork off a manager process in TASE.  Expect a fork bomb if false."), cl::init(true));

  cl::opt<bool>
  tasePreProcess("tasePreProcess", cl::desc("Set to TRUE to run preprocessing in TASE and generate IR with code located in the executable"), cl::init(false));
  
  cl::opt<bool>
  taseDebugArg("taseDebug", cl::desc("Verbose logging in TASE"), cl::init(false));

  cl::opt<bool>
  modelDebug("modelDebug", cl::desc("Logging for models in TASE"), cl::init(false));

  cl::opt<bool>
  taseFloatDebug("taseFloatDebug", cl::desc("Log results of soft float emulation routines"), cl::init(true));
  
  cl::opt<bool>
  dontFork("dontFork", cl::desc("Disable forking in TASE for debugging"), cl::init(false));

  cl::opt<bool>
  noLog("noLog", cl::desc("No logging at all in TASE"), cl::init(false));
  
  cl::opt<bool>
  workerSelfTerminate("workerSelfTerminate", cl::desc("Workers will exit if they see they're in an earlier round"), cl::init(true));

  cl::opt<bool>
  UseLegacyIndependentSolver("use-legacy-independent-solver", cl::desc("Per cliver, pass through getInitialValue call in the independent solver without aggressive optimization"), cl::init(true));

  cl::opt<bool>
  UseCanonicalization("UseCanonicalization", cl::desc("Per cliver, canonicalize queries to be independent of variable name"), cl::init(true));
  
  cl::opt<bool>
  enableBounceback("enableBounceback", cl::desc("Try to bounce back to native execution in TASE depending on abort code"), cl::init(true));

  cl::opt<bool>
  dropS2CArg("dropS2C", cl::desc("Drop server to client messages for verification after the handshake"), cl::init(false));

  cl::opt<bool>
  enableTimeSeriesArg("enableTimeSeries", cl::desc("Perform verification across 21 traces and wait between message arrivals to simulate actual test conditions"), cl::init(false));
  
  cl::opt<bool>
  measureTime("measureTime", cl::desc("Time interpretation rounds in TASE for debugging"), cl::init(false));

  cl::opt<bool>
  useCMS4("useCMS4", cl::desc("Use cryptominisat4 instead of minisat as the SAT backend for STP "), cl::init(true));

  cl::opt<bool>
  useXOROpt("useXOROpt", cl::desc("Use optimization from cliver to eliminate unnecessary XOR expressions when using solver in writesocket model"), cl::init(true));
  
  cl::opt<std::string>
  project("project", cl::desc("Name of project in TASE"), cl::init("-"));

  cl::opt<bool>
  disableSpringboard("disableSpringboard", cl::desc("Enable or noop the springboard"), cl::init(false));

  cl::opt<int>
  retryMax("retryMax", cl::desc("Number of times to try and bounceback to native execution if abort status allows it "), cl::init(1));

  cl::opt<int>
  QRMaxWorkers("QRMaxWorkers", cl::desc("Maximum number of workers in TASE "), cl::init(8));
  
  cl::opt<int>
  tranMaxArg("tranBBMax", cl::desc("Max number of basic blocks to wrap into a single transaction"), cl::init(16));

  cl::opt<bool>
  optimizeOvershiftChecks("optimizeOvershiftChecks", cl::desc("Incorporate KLEE 2.1 optimization to simplify overshift checks"), cl::init(true));

  cl::opt<bool>
  optimizeConstMemOps("optimizeConstMemOps", cl::desc("Use optimizations for load/stores at constant offsets"), cl::init(false));
  
  #ifdef TASE_BIGNUM
  cl::opt<int>
  symIndexArg("symIndex", cl::desc("Index of symbolic byte in bignum test "), cl::init(100));

  cl::opt<int>
  numEntriesArg("numEntries", cl::desc("Size of array in bignum test "), cl::init(1000));
  #endif
}

namespace {
  cl::opt<std::string>
  InputFile(cl::desc("<input bytecode>"), cl::Positional, cl::init("-"));

  
  cl::opt<std::string>
  EntryPoint("entry-point",
               cl::desc("Consider the function with the given name as the entrypoint"),
               cl::init("_Z9dummyMainv"));

  cl::opt<std::string>
  RunInDir("run-in", cl::desc("Change to the given directory prior to executing"));

  cl::opt<std::string>
  Environ("environ", cl::desc("Parse environ from given file (in \"env\" format)"));

  cl::list<std::string>
  InputArgv(cl::ConsumeAfter,
            cl::desc("<program arguments>..."));

  cl::opt<bool>
  NoOutput("no-output",
           cl::desc("Don't generate test files"));

  cl::opt<bool>
  WarnAllExternals("warn-all-externals",
                   cl::desc("Give initial warning for all externals."));

  cl::opt<bool>
  WriteCVCs("write-cvcs",
            cl::desc("Write .cvc files for each test case"));

  cl::opt<bool>
  WriteKQueries("write-kqueries",
            cl::desc("Write .kquery files for each test case"));

  cl::opt<bool>
  WriteSMT2s("write-smt2s",
            cl::desc("Write .smt2 (SMT-LIBv2) files for each test case"));

  cl::opt<bool>
  WriteCov("write-cov",
           cl::desc("Write coverage information for each test case"));

  cl::opt<bool>
  WriteTestInfo("write-test-info",
                cl::desc("Write additional test case information"));

  cl::opt<bool>
  WritePaths("write-paths",
                cl::desc("Write .path files for each test case"));

  cl::opt<bool>
  WriteSymPaths("write-sym-paths",
                cl::desc("Write .sym.path files for each test case"));

  cl::opt<bool>
  OptExitOnError("exit-on-error",
              cl::desc("Exit if errors occur"));

  enum LibcType {
    NoLibc, KleeLibc, UcLibc
  };

  cl::opt<LibcType>
  Libc("libc",
       cl::desc("Choose libc version (none by default)."),
       cl::values(clEnumValN(NoLibc, "none", "Don't link in a libc"),
                  clEnumValN(KleeLibc, "klee", "Link in klee libc"),
		  clEnumValN(UcLibc, "uclibc", "Link in uclibc (adapted for klee)")
		  KLEE_LLVM_CL_VAL_END),
       cl::init(NoLibc));


  cl::opt<bool>
  WithPOSIXRuntime("posix-runtime",
		cl::desc("Link with POSIX runtime.  Options that can be passed as arguments to the programs are: --sym-arg <max-len>  --sym-args <min-argvs> <max-argvs> <max-len> + file model options"),
		cl::init(false));

  cl::opt<bool>
  OptimizeModule("optimize",
                 cl::desc("Optimize before execution"),
		 cl::init(false));

  cl::opt<bool>
  CheckDivZero("check-div-zero",
               cl::desc("Inject checks for division-by-zero"),
               cl::init(true));

  cl::opt<bool>
  CheckOvershift("check-overshift",
               cl::desc("Inject checks for overshift"),
               cl::init(true));

  cl::opt<std::string>
  OutputDir("output-dir",
            cl::desc("Directory to write results in (defaults to klee-out-N)"),
            cl::init(""));

  cl::opt<bool>
  ReplayKeepSymbolic("replay-keep-symbolic",
                     cl::desc("Replay the test cases only by asserting "
                              "the bytes, not necessarily making them concrete."));

  cl::list<std::string>
      ReplayKTestFile("replay-ktest-file",
                      cl::desc("Specify a ktest file to use for replay"),
                      cl::value_desc("ktest file"));

  cl::list<std::string>
      ReplayKTestDir("replay-ktest-dir",
                   cl::desc("Specify a directory to replay ktest files from"),
                   cl::value_desc("output directory"));

  cl::opt<std::string>
  ReplayPathFile("replay-path",
                 cl::desc("Specify a path file to replay"),
                 cl::value_desc("path file"));

  cl::list<std::string>
  SeedOutFile("seed-out");

  cl::list<std::string>
  SeedOutDir("seed-out-dir");

  cl::list<std::string>
  LinkLibraries("link-llvm-lib",
		cl::desc("Link the given libraries before execution"),
		cl::value_desc("library file"));

  cl::opt<unsigned>
  MakeConcreteSymbolic("make-concrete-symbolic",
                       cl::desc("Probabilistic rate at which to make concrete reads symbolic, "
				"i.e. approximately 1 in n concrete reads will be made symbolic (0=off, 1=all).  "
				"Used for testing."),
                       cl::init(0));

  cl::opt<unsigned>
  StopAfterNTests("stop-after-n-tests",
	     cl::desc("Stop execution after generating the given number of tests.  Extra tests corresponding to partially explored paths will also be dumped."),
	     cl::init(0));

  cl::opt<bool>
  Watchdog("watchdog",
           cl::desc("Use a watchdog process to enforce --max-time."),
           cl::init(0));
}

extern cl::opt<double> MaxTime;

/***/

class KleeHandler : public InterpreterHandler {
private:
  Interpreter *m_interpreter;
  TreeStreamWriter *m_pathWriter, *m_symPathWriter;
  llvm::raw_ostream *m_infoFile;

  SmallString<128> m_outputDirectory;

  unsigned m_numTotalTests;     // Number of tests received from the interpreter
  unsigned m_numGeneratedTests; // Number of tests successfully generated
  unsigned m_pathsExplored; // number of paths explored so far

  // used for writing .ktest files
  int m_argc;
  char **m_argv;

public:
  KleeHandler(int argc, char **argv);
  ~KleeHandler();

  llvm::raw_ostream &getInfoStream() const { return *m_infoFile; }
  /// Returns the number of test cases successfully generated so far
  unsigned getNumTestCases() { return m_numGeneratedTests; }
  unsigned getNumPathsExplored() { return m_pathsExplored; }
  void incPathsExplored() { m_pathsExplored++; }

  void setInterpreter(Interpreter *i);

  void processTestCase(const ExecutionState  &state,
                       const char *errorMessage,
                       const char *errorSuffix);

  std::string getOutputFilename(const std::string &filename);
  llvm::raw_fd_ostream *openOutputFile(const std::string &filename);
  std::string getTestFilename(const std::string &suffix, unsigned id);
  llvm::raw_fd_ostream *openTestFile(const std::string &suffix, unsigned id);

  // load a .path file
  static void loadPathFile(std::string name,
                           std::vector<bool> &buffer);

  static void getKTestFilesInDir(std::string directoryPath,
                                 std::vector<std::string> &results);

  static std::string getRunTimeLibraryPath(const char *argv0);
};

KleeHandler::KleeHandler(int argc, char **argv)
    : m_interpreter(0), m_pathWriter(0), m_symPathWriter(0), m_infoFile(0),
      m_outputDirectory(), m_numTotalTests(0), m_numGeneratedTests(0),
      m_pathsExplored(0), m_argc(argc), m_argv(argv) {

  // create output directory (OutputDir or "klee-out-<i>")
  bool dir_given = OutputDir != "";
  SmallString<128> directory(dir_given ? OutputDir : InputFile);

  if (!dir_given) sys::path::remove_filename(directory);
#if LLVM_VERSION_CODE < LLVM_VERSION(3, 5)
  error_code ec;
  if ((ec = sys::fs::make_absolute(directory)) != errc::success) {
#else
  if (auto ec = sys::fs::make_absolute(directory)) {
#endif
    klee_error("unable to determine absolute path: %s", ec.message().c_str());
  }

  if (dir_given) {
    // OutputDir
    if (mkdir(directory.c_str(), 0775) < 0)
      klee_error("cannot create \"%s\": %s", directory.c_str(), strerror(errno));

    m_outputDirectory = directory;
  } else {
    // "klee-out-<i>"
    int i = 0;
    for (; i <= INT_MAX; ++i) {
      SmallString<128> d(directory);
      llvm::sys::path::append(d, "klee-out-");
      raw_svector_ostream ds(d); ds << i; ds.flush();

      // create directory and try to link klee-last
      if (mkdir(d.c_str(), 0775) == 0) {
        m_outputDirectory = d;

        SmallString<128> klee_last(directory);
        llvm::sys::path::append(klee_last, "klee-last");

        if (((unlink(klee_last.c_str()) < 0) && (errno != ENOENT)) ||
            symlink(m_outputDirectory.c_str(), klee_last.c_str()) < 0) {

          klee_warning("cannot create klee-last symlink: %s", strerror(errno));
        }

        break;
      }

      // otherwise try again or exit on error
      if (errno != EEXIST)
        klee_error("cannot create \"%s\": %s", m_outputDirectory.c_str(), strerror(errno));
    }
    if (i == INT_MAX && m_outputDirectory.str().equals(""))
        klee_error("cannot create output directory: index out of range");
  }

  klee_message("output directory is \"%s\"", m_outputDirectory.c_str());

  // open warnings.txt
  std::string file_path = getOutputFilename("warnings.txt");
  if ((klee_warning_file = fopen(file_path.c_str(), "w")) == NULL)
    klee_error("cannot open file \"%s\": %s", file_path.c_str(), strerror(errno));

  // open messages.txt
  file_path = getOutputFilename("messages.txt");
  if ((klee_message_file = fopen(file_path.c_str(), "w")) == NULL)
    klee_error("cannot open file \"%s\": %s", file_path.c_str(), strerror(errno));

  // open info
  m_infoFile = openOutputFile("info");
}

KleeHandler::~KleeHandler() {
  delete m_pathWriter;
  delete m_symPathWriter;
  fclose(klee_warning_file);
  fclose(klee_message_file);
  delete m_infoFile;
}

void KleeHandler::setInterpreter(Interpreter *i) {
  m_interpreter = i;

  if (WritePaths) {
    m_pathWriter = new TreeStreamWriter(getOutputFilename("paths.ts"));
    assert(m_pathWriter->good());
    m_interpreter->setPathWriter(m_pathWriter);
  }

  if (WriteSymPaths) {
    m_symPathWriter = new TreeStreamWriter(getOutputFilename("symPaths.ts"));
    assert(m_symPathWriter->good());
    m_interpreter->setSymbolicPathWriter(m_symPathWriter);
  }
}

std::string KleeHandler::getOutputFilename(const std::string &filename) {
  SmallString<128> path = m_outputDirectory;
  sys::path::append(path,filename);
  return path.str();
}

llvm::raw_fd_ostream *KleeHandler::openOutputFile(const std::string &filename) {
  llvm::raw_fd_ostream *f;
  std::string Error;
  std::string path = getOutputFilename(filename);
  f = klee_open_output_file(path, Error);
  if (!Error.empty()) {
    klee_warning("error opening file \"%s\".  KLEE may have run out of file "
                 "descriptors: try to increase the maximum number of open file "
                 "descriptors by using ulimit (%s).",
                 path.c_str(), Error.c_str());
    return NULL;
  }
  return f;
}

std::string KleeHandler::getTestFilename(const std::string &suffix, unsigned id) {
  std::stringstream filename;
  filename << "test" << std::setfill('0') << std::setw(6) << id << '.' << suffix;
  return filename.str();
}

llvm::raw_fd_ostream *KleeHandler::openTestFile(const std::string &suffix,
                                                unsigned id) {
  return openOutputFile(getTestFilename(suffix, id));
}


/* Outputs all files (.ktest, .kquery, .cov etc.) describing a test case */
void KleeHandler::processTestCase(const ExecutionState &state,
                                  const char *errorMessage,
                                  const char *errorSuffix) {
  if (errorMessage && OptExitOnError) {
    m_interpreter->prepareForEarlyExit();
    klee_error("EXITING ON ERROR:\n%s\n", errorMessage);
  }

  if (!NoOutput) {
    std::vector< std::pair<std::string, std::vector<unsigned char> > > out;
    bool success = m_interpreter->getSymbolicSolution(state, out);

    if (!success)
      klee_warning("unable to get symbolic solution, losing test case");

    double start_time = util::getWallTime();

    unsigned id = ++m_numTotalTests;

    if (success) {
      KTest b;
      b.numArgs = m_argc;
      b.args = m_argv;
      b.symArgvs = 0;
      b.symArgvLen = 0;
      b.numObjects = out.size();
      b.objects = new KTestObject[b.numObjects];
      assert(b.objects);
      for (unsigned i=0; i<b.numObjects; i++) {
        KTestObject *o = &b.objects[i];
        o->name = const_cast<char*>(out[i].first.c_str());
        o->numBytes = out[i].second.size();
        o->bytes = new unsigned char[o->numBytes];
        assert(o->bytes);
        std::copy(out[i].second.begin(), out[i].second.end(), o->bytes);
      }

      if (!kTest_toFile(&b, getOutputFilename(getTestFilename("ktest", id)).c_str())) {
        klee_warning("unable to write output test case, losing it");
      } else {
        ++m_numGeneratedTests;
      }

      for (unsigned i=0; i<b.numObjects; i++)
        delete[] b.objects[i].bytes;
      delete[] b.objects;
    }

    if (errorMessage) {
      llvm::raw_ostream *f = openTestFile(errorSuffix, id);
      *f << errorMessage;
      delete f;
    }

    if (m_pathWriter) {
      std::vector<unsigned char> concreteBranches;
      m_pathWriter->readStream(m_interpreter->getPathStreamID(state),
                               concreteBranches);
      llvm::raw_fd_ostream *f = openTestFile("path", id);
      for (std::vector<unsigned char>::iterator I = concreteBranches.begin(),
                                                E = concreteBranches.end();
           I != E; ++I) {
        *f << *I << "\n";
      }
      delete f;
    }

    if (errorMessage || WriteKQueries) {
      std::string constraints;
      m_interpreter->getConstraintLog(state, constraints,Interpreter::KQUERY);
      llvm::raw_ostream *f = openTestFile("kquery", id);
      *f << constraints;
      delete f;
    }

    if (WriteCVCs) {
      // FIXME: If using Z3 as the core solver the emitted file is actually
      // SMT-LIBv2 not CVC which is a bit confusing
      std::string constraints;
      m_interpreter->getConstraintLog(state, constraints, Interpreter::STP);
      llvm::raw_ostream *f = openTestFile("cvc", id);
      *f << constraints;
      delete f;
    }

    if(WriteSMT2s) {
      std::string constraints;
        m_interpreter->getConstraintLog(state, constraints, Interpreter::SMTLIB2);
        llvm::raw_ostream *f = openTestFile("smt2", id);
        *f << constraints;
        delete f;
    }

    if (m_symPathWriter) {
      std::vector<unsigned char> symbolicBranches;
      m_symPathWriter->readStream(m_interpreter->getSymbolicPathStreamID(state),
                                  symbolicBranches);
      llvm::raw_fd_ostream *f = openTestFile("sym.path", id);
      for (std::vector<unsigned char>::iterator I = symbolicBranches.begin(), E = symbolicBranches.end(); I!=E; ++I) {
        *f << *I << "\n";
      }
      delete f;
    }

    if (WriteCov) {
      std::map<const std::string*, std::set<unsigned> > cov;
      m_interpreter->getCoveredLines(state, cov);
      llvm::raw_ostream *f = openTestFile("cov", id);
      for (std::map<const std::string*, std::set<unsigned> >::iterator
             it = cov.begin(), ie = cov.end();
           it != ie; ++it) {
        for (std::set<unsigned>::iterator
               it2 = it->second.begin(), ie = it->second.end();
             it2 != ie; ++it2)
          *f << *it->first << ":" << *it2 << "\n";
      }
      delete f;
    }

    if (m_numGeneratedTests == StopAfterNTests)
      m_interpreter->setHaltExecution(true);

    if (WriteTestInfo) {
      double elapsed_time = util::getWallTime() - start_time;
      llvm::raw_ostream *f = openTestFile("info", id);
      *f << "Time to generate test case: "
         << elapsed_time << "s\n";
      delete f;
    }
  }
}

  // load a .path file
void KleeHandler::loadPathFile(std::string name,
                                     std::vector<bool> &buffer) {
  std::ifstream f(name.c_str(), std::ios::in | std::ios::binary);

  if (!f.good())
    assert(0 && "unable to open path file");

  while (f.good()) {
    unsigned value;
    f >> value;
    buffer.push_back(!!value);
    f.get();
  }
}

void KleeHandler::getKTestFilesInDir(std::string directoryPath,
                                     std::vector<std::string> &results) {
#if LLVM_VERSION_CODE < LLVM_VERSION(3, 5)
  error_code ec;
#else
  std::error_code ec;
#endif
  for (llvm::sys::fs::directory_iterator i(directoryPath, ec), e; i != e && !ec;
       i.increment(ec)) {
    std::string f = (*i).path();
    if (f.substr(f.size()-6,f.size()) == ".ktest") {
          results.push_back(f);
    }
  }

  if (ec) {
    llvm::errs() << "ERROR: unable to read output directory: " << directoryPath
                 << ": " << ec.message() << "\n";
    exit(1);
  }
}

std::string KleeHandler::getRunTimeLibraryPath(const char *argv0) {
  // allow specifying the path to the runtime library
  const char *env = getenv("KLEE_RUNTIME_LIBRARY_PATH");
  if (env)
    return std::string(env);

  // Take any function from the execution binary but not main (as not allowed by
  // C++ standard)
  void *MainExecAddr = (void *)(intptr_t)getRunTimeLibraryPath;
  SmallString<128> toolRoot(
      llvm::sys::fs::getMainExecutable(argv0, MainExecAddr)
      );

  // Strip off executable so we have a directory path
  llvm::sys::path::remove_filename(toolRoot);

  SmallString<128> libDir;

  if (strlen( KLEE_INSTALL_BIN_DIR ) != 0 &&
      strlen( KLEE_INSTALL_RUNTIME_DIR ) != 0 &&
      toolRoot.str().endswith( KLEE_INSTALL_BIN_DIR ))
  {
    KLEE_DEBUG_WITH_TYPE("klee_runtime", llvm::dbgs() <<
                         "Using installed KLEE library runtime: ");
    libDir = toolRoot.str().substr(0,
               toolRoot.str().size() - strlen( KLEE_INSTALL_BIN_DIR ));
    llvm::sys::path::append(libDir, KLEE_INSTALL_RUNTIME_DIR);
  }
  else
  {
    KLEE_DEBUG_WITH_TYPE("klee_runtime", llvm::dbgs() <<
                         "Using build directory KLEE library runtime :");
    libDir = KLEE_DIR;
    llvm::sys::path::append(libDir,RUNTIME_CONFIGURATION);
    llvm::sys::path::append(libDir,"lib");
  }

  KLEE_DEBUG_WITH_TYPE("klee_runtime", llvm::dbgs() <<
                       libDir.c_str() << "\n");
  return libDir.str();
}

//===----------------------------------------------------------------------===//
// main Driver function
//
static std::string strip(std::string &in) {
  unsigned len = in.size();
  unsigned lead = 0, trail = len;
  while (lead<len && isspace(in[lead]))
    ++lead;
  while (trail>lead && isspace(in[trail-1]))
    --trail;
  return in.substr(lead, trail-lead);
}

static void parseArguments(int argc, char **argv) {
  cl::SetVersionPrinter(klee::printVersion);
  // This version always reads response files
  cl::ParseCommandLineOptions(argc, argv, " klee\n");
}

static int initEnv(Module *mainModule) {

  /*
    nArgcP = alloc oldArgc->getType()
    nArgvV = alloc oldArgv->getType()
    store oldArgc nArgcP
    store oldArgv nArgvP
    klee_init_environment(nArgcP, nArgvP)
    nArgc = load nArgcP
    nArgv = load nArgvP
    oldArgc->replaceAllUsesWith(nArgc)
    oldArgv->replaceAllUsesWith(nArgv)
  */

  Function *mainFn = mainModule->getFunction(EntryPoint);

  if (!mainFn) {
    klee_error("'%s' function not found in module.", EntryPoint.c_str());
  }

  if (mainFn->arg_size() < 2) {
    klee_error("Cannot handle ""--posix-runtime"" when main() has less than two arguments.\n");
  }

  Instruction *firstInst = &*(mainFn->begin()->begin());

  Value *oldArgc = &*(mainFn->arg_begin());
  Value *oldArgv = &*(++mainFn->arg_begin());

  AllocaInst* argcPtr =
    new AllocaInst(oldArgc->getType(), "argcPtr", firstInst);
  AllocaInst* argvPtr =
    new AllocaInst(oldArgv->getType(), "argvPtr", firstInst);

  /* Insert void klee_init_env(int* argc, char*** argv) */
  std::vector<const Type*> params;
  LLVMContext &ctx = mainModule->getContext();
  params.push_back(Type::getInt32Ty(ctx));
  params.push_back(Type::getInt32Ty(ctx));
  Function* initEnvFn =
    cast<Function>(mainModule->getOrInsertFunction("klee_init_env",
                                                   Type::getVoidTy(ctx),
                                                   argcPtr->getType(),
                                                   argvPtr->getType(),
                                                   NULL));
  assert(initEnvFn);
  std::vector<Value*> args;
  args.push_back(argcPtr);
  args.push_back(argvPtr);
  Instruction* initEnvCall = CallInst::Create(initEnvFn, args,
					      "", firstInst);
  Value *argc = new LoadInst(argcPtr, "newArgc", firstInst);
  Value *argv = new LoadInst(argvPtr, "newArgv", firstInst);

  oldArgc->replaceAllUsesWith(argc);
  oldArgv->replaceAllUsesWith(argv);

  new StoreInst(oldArgc, argcPtr, initEnvCall);
  new StoreInst(oldArgv, argvPtr, initEnvCall);

  return 0;
}


// This is a terrible hack until we get some real modeling of the
// system. All we do is check the undefined symbols and warn about
// any "unrecognized" externals and about any obviously unsafe ones.

// Symbols we explicitly support
static const char *modelledExternals[] = {
  "_ZTVN10__cxxabiv117__class_type_infoE",
  "_ZTVN10__cxxabiv120__si_class_type_infoE",
  "_ZTVN10__cxxabiv121__vmi_class_type_infoE",

  // special functions
  "_assert",
  "__assert_fail",
  "__assert_rtn",
  "calloc",
  "_exit",
  "exit",
  "free",
  "abort",
  "klee_abort",
  "klee_assume",
  "klee_check_memory_access",
  "klee_define_fixed_object",
  "klee_get_errno",
  "klee_get_valuef",
  "klee_get_valued",
  "klee_get_valuel",
  "klee_get_valuell",
  "klee_get_value_i32",
  "klee_get_value_i64",
  "klee_get_obj_size",
  "klee_is_symbolic",
  "klee_make_symbolic",
  "klee_mark_global",
  "klee_open_merge",
  "klee_close_merge",
  "klee_prefer_cex",
  "klee_posix_prefer_cex",
  "klee_print_expr",
  "klee_print_range",
  "klee_report_error",
  "klee_set_forking",
  "klee_silent_exit",
  "klee_warning",
  "klee_warning_once",
  "klee_alias_function",
  "klee_stack_trace",
  "llvm.dbg.declare",
  "llvm.dbg.value",
  "llvm.va_start",
  "llvm.va_end",
  "malloc",
  "realloc",
  "_ZdaPv",
  "_ZdlPv",
  "_Znaj",
  "_Znwj",
  "_Znam",
  "_Znwm",
  "__ubsan_handle_add_overflow",
  "__ubsan_handle_sub_overflow",
  "__ubsan_handle_mul_overflow",
  "__ubsan_handle_divrem_overflow",
};
// Symbols we aren't going to warn about
static const char *dontCareExternals[] = {
#if 0
  // stdio
  "fprintf",
  "fflush",
  "fopen",
  "fclose",
  "fputs_unlocked",
  "putchar_unlocked",
  "vfprintf",
  "fwrite",
  "puts",
  "printf",
  "stdin",
  "stdout",
  "stderr",
  "_stdio_term",
  "__errno_location",
  "fstat",
#endif

  // static information, pretty ok to return
  "getegid",
  "geteuid",
  "getgid",
  "getuid",
  "getpid",
  "gethostname",
  "getpgrp",
  "getppid",
  "getpagesize",
  "getpriority",
  "getgroups",
  "getdtablesize",
  "getrlimit",
  "getrlimit64",
  "getcwd",
  "getwd",
  "gettimeofday",
  "uname",

  // fp stuff we just don't worry about yet
  "frexp",
  "ldexp",
  "__isnan",
  "__signbit",
};
// Extra symbols we aren't going to warn about with klee-libc
static const char *dontCareKlee[] = {
  "__ctype_b_loc",
  "__ctype_get_mb_cur_max",

  // io system calls
  "open",
  "write",
  "read",
  "close",
};
// Extra symbols we aren't going to warn about with uclibc
static const char *dontCareUclibc[] = {
  "__dso_handle",

  // Don't warn about these since we explicitly commented them out of
  // uclibc.
  "printf",
  "vprintf"
};
// Symbols we consider unsafe
static const char *unsafeExternals[] = {
  "fork", // oh lord
  "exec", // heaven help us
  "error", // calls _exit
  "raise", // yeah
  "kill", // mmmhmmm
};
#define NELEMS(array) (sizeof(array)/sizeof(array[0]))
void externalsAndGlobalsCheck(const Module *m) {
  std::map<std::string, bool> externals;
  std::set<std::string> modelled(modelledExternals,
                                 modelledExternals+NELEMS(modelledExternals));
  std::set<std::string> dontCare(dontCareExternals,
                                 dontCareExternals+NELEMS(dontCareExternals));
  std::set<std::string> unsafe(unsafeExternals,
                               unsafeExternals+NELEMS(unsafeExternals));

  switch (Libc) {
  case KleeLibc:
    dontCare.insert(dontCareKlee, dontCareKlee+NELEMS(dontCareKlee));
    break;
  case UcLibc:
    dontCare.insert(dontCareUclibc,
                    dontCareUclibc+NELEMS(dontCareUclibc));
    break;
  case NoLibc: /* silence compiler warning */
    break;
  }

  if (WithPOSIXRuntime)
    dontCare.insert("syscall");

  for (Module::const_iterator fnIt = m->begin(), fn_ie = m->end();
       fnIt != fn_ie; ++fnIt) {
    if (fnIt->isDeclaration() && !fnIt->use_empty())
      externals.insert(std::make_pair(fnIt->getName(), false));
    for (Function::const_iterator bbIt = fnIt->begin(), bb_ie = fnIt->end();
         bbIt != bb_ie; ++bbIt) {
      for (BasicBlock::const_iterator it = bbIt->begin(), ie = bbIt->end();
           it != ie; ++it) {
        if (const CallInst *ci = dyn_cast<CallInst>(it)) {
          if (isa<InlineAsm>(ci->getCalledValue())) {
            klee_warning_once(&*fnIt,
                              "function \"%s\" has inline asm",
                              fnIt->getName().data());
          }
        }
      }
    }
  }
  for (Module::const_global_iterator
         it = m->global_begin(), ie = m->global_end();
       it != ie; ++it)
    if (it->isDeclaration() && !it->use_empty())
      externals.insert(std::make_pair(it->getName(), true));
  // and remove aliases (they define the symbol after global
  // initialization)
  for (Module::const_alias_iterator
         it = m->alias_begin(), ie = m->alias_end();
       it != ie; ++it) {
    std::map<std::string, bool>::iterator it2 =
      externals.find(it->getName());
    if (it2!=externals.end())
      externals.erase(it2);
  }

  std::map<std::string, bool> foundUnsafe;
  for (std::map<std::string, bool>::iterator
         it = externals.begin(), ie = externals.end();
       it != ie; ++it) {
    const std::string &ext = it->first;
    if (!modelled.count(ext) && (WarnAllExternals ||
                                 !dontCare.count(ext))) {
      if (unsafe.count(ext)) {
        foundUnsafe.insert(*it);
      } else {
        klee_warning("undefined reference to %s: %s",
                     it->second ? "variable" : "function",
                     ext.c_str());
      }
    }
  }

  for (std::map<std::string, bool>::iterator
         it = foundUnsafe.begin(), ie = foundUnsafe.end();
       it != ie; ++it) {
    const std::string &ext = it->first;
    klee_warning("undefined reference to %s: %s (UNSAFE)!",
                 it->second ? "variable" : "function",
                 ext.c_str());
  }
}

static Interpreter *theInterpreter = 0;

static bool interrupted = false;

// Pulled out so it can be easily called from a debugger.
extern "C"
void halt_execution() {
  theInterpreter->setHaltExecution(true);
}

extern "C"
void stop_forking() {
  theInterpreter->setInhibitForking(true);
}

static void interrupt_handle() {
  if (!interrupted && theInterpreter) {
    llvm::errs() << "KLEE: ctrl-c detected, requesting interpreter to halt.\n";
    halt_execution();
    sys::SetInterruptFunction(interrupt_handle);
  } else {
    llvm::errs() << "KLEE: ctrl-c detected, exiting.\n";
    exit(1);
  }
  interrupted = true;
}

static void interrupt_handle_watchdog() {
  // just wait for the child to finish
}

// This is a temporary hack. If the running process has access to
// externals then it can disable interrupts, which screws up the
// normal "nice" watchdog termination process. We try to request the
// interpreter to halt using this mechanism as a last resort to save
// the state data before going ahead and killing it.
/*
static void halt_via_gdb(int pid) {
  char buffer[256];
  sprintf(buffer,
          "gdb --batch --eval-command=\"p halt_execution()\" "
          "--eval-command=detach --pid=%d &> /dev/null",
          pid);
  //  fprintf(stderr, "KLEE: WATCHDOG: running: %s\n", buffer);
  if (system(buffer)==-1)
    perror("system");
}
*/
// returns the end of the string put in buf
static char *format_tdiff(char *buf, long seconds)
{
  assert(seconds >= 0);

  long minutes = seconds / 60;  seconds %= 60;
  long hours   = minutes / 60;  minutes %= 60;
  long days    = hours   / 24;  hours   %= 24;

  buf = strrchr(buf, '\0');
  if (days > 0) buf += sprintf(buf, "%ld days, ", days);
  buf += sprintf(buf, "%02ld:%02ld:%02ld", hours, minutes, seconds);
  return buf;
}

#ifndef SUPPORT_KLEE_UCLIBC
static llvm::Module *linkWithUclibc(llvm::Module *mainModule, StringRef libDir) {
  klee_error("invalid libc, no uclibc support!\n");
}
#else
static void replaceOrRenameFunction(llvm::Module *module,
		const char *old_name, const char *new_name)
{
  Function *f, *f2;
  f = module->getFunction(new_name);
  f2 = module->getFunction(old_name);
  if (f2) {
    if (f) {
      f2->replaceAllUsesWith(f);
      f2->eraseFromParent();
    } else {
      f2->setName(new_name);
      assert(f2->getName() == new_name);
    }
  }
}
static llvm::Module *linkWithUclibc(llvm::Module *mainModule, StringRef libDir) {
  LLVMContext &ctx = mainModule->getContext();
  // Ensure that klee-uclibc exists
  SmallString<128> uclibcBCA(libDir);
  llvm::sys::path::append(uclibcBCA, KLEE_UCLIBC_BCA_NAME);

#if LLVM_VERSION_CODE >= LLVM_VERSION(3, 6)
  Twine uclibcBCA_twine(uclibcBCA.c_str());
  if (!llvm::sys::fs::exists(uclibcBCA_twine))
#else
  bool uclibcExists=false;
  llvm::sys::fs::exists(uclibcBCA.c_str(), uclibcExists);
  if (!uclibcExists)
#endif
    klee_error("Cannot find klee-uclibc : %s", uclibcBCA.c_str());

  Function *f;
  // force import of __uClibc_main
  mainModule->getOrInsertFunction(
      "__uClibc_main",
      FunctionType::get(Type::getVoidTy(ctx), std::vector<Type *>(), true));

  // force various imports
  if (WithPOSIXRuntime) {
    llvm::Type *i8Ty = Type::getInt8Ty(ctx);
    mainModule->getOrInsertFunction("realpath",
                                    PointerType::getUnqual(i8Ty),
                                    PointerType::getUnqual(i8Ty),
                                    PointerType::getUnqual(i8Ty),
                                    NULL);
    mainModule->getOrInsertFunction("getutent",
                                    PointerType::getUnqual(i8Ty),
                                    NULL);
    mainModule->getOrInsertFunction("__fgetc_unlocked",
                                    Type::getInt32Ty(ctx),
                                    PointerType::getUnqual(i8Ty),
                                    NULL);
    mainModule->getOrInsertFunction("__fputc_unlocked",
                                    Type::getInt32Ty(ctx),
                                    Type::getInt32Ty(ctx),
                                    PointerType::getUnqual(i8Ty),
                                    NULL);
  }

  f = mainModule->getFunction("__ctype_get_mb_cur_max");
  if (f) f->setName("_stdlib_mb_cur_max");

  // Strip of asm prefixes for 64 bit versions because they are not
  // present in uclibc and we want to make sure stuff will get
  // linked. In the off chance that both prefixed and unprefixed
  // versions are present in the module, make sure we don't create a
  // naming conflict.
  for (Module::iterator fi = mainModule->begin(), fe = mainModule->end();
       fi != fe; ++fi) {
    Function *f = &*fi;
    const std::string &name = f->getName();
    if (name[0]=='\01') {
      unsigned size = name.size();
      if (name[size-2]=='6' && name[size-1]=='4') {
        std::string unprefixed = name.substr(1);

        // See if the unprefixed version exists.
        if (Function *f2 = mainModule->getFunction(unprefixed)) {
          f->replaceAllUsesWith(f2);
          f->eraseFromParent();
        } else {
          f->setName(unprefixed);
        }
      }
    }
  }

  mainModule = klee::linkWithLibrary(mainModule, uclibcBCA.c_str());
  assert(mainModule && "unable to link with uclibc");


  replaceOrRenameFunction(mainModule, "__libc_open", "open");
  replaceOrRenameFunction(mainModule, "__libc_fcntl", "fcntl");

  // XXX we need to rearchitect so this can also be used with
  // programs externally linked with uclibc.

  // We now need to swap things so that __uClibc_main is the entry
  // point, in such a way that the arguments are passed to
  // __uClibc_main correctly. We do this by renaming the user main
  // and generating a stub function to call __uClibc_main. There is
  // also an implicit cooperation in that runFunctionAsMain sets up
  // the environment arguments to what uclibc expects (following
  // argv), since it does not explicitly take an envp argument.
  Function *userMainFn = mainModule->getFunction(EntryPoint);
  assert(userMainFn && "unable to get user main");
  Function *uclibcMainFn = mainModule->getFunction("__uClibc_main");
  assert(uclibcMainFn && "unable to get uclibc main");
  userMainFn->setName("__user_main");

  const FunctionType *ft = uclibcMainFn->getFunctionType();
  assert(ft->getNumParams() == 7);

  std::vector<Type *> fArgs;
  fArgs.push_back(ft->getParamType(1)); // argc
  fArgs.push_back(ft->getParamType(2)); // argv
  Function *stub = Function::Create(FunctionType::get(Type::getInt32Ty(ctx), fArgs, false),
                                    GlobalVariable::ExternalLinkage,
                                    EntryPoint,
                                    mainModule);
  BasicBlock *bb = BasicBlock::Create(ctx, "entry", stub);

  std::vector<llvm::Value*> args;
  args.push_back(llvm::ConstantExpr::getBitCast(userMainFn,
                                                ft->getParamType(0)));
  args.push_back(&*(stub->arg_begin())); // argc
  args.push_back(&*(++stub->arg_begin())); // argv
  args.push_back(Constant::getNullValue(ft->getParamType(3))); // app_init
  args.push_back(Constant::getNullValue(ft->getParamType(4))); // app_fini
  args.push_back(Constant::getNullValue(ft->getParamType(5))); // rtld_fini
  args.push_back(Constant::getNullValue(ft->getParamType(6))); // stack_end
  CallInst::Create(uclibcMainFn, args, "", bb);

  new UnreachableInst(ctx, bb);

  klee_message("NOTE: Using klee-uclibc : %s", uclibcBCA.c_str());
  return mainModule;
}
#endif

 void printTASEArgs() {

   printf("TASE args... \n");
   if (execMode == MIXED) 
     printf("\t Running MIXED mode (native with interpreter) \n");
   else if (execMode == INTERP_ONLY)
     printf("\t Running INTERP_ONLY mode \n");
   else
     printf("ERROR: unrecognized execMode \n");

   if (testType == EXPLORATION)
     printf("\t Test type is EXPLORATION (running through all paths) \n");
   else if (testType == VERIFICATION )
     printf("\t Test type is VERIFICATION \n");
   else
     printf("ERROR: unrecognized testType \n");

   printf("\t TASE project name: %s \n", project.c_str());
   printf("\t TASE project args: \"%s\"\n", targetArgs.getValue().c_str());
   //Really shouldn't be casting to "bool" below given it's not
   //a defined type for C-style printfs.  Replace with a printstream
   //or something later.
   
   printf("\t taseManager: %d \n", (bool) taseManager);
   printf("\t tasePreProcess  : %d \n", (bool) tasePreProcess);
   printf("\t taseDebug output: %d \n", (bool) taseDebug);
   printf("\t bufferGuard output: %d \n", (bool) bufferGuard);
   printf("\t modelDebug output: %d \n", (bool) modelDebug);
   printf("\t noLog      output: %d \n", (bool) noLog);
   printf("\t dontFork  output: %d \n", (bool) dontFork);
   
   printf("\t killFlags      output : %d \n", (bool) killFlags);
   printf("\t skipFree           output : %d \n", (bool) skipFree);
   printf("\t measureTime         output : %d \n", (bool) measureTime);
   printf("\t enableBounceback     output : %d \n", (bool) enableBounceback);
   printf("\t workerSelfTerminate  output : %d \n", (bool) workerSelfTerminate);
   printf("\t dropS2C              output : %d \n", (bool) dropS2C);
   printf("\t enableTimeSeries     output : %d \n", (bool) enableTimeSeries);
   printf("\t retryMax             output : %d \n", (int) retryMax);
   printf("\t tranBBMax            output : %lu \n", (size_t) tran_max);
   printf("\t QRMaxWorkers         output : %d  \n", (int) QR_MAX_WORKERS);
   printf("\t useCMS4                    output : %d \n", (bool) useCMS4);
   printf("\t useXOROpt                  output : %d \n", (bool) useXOROpt);
   printf("\t UseLegacyIndependentSolver output : %d \n", (bool) UseLegacyIndependentSolver);
   printf("\t UseCanonicalization        output : %d \n", (bool) UseCanonicalization);
   printf("\t optimizeOvershiftChecks    output : %d \n", (bool) optimizeOvershiftChecks);
   printf("\t optimizeConstMemOps        output : %d \n", (bool) optimizeConstMemOps);
 }


 //Load cartridge reference info.
 //This is marked "noinline" in the hope that any stack space used
 //in the initialization can be reclaimed before we transfer to the
 //analysis target and get stuck dealing with it across forks.
 
 void __attribute__((noinline)) loadCartridgeInfo()  {
   printf("Detected %lu basic block records when loading cartridge info \n", tase_num_global_records);
   
   //Load the start addresses of cartridges for fast lookup later
   for (uint32_t i = 0; i < tase_num_global_records; i++) 
     cartridge_entry_points.insert(tase_global_records[i].head + tase_global_records[i].head_size);

   int numLiveBlocks = 0;
   for (uint32_t i = 0; i < tase_num_live_flags_block_records; i++) {
     cartridges_with_flags_live.insert(tase_live_flags_block_records[i].head + tase_live_flags_block_records[i].head_size);
     numLiveBlocks++;
   }
   printf("Found %d basic blocks with flags live-in \n", numLiveBlocks);
 }

 //Make seperate directories for each of the workers in the time series.
 void makeTSPath(int trace_ID) {
   //TODO -- merge in ssl verification code
  
 }

 bool isTimeSeriesDone() {
   //TODO -- merge in ssl verification code
   return true;
 }

 void __attribute__ ((noreturn)) transferToTarget(int argc, char** argv)  {

   run_start_time = util::getWallTime();
   printf("Inside transferToTarget \n");

   target_ctx.rdi.i64 = argc;
   target_ctx.rsi.u64 = (uint64_t) argv;

   if (execMode == INTERP_ONLY) {
     memset(&target_ctx, 0, sizeof(target_ctx));

     target_ctx.rdi.i64 = argc;
     target_ctx.rsi.u64 = (uint64_t) argv;

     target_ctx.target_exit_addr = (uintptr_t)&exit_tase_shim;
     target_ctx.sentinel[0] = CTX_STACK_SENTINEL;
     target_ctx.sentinel[1] = CTX_STACK_SENTINEL;
     target_ctx.r15.u64 = (uint64_t) &begin_target_inner;
     target_ctx.rip.u64 = (uint64_t) &begin_target_inner;
     target_ctx.rax = target_ctx.rip;
     // We pretend like we have pushed a return address as part of call.
     target_ctx.rsp.u64 = (uint64_t)(&target_ctx.target_exit_addr);
     // Just to be careful.  rbp should not be necessary but debuggers like it.
     target_ctx.rbp.u64 = target_ctx.rsp.u64 + sizeof(uintptr_t);

     tase_springboard = (void *) &sb_disabled;
    
     klee_interp();
    
   } else {
    
     int sbArg = 1;
     enter_tase(&begin_target_inner + trap_off, sbArg);
     if (taseDebug) {
       printf("TASE - returned from enter_tase... \n");
       std::cout.flush();
     }
     while (target_ctx_gregs[GREG_RIP].u64 != (uint64_t) &tase_exit) {
       klee_interp();
       if (taseDebug) {
         printf("Returning from klee_interp ... \n");
         std::cout.flush();
       }
       tase_inject(sbArg);
       if (taseDebug) {
         printf("Returning from tase_inject ... \n");
         std::cout.flush();
       }
     }
   }  
 }
 
 //Attempt to load basic block successor information for basic blocks ending in
 //non-indirect control flow (e.g., je, jne, etc).  Information goes into
 //"knownCartridgeDests" to give solver a hint when encountering symbolic
 //RIP values after certain basic blocks.

 //This can be generalized and automated within the compiler, but for now
 //it's a manual process. 
 void __attribute__((noinline)) loadCartridgeDests() {

   //TODO
   
 }
 
 int main (int argc, char **argv, char **envp) {

   signal(SIGCHLD, SIG_IGN);
     
   glob_argc = argc;
   glob_argv = argv;
   glob_envp = envp;
   
   llvm::InitializeNativeTarget();
   parseArguments(argc, argv);

   if (testType ==VERIFICATION) {
     printf("Enabling multipass verification for openssl \n");
     enableMultipass = true;
   }

   //Ugly!
   taseDebug = taseDebugArg;
   bufferGuard = bufferGuardArg;
   dropS2C = dropS2CArg;
   enableTimeSeries = enableTimeSeriesArg;

   //If we don't explicitly give the project name, use some defaults.
   //This is particularly useful when running TASE from the projects directory because
   //it removes the number of args needed to launch TASE.
   if (strcmp(project.c_str(), "-") == 0) {
     char pathBuf [512];
     getcwd(pathBuf, 512);
     std::string path (pathBuf);
     size_t idx = path.find_last_of('/');
     std::string currDir = path.substr(idx + 1, std::string::npos);

     project=currDir;

     //Also, load the bitcode from ./build/bitcode
     InputFile = path + "/build/bitcode/" + project + ".interp.bc";     
   }
   
   
     
   QR_MAX_WORKERS = QRMaxWorkers;
   tran_max = (uint64_t) tranMaxArg;

   if (!tasePreProcess) {
     printTASEArgs();
   }
   
#ifdef TASE_BIGNUM
   symIndex = symIndexArg;
   numEntries = numEntriesArg;
   printf("BigNum Test: symIndex is %d, numEntries is %d ", symIndex, numEntries);
   fflush(stdout);
#endif

   if (tasePreProcess) {
     printf("Running TASE preprocessing... \n");
     
     FILE * cartridgeLog = fopen("cartridge_info.txt", "w");
     for (uint32_t i = 0; i < tase_num_global_records; i++) {
       uint32_t head = tase_global_records[i].head;
       uint16_t head_size = tase_global_records[i].head_size;
       uint16_t body_size = tase_global_records[i].body_size;
       fprintf(cartridgeLog, "%d %d\n", head + head_size, head + head_size + body_size);
     }
     fclose(cartridgeLog);
     printf("Finished TASE preprocessing \n");
     fflush(stdout);
     std::exit(EXIT_SUCCESS);
   }

   //Redirect stdout messages to a file called "Monitor".
   //Later, calls to unix fork in executor create new filenames
   //after each fork.

   orig_stdout_fd=dup(STDOUT_FILENO); //Backup stdout fd so that we can grab control later.
   
   if (!noLog) {
   
     worker_ID_stream << "Monitor";
     std::string IDString;
     IDString = worker_ID_stream.str();
     FILE * tmpFile = freopen(IDString.c_str(),"w", stdout);
     if (tmpFile == NULL) {
       printf("FATAL ERROR redirecting stdout \n");
       fflush(stdout);
       std::exit(EXIT_FAILURE);
     }
     prev_worker_ID = "Init";
   }

   // Load the bytecode...
   std::string errorMsg;
   LLVMContext ctx;
   interpModule = klee::loadModule(ctx, InputFile, errorMsg);
   if (taseDebug){
     printf("Module has been loaded...\n");
     std::cout.flush();
   }
   
   if (!interpModule) {
    klee_error("error loading program '%s': %s", InputFile.c_str(),
               errorMsg.c_str());
  }
   ///////////////////////Arg Parsing section

   int pArgc;
   char **pArgv;
   char **pEnvp;
   if (Environ != "") {
     std::vector<std::string> items;
     std::ifstream f(Environ.c_str());
     if (!f.good())
       klee_error("unable to open --environ file: %s", Environ.c_str());
     while (!f.eof()) {
       std::string line;
       std::getline(f, line);
       line = strip(line);
       if (!line.empty())
	 items.push_back(line);
     }
     f.close();
     pEnvp = new char *[items.size()+1];
     unsigned i=0;
     for (; i != items.size(); ++i)
       pEnvp[i] = strdup(items[i].c_str());
     pEnvp[i] = 0;
   } else {
     pEnvp = envp;
   }

   printf("InputArgv: \n");
   for(auto& x : InputArgv){
     printf("  %s\n", x.c_str()k);
   }
   fflush(stdout);
   std::vector<size_t> argsizes;
   pArgc = InputArgv.size() + 1;
   pArgv = new char *[pArgc];
   for (unsigned i=0; i<InputArgv.size()+1; i++) {
     std::string &arg = (i==0 ? InputFile : InputArgv[i-1]);
     printf("Arg: %s\n", arg.c_str());
     unsigned size = arg.size() + 1;
     char *pArg = new char[size];
     argsizes.push_back((size_t) size);
     std::copy(arg.begin(), arg.end(), pArg);
     pArg[size - 1] = 0;
     
     pArgv[i] = pArg;
   }
   ///////////////////// End of Arg Parsing Section

   printf("should have seen Args here");
   printf("Creating interpreter... \n");
   Interpreter::InterpreterOptions IOpts;
   KleeHandler *handler = new KleeHandler(pArgc, pArgv);
   Interpreter *interpreter =
     theInterpreter = Interpreter::create(ctx, IOpts, handler);
   handler->setInterpreter(interpreter);

   std::string LibraryDir = KleeHandler::getRunTimeLibraryPath(argv[0]);
   Interpreter::ModuleOptions Opts(LibraryDir.c_str(), EntryPoint,
                                  /*Optimize=*/OptimizeModule,
                                  /*CheckDivZero=*/CheckDivZero,
                                  /*CheckOvershift=*/CheckOvershift);

   interpreter->setModule(interpModule, Opts);

   //ABH: Entry fn for our purposes is a dummy main function.
   // It's specified in parseltongue86 as dummyMain and
   // as_Z9dummyMainv in "EntryPoint" because of cpp name mangling.  
   Function *entryFn = interpModule->getFunction(EntryPoint);
   if (!entryFn){
     printf("ERROR: Couldn't locate entryFn \n");
     std::exit(EXIT_FAILURE);
   }
   if (taseDebug) {
     printf("Initializing interpretation structures ...\n");
     std::cout.flush();
   }

   printf("Total named md, funcs, aliases, ifuncs: %d, %d, %d \n", interpModule->named_metadata_size(),
	  interpModule->size(), interpModule->alias_size());

   interpModule->named_metadata_empty();
   printf("Calling initializeInterpretationStructures");
   fflush(stdout);
   interpreter->initializeInterpretationStructures(entryFn);
   GlobalInterpreter = interpreter;

   printf("loading cartridges");
   fflush(stdout);
   loadCartridgeInfo();
   loadCartridgeDests();

   
	  

	  
   signal(SIGCHLD, SIG_IGN);
   //--------
#ifdef TASE_OPENSSL

   char * ktestModeName = "-playback";
   memset(ktestMode, 0, sizeof (ktestMode));
   strncpy(ktestMode, ktestModeName, strlen(ktestModeName));

   memset(ktestPath, 0, sizeof(ktestPath));

   if (enableTimeSeries) {
     printf("ERROR: Need to reintegrate code for time series \n");
     std::exit(EXIT_FAILURE);
     //spawnTimeSeriesWorkers();
   } else {
     
     const char *  ktestPathName = verificationLog.c_str();
     strncpy(ktestPath, ktestPathName, strlen(ktestPathName));
   }
   
   #endif
   //--------
   if (taseManager) {
     masterPID = getpid();
     initManagerStructures();
   }
   int pid;

   #ifdef TASE_OPENSSL

   if (!enableTimeSeries) {
     sleep(10); //Let khugepaged catch up before we launch
   }

   #endif TASE_OPENSSL
   signal(SIGCHLD, SIG_IGN); //Added
   double theTime = util::getWallTime();
   target_start_time = theTime;  //Moved here to initialize for both manager and workers
   last_message_verification_time = theTime;
   if (taseManager) 
     pid = ::fork();
   else
     pid =0;
   
   if (pid == 0){
     printf("----------------SWAPPING TO TARGET CONTEXT------------------ \n");
     std::cout.flush();
     if (taseManager) {
       
       if (!noLog) {
	 int i = getpid();
	 worker_ID_stream << ".";
	 worker_ID_stream << i;
	 std::string pidString ;
	 pidString = worker_ID_stream.str();
	 FILE * tmpFile1 = freopen(pidString.c_str(),"w",stdout);
	 if (tmpFile1 == NULL) {
	   printf("FATAL ERROR redirecting stdout \n");
	   fflush(stdout);
	   std::exit(EXIT_FAILURE);
	 }
	 FILE * tmpFile2 = freopen(pidString.c_str(),"w",stderr);
	 if (tmpFile2 == NULL) {
	   printf("FATAL ERROR redirecting stderr \n");
	   fflush(stdout);
	   std::exit(EXIT_FAILURE);
	 }
       }
       
       int res = prctl(PR_SET_CHILD_SUBREAPER, 1);
       if (res == -1)
	 perror("Initial prctl error ");

       
       get_sem_lock();
       *target_started_ptr = 1; //Signals that analysis has started.
       int offset = *ms_QR_size_ptr;
       *(ms_QR_base + offset) = getpid(); //Add self to QR
       offset++;
       *ms_QR_size_ptr = offset;

       #ifndef TASE_OPENSSL
       DFS_push(getpid());

       BFS_enqueue(getpid());
       #endif
       
       release_sem_lock();
     }

     if (taseDebug) {
       printf("Calling transferToTarget() \n");
       std::cout.flush();
     }

     auto exe = static_cast<klee::Executor*>(interpreter);
     exe->tase_map_buf((uint64_t) &pArgc, sizeof(pArgc));
     exe->tase_map_buf((uint64_t) pArgv, sizeof(pArgv)*pArgc);
     for(int i = pArgc-1; i>=0; --i){
       exe->tase_map_buf((uint64_t) pArgv[i], argsizes[i]);
     }

     transferToTarget(pArgc, pArgv);
     printf("RETURNING TO MAIN HANDLER \n");
     return 0;

   } else {
     while (true) {
       manage_workers();
     }
   }
 }
