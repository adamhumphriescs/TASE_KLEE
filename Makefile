KLEE_BUILD_TYPE=Release+Debug+Asserts
KLEE_ENV=CXXFLAGS="-D_GLIBCXX_USE_CXX11_ABI=0 -fPIC -fno-pie -no-pie -T$(TASE_LINK)" CFLAGS="-fPIC"

KLEE_OPTS=-DCMAKE_INSTALL_PREFIX="$(RUN_DIR)" -DLLVM_CONFIG_BINARY="$(RUN_DIR)/llvm-3.4.2/bin/llvm-config" -DLLVMCC="$(CLANG)" -DLLVMCXX="$(CLANG)++" -DENABLE_KLEE_UCLIBC=FALSE -DENABLE_POSIX_RUNTIME=FALSE -DENABLE_SOLVER_STP=TRUE -DSTP_DIR="$(BUILD_DIR)/build/stp" -DCMAKE_BUILD_TYPE=Release -DENABLE_KLEE_ASSERTS=FALSE -DENABLE_UNIT_TESTS=FALSE -DENABLE_SYSTEM_TESTS=FALSE -DENABLE_DOCS=FALSE -DLLVM_INCLUDE_DIRS="$(INCLUDE_DIR)/llvm-3.4.2/include/" -DLLVM_LIBRARY_DIRS="$(INCLUDE_DIR)/llvm-3.4.2/lib/" -DKLEE_RUNTIME_BUILD_TYPE=$(KLEE_BUILD_TYPE) -DENABLE_SOLVER_Z3=OFF -DTASE_INCLUDE_DIR="$(INCLUDE_DIR)/tase/" -DOPENSSL_INCLUDE_DIR="$(BUILD_DIR)/openssl/include/"

LIBS=$(addprefix $(RUN_DIR)/lib/lib,$(addsuffix .a,$(KLEE_LIBS)))
BITCODE=$(addprefix $(BUILD_DIR)/build/klee/$(KLEE_BUILD_TYPE)/lib/,$(notdir $(KLEE_BITCODE)))

all: $(BUILD_DIR)/build/klee/bin/klee $(LIBS) $(KLEE_BITCODE) $(RUN_DIR)/lib/main.cpp.o

$(BUILD_DIR)/build/klee/bin/klee:
	mkdir -p $(BUILD_DIR)/build/klee/lib/
	cp $(RUN_DIR)/lib/$(LIBTASE) $(BUILD_DIR)/build/klee/lib/libtase.a
	cd $(BUILD_DIR)/build/klee/ && $(KLEE_ENV) cmake $(KLEE_OPTS) $(BUILD_DIR)/klee/
	cp link.txt $(BUILD_DIR)/build/klee/tools/klee/CMakeFiles/klee.dir/link.txt
	$(MAKE) -C $(BUILD_DIR)/build/klee/


$(LIBS): $(RUN_DIR)/lib/%.a: $(BUILD_DIR)/build/klee/lib/%.a
	cp $< $@


$(addprefix $(BUILD_DIR)/build/klee/lib/lib,$(addsuffix .a,$(KLEE_LIBS))): $(BUILD_DIR)/build/klee/bin/klee


$(KLEE_BITCODE): $(RUN_DIR)/install/klee_bitcode/%.bc: $(BUILD_DIR)/build/klee/$(KLEE_BUILD_TYPE)/lib/%.bc
	cp $< $@


$(BITCODE): $(BUILD_DIR)/build/klee/bin/klee $(RUN_DIR)/install/klee_bitcode


$(RUN_DIR)/install/klee_bitcode:
	mkdir -p $(RUN_DIR)/install/klee_bitcode/


$(RUN_DIR)/lib/main.cpp.o: $(BUILD_DIR)/build/klee/tools/klee/CMakeFiles/klee.dir/main.cpp.o
	cp $< $@

$(BUILD_DIR)/build/klee/tools/klee/CMakeFiles/klee.dir/main.cpp.o: $(BUILD_DIR)/build/klee/bin/klee
