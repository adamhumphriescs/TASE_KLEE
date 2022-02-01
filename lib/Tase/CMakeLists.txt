#===------------------------------------------------------------------------===#
#
#                     The KLEE Symbolic Virtual Machine
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
#===------------------------------------------------------------------------===#
klee_add_component(kleeTase
  CVAssignment.cpp
  TASEControl.cpp
  TASESysModels.cpp
  TASEFloatModels.cpp
  TASESSLVerModels.cpp
)

target_link_libraries(kleeTase PRIVATE ${ZLIB_LIBRARIES})

klee_get_llvm_libs(LLVM_LIBS ${LLVM_COMPONENTS})
target_link_libraries(kleeTase PUBLIC ${LLVM_LIBS})

target_link_libraries(kleeTase PRIVATE
  kleeBasic
  kleeCore
  kleeModule
  kleaverSolver
  kleaverExpr
  kleeSupport
  ${CMAKE_SOURCE_DIR}/../build_klee/lib/libtase.a
#  ${CMAKE_SOURCE_DIR}/../build_klee/lib/proj.a
  )
