# *******************************************************************************
# Copyright (c) 2010 Massachusetts Institute of Technology  All rights reserved.
# *******************************************************************************

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of MIT nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

cmake_minimum_required(VERSION 2.6)

# High level library used by clients to insert instrumentation.
add_library(drcalls STATIC
  calls.c       # exported routines go here
  code_cache.c
  lean_call.c
  callee.c
  optimize.c
  inline.c
  mcontext.c
  clean_call.c
  # add more here
  )
configure_DynamoRIO_client(drcalls)
use_DynamoRIO_extension(drcalls drcontainers)
get_target_property(cflags drcalls COMPILE_FLAGS)
if (DEBUG)
  set(cflags "${cflags} -DDEBUG")
endif ()
if (UNIX)
  # static extensions must be PIC to be linked into clients: else requires
  # relocations that run afoul of security policies, etc.
  set(cflags "${cflags} -fPIC")
endif (UNIX)
set_target_properties(drcalls PROPERTIES COMPILE_FLAGS "${cflags}")

# Clients used for benchmarking.
set(BENCH_CLIENTS
  inscount_cleancall
  inscount_leancall
  inscount_manual
  mem_buffer
  alignment
  )
foreach (client ${BENCH_CLIENTS})
  set(srcs "bench_clients/${client}.c")
  set(client "drcalls_bench.${client}")
  if ("${client}" MATCHES "^inscount_.*$")
    set (srcs ${srcs} "bench_clients/inscount_common.c")
  endif ()
  add_library(${client} SHARED ${srcs})
  configure_DynamoRIO_client(${client})
  use_DynamoRIO_extension(${client} drcalls)
  get_target_property(cflags ${client} COMPILE_FLAGS)
  set(cflags "${cflags} -O2")
  set_target_properties(${client} PROPERTIES COMPILE_FLAGS "${cflags}")
endforeach ()


if (BUILD_TESTS)
  # Unit tests.  These are finer grained than the system tests in suite/tests.
  # They also break the abstraction barrier, and if we go to shared libraries, may
  # need macro magic or an export list to get access to internal functions.
  set(UNIT_TESTS
    optimize_test
    )
  foreach (test ${UNIT_TESTS})
    # exe and test targets must have different names or things explode.
    set(src "${test}.c")
    set(exe "drcalls.${test}")
    set(test "unittest|drcalls.${test}")
    add_executable(${exe} ${src})
    configure_DynamoRIO_standalone(${exe})
    target_link_libraries(${exe} drcalls)
    add_test(NAME ${test} COMMAND ${exe})
  endforeach ()
endif ()

export(TARGETS drcalls
  FILE ${PROJECT_BINARY_DIR}/cmake/${exported_targets_name}.cmake
  APPEND)
install(TARGETS drcalls EXPORT ${exported_targets_name}
  DESTINATION ${INSTALL_EXT_LIB})
install(FILES
  dr_calls.h
  # add more here
  DESTINATION ${INSTALL_EXT_INCLUDE})
