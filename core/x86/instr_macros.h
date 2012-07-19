/* **********************************************************
 * Copyright (c) 2012 Google, Inc.  All rights reserved.
 * **********************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of Google, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _INSTR_MACROS_H_
#define _INSTR_MACROS_H_ 1

/* NOCHECKIN: Piggy backing on instr_create.h for now.  Should we create our own
 * header?
 */
/* DR_API EXPORT TOFILE dr_ir_macros.h */
/* DR_API EXPORT BEGIN */

#define CHECK_INSTR(instr, ret)                                             \
    (IF_DEBUG_(CLIENT_ASSERT(sizeof(*instr) == sizeof(instr_t),             \
                             "invalid type"))                               \
     IF_DEBUG_(CLIENT_ASSERT((instr)->opcode != OP_UNDECODED, "undecoded")) \
     (ret))

#define INSTR_GET_OPCODE(instr) \
    CHECK_INSTR(instr, (instr)->opcode)

/* If DR_FAST_IR is defined, we're either being included internally by DR, or by
 * a client who doesn't care about ABI compatibiliy.  Re-define the exported IR
 * routines to their macro counterparts.
 */
#ifdef DR_FAST_IR
# define instr_get_opcode INSTR_GET_OPCODE
#endif /* DR_FAST_IR */

/* DR_API EXPORT END */

#endif /* _INSTR_H_ */
