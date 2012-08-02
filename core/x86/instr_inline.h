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

#ifndef _INSTR_INLINE_H_
#define _INSTR_INLINE_H_ 1

/* DR_API EXPORT TOFILE dr_ir_instr.h */
/* DR_API EXPORT BEGIN */

#ifdef DR_FAST_IR

/* These macros have no checks.  If the client wants assertions, they can use a
 * debug build and turn of DR_FAST_IR.
 */

#define INSTR_NUM_SRCS(instr) (instr)->num_srcs
#define INSTR_NUM_DSTS(instr) (instr)->num_dsts
#define INSTR_GET_DST(instr, pos) (instr)->dsts[pos]
/* src0 is static, rest are dynamic. */
/* FIXME: Double evaluation. */
#define INSTR_GET_SRC(instr, pos) \
     ((pos) == 0 ? (instr)->src0 : (instr)->srcs[(pos) - 1])
#define INSTR_GET_TARGET(instr) (instr)->src0

/* API EXPORT END */
#ifndef DEBUG
/* API EXPORT BEGIN */
#define instr_num_srcs INSTR_NUM_SRCS
#define instr_num_dsts INSTR_NUM_DSTS
#define instr_get_src INSTR_GET_SRC
#define instr_get_dst INSTR_GET_DST
#define instr_get_target INSTR_GET_TARGET
#endif

INSTR_INLINE
void
instr_set_note(instr_t *instr, void *value)
{
    instr->note = value;
}

INSTR_INLINE
void *
instr_get_note(instr_t *instr)
{
    return instr->note;
}

INSTR_INLINE
instr_t*
instr_get_next(instr_t *instr)
{
    return instr->next;
}

INSTR_INLINE
instr_t*
instr_get_prev(instr_t *instr)
{
    return instr->prev;
}

INSTR_INLINE
void
instr_set_next(instr_t *instr, instr_t *next)
{
    instr->next = next;
}

INSTR_INLINE
void
instr_set_prev(instr_t *instr, instr_t *prev)
{
    instr->prev = prev;
}

#endif /* DR_FAST_IR */

/* DR_API EXPORT END */

#endif /* _INSTR_INLINE_H_ */
