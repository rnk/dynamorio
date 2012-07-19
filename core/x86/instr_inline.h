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

#ifdef AVOID_API_EXPORT
/* Internally DR has multiple levels or IR, but once it gets to a client, we
 * assume it's already level 3 or higher, and we don't need to do any checks.
 * Furthermore, instr_decode() and get_thread_private_dcontext() are not
 * exported.
 * FIXME: If this is getting inlined everywhere, we should drop the
 * get_thread_private_dcontext() at the call site to make the cold path smaller.
 */
# define CHECK_OPNDS_VALID(instr) \
    if ((instr->flags & INSTR_OPERANDS_VALID) == 0) \
        instr_decode_with_current_dcontext(instr)
#endif

/* Turn off checks if a client includes us with DR_FAST_IR.  We can't call the
 * internal routines we'd use for these checks anyway.
 */
#ifdef API_EXPORT_ONLY
#define CHECK_OPNDS_VALID(instr)
#define CLIENT_ASSERT(cond, msg)
#endif

INSTR_INLINE
int
instr_num_srcs(instr_t *instr)
{
    CHECK_OPNDS_VALID(instr);
    return instr->num_srcs;
}

INSTR_INLINE
int
instr_num_dsts(instr_t *instr)
{
    CHECK_OPNDS_VALID(instr);
    return instr->num_dsts;
}

INSTR_INLINE
opnd_t
instr_get_src(instr_t *instr, uint pos)
{
    CHECK_OPNDS_VALID(instr);
    CLIENT_ASSERT(pos >= 0 && pos < instr->num_srcs,
                  "instr_get_src: ordinal invalid");
    if (pos == 0)
        return instr->src0;
    else
        return instr->srcs[pos-1];
}

INSTR_INLINE
opnd_t
instr_get_dst(instr_t *instr, uint pos)
{
    CHECK_OPNDS_VALID(instr);
    CLIENT_ASSERT(pos >= 0 && pos < instr->num_dsts,
                  "instr_get_dst: ordinal invalid");
    return instr->dsts[pos];
}

INSTR_INLINE
opnd_t
instr_get_target(instr_t *cti_instr)
{
    CHECK_OPNDS_VALID(cti_instr);
    CLIENT_ASSERT(instr_is_cti(cti_instr),
                  "instr_get_target called on non-cti");
    CLIENT_ASSERT(cti_instr->num_srcs >= 1,
                  "instr_get_target: instr has no sources");
    return cti_instr->src0;
}

INSTR_INLINE
/* set the note field of instr to value */
void 
instr_set_note(instr_t *instr, void *value)
{
    instr->note = value;
}

INSTR_INLINE
/* return the note field of instr */
void *
instr_get_note(instr_t *instr)
{
    return instr->note;
}

INSTR_INLINE
/* return instr->next */
instr_t*
instr_get_next(instr_t *instr)
{
    return instr->next;
}

INSTR_INLINE
/* return instr->prev */
instr_t*
instr_get_prev(instr_t *instr)
{
    return instr->prev;
}

INSTR_INLINE
/* set instr->next to next */
void
instr_set_next(instr_t *instr, instr_t *next)
{
    instr->next = next;
}

INSTR_INLINE
/* set instr->prev to prev */
void
instr_set_prev(instr_t *instr, instr_t *prev)
{
    instr->prev = prev;
}

#endif /* DR_FAST_IR */

/* DR_API EXPORT END */

#endif /* _INSTR_INLINE_H_ */
