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

#ifdef _DR_INSTR_INLINE_H_
# error "should not be double included"
#endif
#define _DR_INSTR_INLINE_H_

#include "decode.h"

/*************************
 ***       opnd_t        ***
 *************************/

/* TODO cleanup */
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect((x), 0)


FOO int
instr_get_opcode(instr_t *instr)
{
    if (UNLIKELY(instr->opcode == OP_UNDECODED)) {
        instr_decode_opcode(get_thread_private_dcontext(), instr);
    }
    return instr->opcode;
}
/* predicates */

FOO bool opnd_is_valid(opnd_t opnd)
{
    return LIKELY(opnd.kind < LAST_kind);
}
FOO bool opnd_is_null(opnd_t opnd) { return opnd.kind == NULL_kind; }
FOO bool opnd_is_reg(opnd_t opnd) { return opnd.kind == REG_kind; }
FOO bool opnd_is_immed(opnd_t opnd) { return opnd.kind == IMMED_INTEGER_kind ||
                               opnd.kind == IMMED_FLOAT_kind; }
FOO bool opnd_is_immed_int(opnd_t opnd) { return opnd.kind == IMMED_INTEGER_kind; }
FOO bool opnd_is_immed_float(opnd_t opnd) { return opnd.kind == IMMED_FLOAT_kind; }
FOO bool opnd_is_pc(opnd_t opnd) {
    return opnd.kind == PC_kind || opnd.kind == FAR_PC_kind; 
}
FOO bool opnd_is_near_pc(opnd_t opnd) { return opnd.kind == PC_kind; }
FOO bool opnd_is_far_pc(opnd_t opnd) { return opnd.kind == FAR_PC_kind; }
FOO bool opnd_is_instr(opnd_t opnd) {
    return opnd.kind == INSTR_kind || opnd.kind == FAR_INSTR_kind;
}
FOO bool opnd_is_near_instr(opnd_t opnd) { return opnd.kind == INSTR_kind; }
FOO bool opnd_is_far_instr(opnd_t opnd) { return opnd.kind == FAR_INSTR_kind; }
FOO bool opnd_is_base_disp(opnd_t opnd) { return opnd.kind == BASE_DISP_kind; }

/* We allow overlap between ABS_ADDR_kind and BASE_DISP_kind w/ no base or index */
/* TODO don't export */
FOO bool
opnd_is_abs_base_disp(opnd_t opnd) {
    return (opnd_is_base_disp(opnd) && opnd_get_base(opnd) == REG_NULL &&
            opnd_get_index(opnd) == REG_NULL);
}
FOO bool opnd_is_abs_addr(opnd_t opnd) {
    return IF_X64(opnd.kind == ABS_ADDR_kind ||) opnd_is_abs_base_disp(opnd);
}
FOO bool opnd_is_near_abs_addr(opnd_t opnd) {
    return opnd_is_abs_addr(opnd) && opnd.seg.segment == REG_NULL; 
}
FOO bool opnd_is_far_abs_addr(opnd_t opnd) {
    return opnd_is_abs_addr(opnd) && opnd.seg.segment != REG_NULL; 
}

/* null operands */

FOO opnd_t
opnd_create_null(void)
{
    opnd_t opnd;
    opnd.kind = NULL_kind;
    return opnd;
}

/* register operands */

FOO opnd_t
opnd_create_reg(reg_id_t r)
{
    opnd_t opnd IF_DEBUG(= {0});  /* FIXME: Needed until i#417 is fixed. */
    CLIENT_ASSERT(r <= REG_LAST_ENUM && r != REG_INVALID,
                  "opnd_create_reg: invalid register");
    opnd.kind = REG_kind;
    opnd.value.reg = r;
    return opnd;
}

FOO reg_id_t
opnd_get_reg(opnd_t opnd)
{
    CLIENT_ASSERT(opnd_is_reg(opnd), "opnd_get_reg called on non-reg opnd");
    return opnd.value.reg;
}

FOO reg_id_t
opnd_get_base(opnd_t opnd)
{
#ifdef X64
    if (LIKELY(opnd_is_base_disp(opnd)))
        return opnd.value.base_disp.base_reg;
    CLIENT_ASSERT(opnd_is_abs_addr(opnd), "ZZZ");
    return REG_NULL;
#else
    CLIENT_ASSERT(opnd_is_base_disp(opnd), "ZZZ");
    return opnd.value.base_disp.base_reg;
#endif
}

FOO int
opnd_get_disp(opnd_t opnd)
{
    CLIENT_ASSERT(opnd_is_base_disp(opnd),
                  "opnd_get_disp called on invalid opnd type");
    return opnd.value.base_disp.disp;
}

FOO bool
instr_is_cbr(instr_t *instr)      /* conditional branch */
{
    int opc = instr_get_opcode(instr);
    return ((opc >= OP_jo && opc <= OP_jnle) ||
            (opc >= OP_jo_short && opc <= OP_jnle_short) ||
            (opc >= OP_loopne && opc <= OP_jecxz));
}

FOO bool
instr_is_mbr(instr_t *instr)      /* multi-way branch */
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_jmp_ind ||
            opc == OP_call_ind ||
            opc == OP_ret ||
            opc == OP_jmp_far_ind ||
            opc == OP_call_far_ind ||
            opc == OP_ret_far ||
            opc == OP_iret);
}

FOO bool
instr_is_far_cti(instr_t *instr) /* target address has a segment and offset */
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_jmp_far ||
            opc == OP_call_far ||
            opc == OP_jmp_far_ind ||
            opc == OP_call_far_ind ||
            opc == OP_ret_far ||
            opc == OP_iret);
}

FOO bool
instr_is_far_abs_cti(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_jmp_far || opc == OP_call_far);
}

FOO bool
instr_is_ubr(instr_t *instr)      /* unconditional branch */
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_jmp ||
            opc == OP_jmp_short ||
            opc == OP_jmp_far);
}

FOO bool 
instr_is_mov(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_mov_st ||
            opc == OP_mov_ld ||
            opc == OP_mov_imm ||
            opc == OP_mov_seg ||
            opc == OP_mov_priv);
}

FOO bool 
instr_is_call(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_call ||
            opc == OP_call_far ||
            opc == OP_call_ind ||
            opc == OP_call_far_ind);
}

FOO bool 
instr_is_call_direct(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_call || opc == OP_call_far);
}

FOO bool 
instr_is_call_indirect(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_call_ind || opc == OP_call_far_ind);
}

FOO bool
instr_is_return(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    return (opc == OP_ret || opc == OP_ret_far || opc == OP_iret);
}

FOO bool 
instr_is_cti_loop(instr_t *instr)
{
    int opc = instr_get_opcode(instr);
    /* only looking for loop* and jecxz */
    return (opc >= OP_loopne && opc <= OP_jecxz);
}

FOO void
instr_decode_if_needed(instr_t *instr)
{
    if ((instr->flags & INSTR_OPERANDS_VALID) == 0)
        instr_decode(get_thread_private_dcontext(), instr);
}

FOO int
instr_num_srcs(instr_t *instr)
{
    instr_decode_if_needed(instr);
    return instr->num_srcs;
}

FOO int
instr_num_dsts(instr_t *instr)
{
    instr_decode_if_needed(instr);
    return instr->num_dsts;
}

/* Returns the pos-th source operand of instr.
 * If instr's operands are not decoded, goes ahead and decodes them.
 * Assumes that instr is a single instr (i.e., NOT Level 0).
 */
FOO opnd_t
instr_get_src(instr_t *instr, uint pos)
{
    instr_decode_if_needed(instr);
    CLIENT_ASSERT(pos >= 0 && pos < instr->num_srcs, "instr_get_src: ordinal invalid");
    /* remember that src0 is static, rest are dynamic */
    if (pos == 0)
        return instr->src0;
    else
        return instr->srcs[pos-1];
}

/* returns the dst opnd at position pos in instr */
FOO opnd_t
instr_get_dst(instr_t *instr, uint pos)
{
    instr_decode_if_needed(instr);
    CLIENT_ASSERT(pos >= 0 && pos < instr->num_dsts, "instr_get_dst: ordinal invalid");
    return instr->dsts[pos];
}

FOO bool
instr_needs_encoding(instr_t *instr)
{
    return ((instr->flags & INSTR_RAW_BITS_VALID) == 0);
}
