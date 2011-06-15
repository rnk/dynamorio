/* *******************************************************************************
 * Copyright (c) 2010-2011 Massachusetts Institute of Technology  All rights reserved.
 * *******************************************************************************/

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
 * * Neither the name of MIT nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL MIT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/* Logic for saving and restoring register state to and from mcontexts on
 * dstack.  Can be used inline, in partial-inline slowpaths, and regular clean
 * calls.
 */

#include "dr_api.h"

#include "core_compat.h"
#include "inline.h"

#include <stddef.h> /* offsetof */

/* Maps DR_REG_* enums to offsets into dr_mcontext_t.  Relies on register enum
 * ordering in instr.h.  Indexed by DR_REG_X* - DR_REG_XAX. */
static const uint reg_mc_offset[NUM_GP_REGS] = {
    offsetof(dr_mcontext_t, xax),
    offsetof(dr_mcontext_t, xcx),
    offsetof(dr_mcontext_t, xdx),
    offsetof(dr_mcontext_t, xbx),
    offsetof(dr_mcontext_t, xsp),
    offsetof(dr_mcontext_t, xbp),
    offsetof(dr_mcontext_t, xsi),
    offsetof(dr_mcontext_t, xdi)
#ifdef X64
    ,
    offsetof(dr_mcontext_t, r8 ),
    offsetof(dr_mcontext_t, r9 ),
    offsetof(dr_mcontext_t, r10),
    offsetof(dr_mcontext_t, r11),
    offsetof(dr_mcontext_t, r12),
    offsetof(dr_mcontext_t, r13),
    offsetof(dr_mcontext_t, r14),
    offsetof(dr_mcontext_t, r15)
#endif
};

#define XFLAGS_OFFSET offsetof(dr_mcontext_t, xflags)

/* Turn an offset into dr_mcontext_t into an opnd_t that will access that slot
 * for this callee.  For example, mc_frame_opnd(ci, XAX_OFFSET) gets a memory
 * operand for the XAX slot.
 */
opnd_t
mc_frame_opnd(uint framesize, uint mc_offset)
{
    uint frame_offset = framesize - sizeof(dr_mcontext_t) + mc_offset;
    return OPND_CREATE_MEMPTR(DR_REG_XSP, frame_offset);
}

/* Get an mcontext operand for a given register. */
opnd_t
mc_reg_opnd(uint framesize, reg_id_t reg)
{
    uint frame_offset;
    ASSERT(reg_is_pointer_sized(reg));
    frame_offset = (framesize - sizeof(dr_mcontext_t) +
                    reg_mc_offset[reg - DR_REG_XAX]);
    return OPND_CREATE_MEMPTR(DR_REG_XSP, frame_offset);
}

/* Saves a register to the mcontext at the base of the stack.  Assumes XSP is
 * at dstack - framesize.
 */
void
insert_mc_reg_save(void *dc, uint framesize, instrlist_t *ilist,
                   instr_t *where, reg_id_t reg)
{
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dc, mc_reg_opnd(framesize, reg), opnd_create_reg(reg)));
}

/* Loads a register from the mcontext at the base of the stack.  Assumes XSP is
 * at dstack - framesize.
 */
void
insert_mc_reg_restore(void *dc, uint framesize, instrlist_t *ilist,
                      instr_t *where, reg_id_t reg)
{
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, opnd_create_reg(reg), mc_reg_opnd(framesize, reg)));
}

/* Saves aflags to the mcontext at the base of the stack.  Assumes XSP is at
 * dstack - framesize.  Assumes that XAX is dead and can be used as scratch.
 */
void
insert_mc_flags_save(void *dc, uint framesize, instrlist_t *ilist,
                     instr_t *where)
{
    PRE(ilist, where, INSTR_CREATE_lahf(dc));
    PRE(ilist, where, INSTR_CREATE_setcc
        (dc, OP_seto, opnd_create_reg(DR_REG_AL)));
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dc, mc_frame_opnd(framesize, XFLAGS_OFFSET),
         opnd_create_reg(DR_REG_XAX)));
}

/* Saves aflags to the mcontext at the base of the stack.  Assumes XSP is at
 * dstack - framesize.  Assumes that XAX is dead and can be used as scratch.
 */
void
insert_mc_flags_restore(void *dc, uint framesize, instrlist_t *ilist,
                        instr_t *where)
{
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, opnd_create_reg(DR_REG_XAX),
         mc_frame_opnd(framesize, XFLAGS_OFFSET)));
    PRE(ilist, where, INSTR_CREATE_add
        (dc, opnd_create_reg(DR_REG_AL), OPND_CREATE_INT8(0x7F)));
    PRE(ilist, where, INSTR_CREATE_sahf(dc));
}

/* XXX: Should live in core_compat.c? */
opnd_t
opnd_get_tls_xax(void *dc)
{
    /* XXX: We access the XAX slot by first requesting an opnd for slot 3, which
     * is xbx's slot, and we know xax is one before xbx, so we subtract the size
     * of a slot from the displacement. */
    /* TODO(rnk): This is a shameful way to access XAX, but I don't
     * actually want to expose this information to clients.  */
    opnd_t slot = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_3);
    opnd_set_disp(&slot, opnd_get_disp(slot) - sizeof(reg_t));
    return slot;
}

static opnd_t
opnd_get_tls_dcontext(void *dc)
{
    /* XXX: We access the dcontext slot by first requesting an opnd for slot 1,
     * which is xdx's slot, and we know xdx is one before dcontext, so we add
     * the size of a slot to the displacement. */
    opnd_t slot = dr_reg_spill_slot_opnd(dc, SPILL_SLOT_1);
    opnd_set_disp(&slot, opnd_get_disp(slot) + sizeof(reg_t));
    return slot;
}

static uint
get_dstack_offset(void)
{
    /* XXX: Relying on undocumented DynamoRIO internals: dcontext is not exposed
     * to extensions, but dstack is part of the offset-critical portion of
     * dcontext, so it's offset is unlikely to change.  We hardcode the size
     * here.
     *
     * Layout is the following:
     * dcontext {
     *   upcontext {
     *     priv_mcontext {
     *       gprs
     *       flags
     *       pc
     *       padding
     *       SSE
     *     }
     *     int
     *     bool
     *   }
     *   ptr
     *   ptr
     *   ptr
     *   dstack
     * }
     */
#ifdef X64
# ifdef WINDOWS
#  define NUM_XMM_SLOTS 6 /* xmm0-5 */
# else
#  define NUM_XMM_SLOTS 16 /* xmm0-15 */
# endif
# define PRE_XMM_PADDING 16
#else
# define NUM_XMM_SLOTS 8 /* xmm0-7 */
# define PRE_XMM_PADDING 24
#endif
    uint psz = sizeof(reg_t);
    uint priv_size = IF_X64_ELSE(16, 8) * psz + 2 * psz;
    priv_size += PRE_XMM_PADDING + NUM_XMM_SLOTS * XMM_SAVED_REG_SIZE;
    priv_size = ALIGN_FORWARD_UINT(priv_size + sizeof(int) + sizeof(bool), psz);
    return priv_size + 3 * psz;
}

void
insert_switch_to_dstack(void *dc, int framesize, instrlist_t *ilist,
                        instr_t *where)
{
    PRE(ilist, where, instr_create_0dst_1src
        (dc, DRC_OP_dstack, OPND_CREATE_INT32(framesize)));
}

uint
get_framesize(instr_t *instr)
{
    ASSERT(instr_get_opcode(instr) == DRC_OP_dstack ||
           instr_get_opcode(instr) == DRC_OP_appstack);
    return (uint)opnd_get_immed_int(instr_get_src(instr, 0));
}

void
expand_op_dstack(void *dc, instrlist_t *ilist, instr_t *where)
{
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    uint framesize = get_framesize(where);

    /* Spill XSP to TLS_XAX_SLOT because it's not exposed to the client. */
    PRE(ilist, where, INSTR_CREATE_mov_st(dc, opnd_get_tls_xax(dc), xsp));
    /* Switch to dstack. */
    PRE(ilist, where, INSTR_CREATE_mov_ld(dc, xsp, opnd_get_tls_dcontext(dc)));
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, xsp, OPND_CREATE_MEMPTR(DR_REG_XSP, get_dstack_offset())));
    /* Make room for a partially initialized mcontext
     * structure. */
    PRE(ilist, where, INSTR_CREATE_lea
        (dc, xsp, OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, -framesize)));
}

void
insert_switch_to_appstack(void *dc, int framesize, instrlist_t *ilist,
                          instr_t *where)
{
    PRE(ilist, where, instr_create_0dst_1src
        (dc, DRC_OP_appstack, OPND_CREATE_INT32(framesize)));
}

void
expand_op_appstack(void *dc, instrlist_t *ilist, instr_t *where)
{
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    PRE(ilist, where, INSTR_CREATE_mov_ld(dc, xsp, opnd_get_tls_xax(dc)));
}

/* Save or restore all application registers that weren't saved inline. */
void
insert_mc_regs(void *dc, int framesize, bool save, bool save_regs[NUM_GP_REGS],
               instrlist_t *ilist, instr_t *where)
{
    int i;
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (!save_regs[i])
            continue;
        reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
        if (save)
            insert_mc_reg_save(dc, framesize, ilist, where, reg);
        else {
            /* We save xsp, but do not restore it. */
            if (reg == DR_REG_XSP)
                continue;
            insert_mc_reg_restore(dc, framesize, ilist, where, reg);
        }
    }
}

static uint
move_mm_reg_opcode(bool aligned16, bool aligned32)
{
    if (YMM_ENABLED()) {
        /* must preserve ymm registers */
        return (aligned32 ? OP_vmovdqa : OP_vmovdqu);
    }
    else if (proc_has_feature(FEATURE_SSE2)) {
        return (aligned16 ? OP_movdqa : OP_movdqu);
    } else {
        CLIENT_ASSERT(proc_has_feature(FEATURE_SSE), "running on unsupported processor");
        return (aligned16 ? OP_movaps : OP_movups);
    }
}

void
insert_mc_xmm_regs(void *dc, int framesize, bool save, instrlist_t *ilist,
                   instr_t *where)
{
    /* Save XMM regs, if appropriate. */
    if (dr_mcontext_xmm_fields_valid()) {
        /* PR 264138: we must preserve xmm0-5 on WOW64. */
        /* We align the stack ourselves, so we assume aligned SSE ops are OK. */
        int i;
        uint opcode = move_mm_reg_opcode(/*aligned16=*/true, /*aligned32=*/true);
        for (i = 0; i < NUM_XMM_SAVED; i++) {
            uint mc_offset = offsetof(dr_mcontext_t, ymm) + i * XMM_SAVED_REG_SIZE;
            uint frame_offset = framesize - sizeof(dr_mcontext_t) + mc_offset;
            opnd_t reg = opnd_create_reg(REG_SAVED_XMM0 + (reg_id_t)i);
            opnd_t mem = opnd_create_base_disp(DR_REG_XSP, DR_REG_NULL, 0,
                                               frame_offset, OPSZ_16);
            /* If we're saving, it's reg -> mem, otherwise mem -> reg. */
            opnd_t src = save ? reg : mem;
            opnd_t dst = save ? mem : reg;
            PRE(ilist, where, instr_create_1dst_1src(dc, opcode, dst, src));
        }
    }
}
