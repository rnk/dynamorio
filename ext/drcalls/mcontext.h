/* *******************************************************************************
 * Copyright (c) 2011 Massachusetts Institute of Technology  All rights reserved.
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

#ifndef DRCALLS_MCONTEXT_H
#define DRCALLS_MCONTEXT_H

#include "core_compat.h"

opnd_t
mc_frame_opnd(uint framesize, uint mc_offset);

void
insert_mc_reg_save(void *dc, uint framesize, instrlist_t *ilist,
                   instr_t *where, reg_id_t reg);
void
insert_mc_reg_restore(void *dc, uint framesize, instrlist_t *ilist,
                      instr_t *where, reg_id_t reg);

void
insert_mc_flags_save(void *dc, uint framesize, instrlist_t *ilist,
                     instr_t *where);
void
insert_mc_flags_restore(void *dc, uint framesize, instrlist_t *ilist,
                        instr_t *where);

opnd_t
opnd_get_tls_xax(void *dc);
uint
get_framesize(instr_t *instr);

void
insert_switch_to_dstack(void *dc, int framesize, instrlist_t *ilist,
                        instr_t *where);
void
insert_switch_to_appstack(void *dc, int framesize, instrlist_t *ilist,
                          instr_t *where);
void
expand_op_dstack(void *dc, instrlist_t *ilist, instr_t *where);
void
expand_op_appstack(void *dc, instrlist_t *ilist, instr_t *where);

void
insert_mc_regs(void *dc, int framesize, bool save, bool save_regs[NUM_GP_REGS],
               instrlist_t *ilist, instr_t *where);
void
insert_mc_xmm_regs(void *dc, int framesize, bool save, instrlist_t *ilist,
                   instr_t *where);

#endif /* DRCALLS_MCONTEXT_H */
