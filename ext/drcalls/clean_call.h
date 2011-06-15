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

#ifndef DRCALLS_CLEANCALL_H
#define DRCALLS_CLEANCALL_H

#include "core_compat.h"

typedef opnd_t (*reg_slot_fn_t)(void *dc, void *val, reg_id_t reg);

/* Default reg slot finder: Pulls registers out of the stack mcontext, and pulls
 * XSP from the TLS XAX slot. */
opnd_t get_mc_reg_slot(void *dc, void *val, reg_id_t reg);

/* Materializes an argument into a register.  reg_slot_fn should return an
 * operand which can be loaded to restore the application register value for use
 * in the argument materialization.
 */
void
materialize_arg_into_reg(void *dc, instrlist_t *ilist, instr_t *where,
                         bool reg_clobbered[NUM_GP_REGS],
                         reg_slot_fn_t reg_slot_fn, void *val, opnd_t arg,
                         reg_id_t reg);
void
insert_clean_call(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                  bool fpstate, uint num_args, opnd_t *args);

#endif /* DRCALLS_CLEANCALL_H */
