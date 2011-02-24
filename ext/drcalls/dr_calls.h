/* *******************************************************************************
 * Copyright (c) 2010 Massachusetts Institute of Technology  All rights reserved.
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
 * ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _DRCALLS_CALLS_H_
#define _DRCALLS_CALLS_H_ 1

#include "dr_api.h"

DR_EXPORT
/* Initialize the drcalls extension. */
void drcalls_init(void);

DR_EXPORT
/* Uninitialize the drcalls extension. */
void drcalls_exit(void);

DR_EXPORT
/* Insert a clean call to a function before the provided instruction using a
 * shared, out-of-line save and restore routine.
 *
 * This clobbers SPILL_SLOT_1, and SPILL_SLOT_2 and SPILL_SLOT_3 are used for
 * argument passing.  If you wish to materialize your own arguments, you may
 * pass SPILL_SLOT_2 as argument 1 and/or SPILL_SLOT_3 as argument 2.  Any
 * other arrangment of spill slots will corrupt the arguments.
 */
void drcalls_shared_call(void *drcontext, instrlist_t *ilist, instr_t *where,
                         void *callee, uint num_args, ...);

#endif /* _DRCALLS_CALLS_H_ */
