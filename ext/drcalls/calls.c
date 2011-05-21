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

#include "dr_api.h"
#include "dr_calls.h"
#include "hashtable.h"

#include <limits.h> /* for INT_MAX */
#include <stdarg.h> /* for varargs */
#include <stddef.h> /* for offsetof */
#include <string.h> /* for memcmp */

/* internal includes */
#include "code_cache.h"
#include "core_compat.h"
#include "lean_call.h"

#define VERBOSE 0

/* Analysis results for functions called. */
typedef struct _callee_info_t {
    bool bailout;             /* if we bail out on function analysis */
    uint num_args;            /* number of args the callee takes */
    int num_instrs;           /* total number of instructions of a function */
    app_pc start;             /* entry point of a function  */
    int num_xmms_used;        /* number of xmms used by callee */
    bool xmm_used[NUM_XMM_REGS];  /* xmm/ymm registers usage */
    int num_regs_used;        /* number of regs used by callee */
    bool reg_used[NUM_GP_REGS];   /* general purpose registers usage */
    bool callee_save_regs[NUM_GP_REGS]; /* callee-save registers */
    int frame_size;           /* size of stack frame adjustment for locals */
    bool has_locals;          /* if reference local via statck */
    bool xbp_is_fp;           /* if xbp is used as frame pointer */
    bool opt_inline;          /* can be inlined or not */
    bool opt_partial;         /* can be partially inlined */
    bool write_aflags;        /* if the function changes aflags */
    bool read_aflags;         /* if the function reads aflags from caller */
    bool tls_used;            /* application accesses TLS (errno, etc.) */
    bool has_cti;             /* true if callee has any control flow */
    bool is_leaf;             /* true if all control flow is within callee */
    bool stack_complex;       /* true if the stack usage is complicated */
    instrlist_t *ilist;       /* instruction list of function for inline. */
    instr_t *partial_label;   /* label of slowpath entry */
    app_pc partial_pc;      /* pc of slowpath entry */
    uint framesize;           /* size of the frame on dstack */
} callee_info_t;

/* Call-site specific information. */
typedef struct _clean_call_info_t {
    void *callee;
    uint num_args;
    bool save_fpstate;
    bool opt_inline;
    bool should_align;
    bool save_all_regs;
    bool skip_save_aflags;
    bool skip_clear_eflags;
    uint num_xmms_skip;
    bool xmm_skip[NUM_XMM_REGS];
    uint num_regs_skip;
    bool reg_skip[NUM_GP_REGS];
    void *callee_info;  /* callee information */
    instrlist_t *ilist; /* instruction list for inline optimization */
} clean_call_info_t;

static callee_info_t     default_callee_info;
static clean_call_info_t default_clean_call_info;

/* Optimization level of calls. */
/* TODO(rnk): Expose this via flags. */
static uint opt_cleancall = 3;

/* Prototypes for analysis data structures. */
static void callee_info_init(callee_info_t *ci);
static void callee_info_table_init(void);
static void callee_info_table_destroy(void);
static void clean_call_info_init(clean_call_info_t *cci, void *callee,
                                 bool save_fpstate, uint num_args);

/* Hashtable for storing callee analysis info. */
static hashtable_t *callee_info_table;
/* We only free callee info at exit, when callee_info_table_exit is true. */
static bool callee_info_table_exit = false;
#define CALLEE_INFO_TABLE_BITS 6

static bool drcalls_initialized = false;

/* Inlining prototypes. */
static bool analyze_clean_call(void *dcontext, clean_call_info_t *cci,
                               instr_t *where, void *callee, bool save_fpstate,
                               uint num_args, opnd_t *args);
static void insert_inline_clean_call(void *dcontext, clean_call_info_t *cci,
                                     instrlist_t *ilist, instr_t *where,
                                     opnd_t *args);

void
drcalls_init(void)
{
    callee_info_init(&default_callee_info);
    clean_call_info_init(&default_clean_call_info, NULL, false, 0);

    code_cache_init();
    callee_info_table_init();
    lean_call_init();

    drcalls_initialized = true;
}

static void
check_init(void)
{
    DR_ASSERT_MSG(drcalls_initialized,
                  "Used drcalls before calling drcalls_init\n");
}

void
drcalls_exit(void)
{
    check_init();
    callee_info_table_destroy();
    lean_call_exit();
    code_cache_destroy();
    drcalls_initialized = false;
}

static void
convert_va_list_to_opnd(opnd_t *args, uint num_args, va_list ap)
{
    uint i;
    /* There's no way to check num_args vs actual args passed in, or publicly
     * check opnd_t for validity. */
    for (i = 0; i < num_args; i++) {
        args[i] = va_arg(ap, opnd_t);
    }
}

void
drcalls_insert_call(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                    bool fpstate, uint num_args, ...)
{
    va_list ap;
    opnd_t *args = NULL;
    clean_call_info_t cci; /* information for clean call insertion. */

    check_init();

    /* Read arguments. */
    if (num_args > 0)
        args = dr_thread_alloc(dc, sizeof(opnd_t) * num_args);
    va_start(ap, num_args);
    convert_va_list_to_opnd(args, num_args, ap);
    va_end(ap);

    if (analyze_clean_call(dc, &cci, where, callee, fpstate, num_args, args)) {
        /* See if we can inline. */
        insert_inline_clean_call(dc, &cci, ilist, where, args);
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: inlined callee "PFX"\n", callee);
    } else {
        /* Otherwise, just use a clean call. */
        dr_insert_clean_call_vargs(dc, ilist, where, callee, fpstate, num_args,
                                   args);
    }

    if (num_args > 0)
        dr_thread_free(dc, args, sizeof(opnd_t) * num_args);
}

/* TODO(rnk): Should take save_fpstate. */
void
drcalls_lean_call(void *dc, instrlist_t *ilist, instr_t *where, void *callee,
                  uint num_args, ...)
{
    va_list ap;
    opnd_t *args = NULL;
    bool fpstate = false;

    check_init();

    if (num_args > 0) {
        args = dr_thread_alloc(dc, sizeof(opnd_t) * num_args);
        va_start(ap, num_args);
        convert_va_list_to_opnd(args, num_args, ap);
        va_end(ap);
    }

    /* If there are more args than TLS spill slots, give up and insert a normal
     * clean call. */
    if (num_args > 2) {
        dr_log(dc, LOG_ALL, 1, "drcalls: too many arguments for lean call, "
               "performance may suffer.\n");
        dr_insert_clean_call_vargs(dc, ilist, where, callee, fpstate, num_args,
                                   args);
    } else {
        lean_call_insert(dc, ilist, where, callee, fpstate, num_args, args);
    }

    if (num_args > 0) {
        dr_thread_free(dc, args, sizeof(opnd_t) * num_args);
    }
}

/****************************************************************************/
/* Callee info manipulation functions. */

static void
callee_info_init(callee_info_t *ci)
{
    uint i;
    memset(ci, 0, sizeof(*ci));
    ci->bailout = true;
    /* to be conservative */
    ci->has_locals    = true;
    ci->write_aflags  = true;
    ci->read_aflags   = true;
    ci->tls_used      = true;
    ci->stack_complex = true;
    ci->has_cti       = true;
    /* We use loop here and memset in analyze_callee_regs_usage later.
     * We could reverse the logic and use memset to set the value below,
     * but then later in analyze_callee_regs_usage, we have to use the loop.
     */
    /* assuming all xmm registers are used */
    ci->num_xmms_used = NUM_XMM_REGS;
    for (i = 0; i < NUM_XMM_REGS; i++)
        ci->xmm_used[i] = true;
    /* assuming all gp registers are used */
    ci->num_regs_used = NUM_XMM_REGS;
    for (i = 0; i < NUM_GP_REGS; i++)
        ci->reg_used[i] = true;
    for (i = 0; i < NUM_GP_REGS; i++)
        ci->reg_used[i] = true;
    /* The frame should hold at least the mcontext. */
    ci->framesize = sizeof(dr_mcontext_t);
}

static callee_info_t *
callee_info_create(app_pc start, uint num_args)
{
    callee_info_t *ci;

    ci = dr_global_alloc(sizeof(*ci));
    callee_info_init(ci);
    ci->start = start;
    ci->num_args = num_args;
    return ci;
}

static void
callee_info_free(callee_info_t *ci)
{
    ASSERT(callee_info_table_exit);
    if (ci->ilist != NULL) {
        ASSERT(ci->opt_inline);
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
    }
    dr_global_free(ci, sizeof(*ci));
}

static void
callee_info_table_init(void)
{
    callee_info_table = dr_global_alloc(sizeof(*callee_info_table));
    hashtable_init_ex(callee_info_table,
                      CALLEE_INFO_TABLE_BITS,
                      HASH_INTPTR,
                      /* str_dup */ false,
                      /* sync */ false,
                      (void(*)(void*)) callee_info_free,
                      /* hash key */ NULL,
                      /* cmp key */ NULL
                      );
}

static void
callee_info_table_destroy(void)
{
    callee_info_table_exit = true;
    hashtable_delete(callee_info_table);
    dr_global_free(callee_info_table, sizeof(*callee_info_table));
}

static callee_info_t *
callee_info_table_lookup(void *callee)
{
    callee_info_t *ci;
    hashtable_lock(callee_info_table);
    ci = hashtable_lookup(callee_info_table, callee);
    hashtable_unlock(callee_info_table);
    /* We only delete the callee_info from the callee_info_table when destroy
     * the table on exit, so we can keep the ci without holding the lock.
     */
    return ci;
}

static callee_info_t *
callee_info_table_add(callee_info_t *new_ci)
{
    callee_info_t *ci;
    hashtable_lock(callee_info_table);
    ci = hashtable_lookup(callee_info_table, new_ci->start);
    if (ci == NULL) {
        ci = new_ci;
        hashtable_add(callee_info_table, new_ci->start, new_ci);
    } else {
        /* Have one in the table, free the new one and use existing one. 
         * We cannot free the existing one in the table as it might be used by 
         * other thread without holding the lock.
         * Since we assume callee should never be changed, they should have
         * the same content of ci.
         */
        callee_info_free(new_ci);
    }
    hashtable_unlock(callee_info_table);
    return ci;
}

static void
clean_call_info_init(clean_call_info_t *cci, void *callee,
                     bool save_fpstate, uint num_args)
{
    memset(cci, 0, sizeof(*cci));
    cci->callee        = callee;
    cci->num_args      = num_args;
    cci->save_fpstate  = save_fpstate;
    cci->save_all_regs = true;
    cci->should_align  = true;
    cci->callee_info   = &default_callee_info;
}

/****************************************************************************/
/* clean call optimization code */

static app_pc emit_partial_slowpath(void *dc, callee_info_t *ci);

/* The max number of instructions try to decode from a function. */
#define MAX_NUM_FUNC_INSTRS 1024
/* the max number of instructions the callee can have for inline. */
#define MAX_NUM_INLINE_INSTRS 20
#define NUM_SCRATCH_SLOTS 6 /* XXX: Can support more now that we use dstack. */

/* Decode instruction from callee and return the next_pc to be decoded. */
static app_pc
decode_callee_instr(void *dcontext, callee_info_t *ci, app_pc instr_pc)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *instr;
    app_pc   next_pc = NULL;

    instr = instr_create(GLOBAL_DCONTEXT);
    instrlist_append(ilist, instr);
    ci->num_instrs++;
    TRY_EXCEPT(dcontext, {
        next_pc = decode(GLOBAL_DCONTEXT, instr_pc, instr);
    }, { /* EXCEPT */
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: crash on decoding callee instruction at: "PFX"\n",
               instr_pc);
        ASSERT_CURIOSITY(false && "crashed while decoding clean call");
        ci->bailout = true;
        return NULL;
    });
    if (!instr_valid(instr)) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: decoding invalid instruction at: "PFX"\n", instr_pc);
        ci->bailout = true;
        return NULL;
    }
    instr_set_translation(instr, instr_pc);
    DOLOG(3, LOG_CLEANCALL, {
        disassemble_with_info(dcontext, instr_pc, dr_get_logfile(dcontext),
                              true, true);
    });
    return next_pc;
}

/* Uses a heuristic to decide if the target pc is reasonably within the current
 * callee.  If the callee tail calls to a nearby helper, we don't really mind if
 * we end up including the helper in the callee.
 */
static bool
tgt_pc_in_callee(app_pc tgt_pc, callee_info_t *ci)
{
    return ((ptr_uint_t)ci->start <= (ptr_uint_t)tgt_pc &&
            (ptr_uint_t)tgt_pc < (ptr_uint_t)ci->start + 4096);
}

static void
resolve_internal_brs(void *dcontext, callee_info_t *ci)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *cti, *tgt;
    app_pc   tgt_pc;

    /* Check that no target pc of any branch is in a middle of an instruction,
     * and replace target pc with label before the target instr.
     */
    for (cti = instrlist_first(ilist); cti != NULL; cti = instr_get_next(cti)) {
        instr_t *label;
        /* Ignore indirect branches, syscalls, calls, etc for now.  They may be
         * part of code that we choose not to inline. */
        if (!(instr_is_cbr(cti) || instr_is_ubr(cti)))
            continue;
        tgt_pc = opnd_get_pc(instr_get_target(cti));
        if (!tgt_pc_in_callee(tgt_pc, ci))
            continue;  /* Probably a tail call.  We've logged it previously. */
        for (tgt  = instrlist_first(ilist);
             tgt != NULL;
             tgt  = instr_get_next(tgt)) {
            if (tgt_pc == instr_get_app_pc(tgt))
                break;
        }
        if (tgt == NULL) {
            /* cannot find a target instruction, bail out */
            dr_log(dcontext, LOG_CLEANCALL, 2,
                   "CLEANCALL: bail out on strange internal branch at: "PFX
                   " to "PFX"\n", instr_get_app_pc(cti), tgt_pc);
            ci->bailout = true;
            break;
        }
        label = INSTR_CREATE_label(GLOBAL_DCONTEXT);
        instr_set_translation(label, instr_get_app_pc(tgt));
        instrlist_preinsert(ilist, tgt, label);
        instr_set_target(cti, opnd_create_instr(label));
    }
}

/* Check if there is any control flow we couldn't follow.
 */
static void
analyze_callee_cti(void *dc, callee_info_t *ci)
{
    bool is_leaf = true;
    bool has_cti = false;
    instr_t *instr;
    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        if (instr_is_syscall(instr) || instr_is_interrupt(instr)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: not leaf, syscall or interrupt at "PFX"\n",
                   instr_get_app_pc(instr));
            is_leaf = false;
            has_cti = true;  /* syscalls etc. count as control flow. */
        }
        if (!instr_is_cti(instr))
            continue;
        /* XXX: We assume callees do not push arbitrary return addresses and ret
         * to them.  */
        if (instr_is_return(instr))
            continue;
        has_cti = true;
        if (instr_is_call(instr) || instr_is_mbr(instr)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: not leaf, call or indirect jump at "PFX"\n",
                   instr_get_app_pc(instr));
            is_leaf = false;
        }
        if ((instr_is_cbr(instr) || instr_is_ubr(instr)) &&
            opnd_is_pc(instr_get_target(instr))) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: not leaf, unresolved jump at "PFX"\n",
                   instr_get_app_pc(instr));
            is_leaf = false;
        }
    }
    ci->is_leaf = is_leaf;
    ci->has_cti = has_cti;
}

static bool
instr_is_mov_ld_xsp(instr_t *instr)
{
    return (instr_get_opcode(instr) == OP_mov_ld &&
            opnd_same(instr_get_src(instr, 0),
                      OPND_CREATE_MEMPTR(DR_REG_XSP, 0)));
}

/* Rewrite PIC code to materialize application PC directly.  Two major PIC
 * styles:
 * 1. call next_pc; pop r1;
 * 2. call pic_func;
 *    and in pic_func: mov [%xsp] %r1; ret;
 */
static void
rewrite_pic_code(void *dcontext, callee_info_t *ci)
{
    instr_t *call_instr;
    instr_t *next_instr;
    instrlist_t *ilist = ci->ilist;

    for (call_instr  = instrlist_first(ilist);
         call_instr != NULL;
         call_instr  = next_instr) {
        app_pc cur_pc, next_pc;
        instr_t *new_instr;
        bool is_pic = false;
        opnd_t pic_reg;
        opnd_t tgt;
        app_pc tgt_pc = NULL;

        next_instr = instr_get_next(call_instr);
        if (!instr_is_call(call_instr))
            continue;
        if (next_instr == NULL) {
            ASSERT_CURIOSITY(false && "call is last instruction in ilist?");
            break;
        }

        cur_pc = instr_get_app_pc(call_instr);
        tgt = instr_get_target(call_instr);
        if (opnd_is_pc(tgt))
            tgt_pc = opnd_get_pc(tgt);
        /* Will be NULL if we inserted next_instr. */
        next_pc = instr_get_app_pc(next_instr);

        if (tgt_pc != NULL && tgt_pc == next_pc) {
            /* "pop %r1" or "mov [%rsp] %r1" */
            if (instr_get_opcode(next_instr) == OP_pop ||
                instr_is_mov_ld_xsp(next_instr)) {
                is_pic = true;
                pic_reg = instr_get_dst(next_instr, 0);
                /* Delete mov or pop. */
                instrlist_remove(ilist, next_instr);
                instr_destroy(GLOBAL_DCONTEXT, next_instr);
            }
        } else if (tgt_pc != NULL) {
            /* a callout */
            instr_t mov_ins, ret_ins;
            app_pc tmp_pc;
            instr_init(dcontext, &mov_ins);
            instr_init(dcontext, &ret_ins);
            TRY_EXCEPT(dcontext, {
                tmp_pc = decode(dcontext, tgt_pc, &mov_ins);
                tmp_pc = decode(dcontext, tmp_pc, &ret_ins);
            }, {
                ASSERT_CURIOSITY(false && "crashed while decoding clean call");
                /* Frees any instr internal memory and reinitializes.  Following
                 * checks will fail. */
                instr_reset(dcontext, &mov_ins);
                instr_reset(dcontext, &ret_ins);
            });
            if (instr_is_mov_ld_xsp(&mov_ins) &&
                instr_is_return(&ret_ins)) {
                pic_reg = instr_get_dst(&mov_ins, 0);
                is_pic = true;
            }
            instr_free(dcontext, &mov_ins);
            instr_free(dcontext, &ret_ins);
        }

        if (!is_pic)
            continue;

        ASSERT(opnd_is_reg(pic_reg));
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: special PIC code at: "PFX"\n", cur_pc);

        /* Insert "mov next_pc r1". */
        /* XXX: The memory on top of stack will not be next_pc, and the stack
         * may become unaligned. */
        new_instr = INSTR_CREATE_mov_imm
            (GLOBAL_DCONTEXT, pic_reg, OPND_CREATE_INTPTR(next_pc));
        instr_set_translation(new_instr, cur_pc);
        instrlist_preinsert(ilist, call_instr, new_instr);

        /* Remove call. */
        instrlist_remove(ilist, call_instr);
        instr_destroy(GLOBAL_DCONTEXT, call_instr);
    }
}

/* Callee decoder.  Starts at the entry pc and remembers farthest forward
 * target pc that doesn't look like a tail call.  Decodes until that pc is
 * encountered and then onwards towards the next tail call, backwards jmp, or
 * ret.
 */
static void
decode_callee_ilist(void *dc, callee_info_t *ci)
{
    app_pc cur_pc;
    app_pc max_tgt_pc = NULL;
    bool decode_next = true;

    dr_log(dc, LOG_CLEANCALL, 2,
           "CLEANCALL: decoding callee starting at: "PFX"\n", ci->start);
    ci->ilist = instrlist_create(GLOBAL_DCONTEXT);
    ci->bailout = false;
    cur_pc = ci->start;

    while (decode_next) {
        instr_t *instr;
        app_pc next_pc;
        app_pc tgt_pc = NULL;

        /* Sanity check whether we want to decode this pc. */
        if (!tgt_pc_in_callee(cur_pc, ci)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: jump target "PFX" too far, bailing from decode\n",
                   cur_pc);
            ci->bailout = true;
            break;
        }

        next_pc = decode_callee_instr(dc, ci, cur_pc);
        if (next_pc == NULL) {  /* bailout */
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: decode returned NULL\n");
            break;
        }
        if (ci->num_instrs > MAX_NUM_FUNC_INSTRS) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: too many instructions, bailing from decode\n");
            ci->bailout = true;
            break;
        }

        instr = instrlist_last(ci->ilist);

        /* Remember the maximum branch target if it looks like it's within the
         * callee.  */
        if (instr_is_ubr(instr) || instr_is_cbr(instr)) {
            opnd_t tgt = instr_get_target(instr);
            if (opnd_is_pc(tgt)) {
                tgt_pc = opnd_get_pc(tgt);
                if (tgt_pc_in_callee(tgt_pc, ci)) {
                    max_tgt_pc = (app_pc)MAX((ptr_uint_t)max_tgt_pc,
                                             (ptr_uint_t)tgt_pc);
                }
            }
        }

        /* Stop decoding after a ret. */
        if (instr_is_return(instr)) {
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: stop decode for ret\n");
            decode_next = false;
        }

        if (instr_is_ubr(instr) && tgt_pc != NULL) {
            /* Stop decoding after backwards jmps. */
            if ((ptr_uint_t)tgt_pc < (ptr_uint_t)cur_pc) {
                dr_log(dc, LOG_CLEANCALL, 2,
                       "CLEANCALL: stop decode for bwds jmp from "PFX" to "PFX"\n",
                       cur_pc, tgt_pc);
                decode_next = false;
            }

            /* Stop decoding after tail calls */
            if (!tgt_pc_in_callee(tgt_pc, ci)) {
                dr_log(dc, LOG_CLEANCALL, 2,
                       "CLEANCALL: stop decode for probable tail call at "PFX
                       " to tgt "PFX"\n", cur_pc, tgt_pc);
                decode_next = false;
            }
        }

        /* Keep decoding if we haven't hit the max jump target. */
        if (!decode_next && (ptr_uint_t)max_tgt_pc > (ptr_uint_t)cur_pc) {
            decode_next = true;
        }

        cur_pc = next_pc;
    }

    if (ci->bailout) {
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
        ci->ilist = NULL;
        return;
    }
}

/* Remove and delete an instruction from an ilist.  This helper routine exists
 * to avoid frequent typos where different instructions are passed to the remove
 * and destory routines.
 */
static void
remove_and_destroy(void *dc, instrlist_t *ilist, instr_t *instr)
{
    instrlist_remove(ilist, instr);
    instr_destroy(dc, instr);
}

/* Dead register analysis abstraction.  Drive it by looping over instructions
 * in reverse.  At end of loop body, call reg_dead_update. */

typedef struct _liveness_t {
    bool reg_live[NUM_GP_REGS];
    /* Flags that are live.  Uses EFLAGS_READ_* form of the masks. */
    uint flags_live;
} liveness_t;

#define IS_LIVE(r) (liveness->reg_live[reg_to_pointer_sized(r) - DR_REG_XAX])
#define IS_DEAD(r) !IS_LIVE(r)
#define SET_LIVE(r, d) \
    (liveness->reg_live[reg_to_pointer_sized(r) - DR_REG_XAX] = (d))

static void
liveness_init(liveness_t *liveness, const callee_info_t *ci)
{
    int i;
    /* At callee end, all registers are considered dead except XSP. */
    for (i = 0; i < NUM_GP_REGS; i++)
        liveness->reg_live[i] = false;
    SET_LIVE(DR_REG_XSP, true);
    if (ci->xbp_is_fp)
        SET_LIVE(DR_REG_XBP, true);
    liveness->flags_live = 0;
}

static void
liveness_update(liveness_t *liveness, instr_t *instr)
{
    uint flags = instr_get_eflags(instr);
    int i;

    /* Mark all flags written to as dead.  */
    liveness->flags_live &= ~(EFLAGS_WRITE_TO_READ(flags & EFLAGS_WRITE_ALL));
    /* Mark all flags read from as live.  */
    liveness->flags_live |= (flags & EFLAGS_READ_ALL);

    /* TODO(rnk): Can mark destination register as dead here, but will
     * break if there is control flow, for example rax is still live here:
     * mov %rbx, %rax
     * test %rcx, %rcx
     * jz next
     * xor %rax, %rax
     * next:
     * mov %rax, global
     */

    /* Mark all regs read from as live.  */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (instr_reads_from_reg(instr, DR_REG_XAX + (reg_id_t)i)) {
            SET_LIVE(DR_REG_XAX + (reg_id_t)i, true);
        }
    }

    /* TODO(rnk): To support partial inlining, we may need to change this to
     * consider argument registers live-in to the call.
     */
}

/* Rewrite all reads of 'old' in 'opnd' to 'new', or return false.
 */
/* TODO(rnk): This doesn't handle al/ah type stuff.  ie, if we want to replace
 * RAX w/ RBX we need to rewrite BH to AH and not AL. */
static void
rewrite_opnd_reg(void *dc, instr_t *instr, opnd_t *opnd,
                 reg_id_t old, reg_id_t new, bool check_only)
{
    int i;
    new = reg_to_pointer_sized(new);
    IF_X64(new = reg_64_to_32(new);)
    for (i = 0; i < opnd_num_regs_used(*opnd); i++) {
        reg_id_t reg_used = opnd_get_reg_used(*opnd, i);
        if (reg_overlap(reg_used, old)) {
            reg_id_t new_sized = reg_32_to_opsz(new, reg_get_size(reg_used));
            if (!check_only) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "CLEANCALL: rewriting reg %s to %s at "PFX"\n",
                       get_register_name(reg_used), get_register_name(new_sized),
                       instr_get_app_pc(instr));
            }
            opnd_replace_reg(opnd, reg_used, new_sized);
        }
    }
}

/* Wrapper for instr_is_encoding_possible that works on labels. */
static bool
can_encode(instr_t *instr)
{
    return instr_is_label(instr) || instr_is_encoding_possible(instr);
}

static bool
rewrite_reg_dst(void *dc, instr_t *instr, reg_id_t old, reg_id_t new,
                bool check_only)
{
    bool encodable;
    opnd_t opnd;

    /* Only rewrite instrs writing a single reg dst. */
    if (!instr_num_dsts(instr) == 1)
        return false;
    opnd = instr_get_dst(instr, 0);
    if (!opnd_is_reg(opnd))
        return false;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    opnd = instr_get_dst(instr, 0);
    rewrite_opnd_reg(dc, instr, &opnd, old, new, check_only);
    instr_set_dst(instr, 0, opnd);

    encodable = can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return encodable;
}

/* Rewrite reads from in instr from 'old' to 'new'.  Returns true or false if
 * the resulting instruction is encodable.  If check_only is true, the
 * instruction will not be modified regardless of success or failure. */
static bool
rewrite_reg_reads(void *dc, instr_t *instr, reg_id_t old, reg_id_t new,
                  bool check_only)
{
    bool encodable;
    int i;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    /* srcs */
    for (i = 0; i < instr_num_srcs(instr); i++) {
        opnd_t opnd = instr_get_src(instr, i);
        rewrite_opnd_reg(dc, instr, &opnd, old, new, check_only);
        instr_set_src(instr, i, opnd);
    }

    /* dsts */
    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t opnd = instr_get_dst(instr, i);
        if (!opnd_is_base_disp(opnd))
            continue;  /* The only reads in dsts are regs in base/disp opnds. */
        rewrite_opnd_reg(dc, instr, &opnd, old, new, check_only);
        instr_set_dst(instr, i, opnd);
    }

    encodable = can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return encodable;
}

/* Find the next full clobber of 'reg', if it exists.  If there is a
 * subregister write that does not fully clobber the register, return false.
 */
static bool
find_next_full_clobber(void *dc, instr_t *start, reg_id_t reg,
                       instr_t **end)
{
    instr_t *instr;
    int i;

    /* Find the next write of dst.  If it's a complicated subregister write, we
     * leave it alone.  Otherwise, we can rewrite up until that instruction. */
    for (instr = instr_get_next(start); instr != NULL;
         instr = instr_get_next(instr)) {
        if (!instr_writes_to_reg(instr, reg))
            continue;

        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t d = instr_get_dst(instr, i);
            if (opnd_is_reg(d) && reg_overlap(opnd_get_reg(d), reg)) {
                opnd_size_t sz = opnd_get_size(d);
                if (sz == OPSZ_4 || sz == OPSZ_8) {
                    /* dst is fully clobbered here. */
                    *end = instr;
                    return true;
                } else {
                    /* Subregister write, clobber is not full. */
                    return false;
                }
            }
        }
    }
    if (instr == NULL)
        *end = NULL;
    return true;
}

/* Rewrite all uses of reg 'old' to 'new' over the live range of 'old' starting
 * at start.  Rewrites the destination of start, and all reads up until the
 * next write that fully clobbers 'old'.  Returns false without making any
 * changes on failure. */
static bool
rewrite_live_range(void *dc, instr_t *start, reg_id_t old, reg_id_t new)
{
    instr_t *instr;
    instr_t *last_instr = NULL;
    DEBUG_DECLARE(bool ok);

    /* We only rewrite up until the next instruction that fully clobbers the
     * destination register.  If the update is not a full clobber (ie a write
     * of a subreg) then we bail. */
    if (!find_next_full_clobber(dc, start, old, &last_instr))
        return false;
    /* If the write contains any reads, make sure to update them.  For example,
     * if %rax is old and %rbx is new and the following is last_instr:
     * lea -24(%rax), %rax
     * It should be rewritten as well to become:
     * lea -24(%rbx), %rax
     */
    if (last_instr != NULL)
        last_instr = instr_get_next(last_instr);

    /* First check if we can rewrite the dst. */
    if (!rewrite_reg_dst(dc, start, old, new, /*check_only=*/true))
        return false;
    /* Find all reads of old and check if we can rewrite. */
    for (instr = instr_get_next(start); instr != last_instr;
         instr = instr_get_next(instr))
        if (!rewrite_reg_reads(dc, instr, old, new, /*check_only=*/true))
            return false;

    /* Now that we know this will work, actually rewrite the registers. */
    IF_DEBUG(ok =) rewrite_reg_dst(dc, start, old, new, /*check_only=*/false);
    ASSERT(ok);
    for (instr = instr_get_next(start); instr != last_instr;
         instr = instr_get_next(instr)) {
        IF_DEBUG(ok =) rewrite_reg_reads(dc, instr, old, new,
                                         /*check_only=*/false);
        ASSERT(ok);
    }
    return true;
}

/* Propagate copies from dead source registers to reduce register pressure.
 * Removes and destroys 'copy' and returns true if successful, otherwise
 * returns false.
 */
static bool
propagate_copy(void *dc, instrlist_t *ilist, instr_t *copy)
{
    reg_id_t src = opnd_get_reg(instr_get_src(copy, 0));
    reg_id_t dst = opnd_get_reg(instr_get_dst(copy, 0));
    opnd_size_t copy_sz = reg_get_size(dst);

    /* Let's not get into copy sizes that don't clobber the entire GPR. */
    if (!(copy_sz == OPSZ_4 || copy_sz == OPSZ_8))
        return false;

    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: propagating copy at "PFX"\n", instr_get_app_pc(copy));

    if (!rewrite_live_range(dc, copy, dst, src))
        return false;

    /* Delete the copy. */
    remove_and_destroy(GLOBAL_DCONTEXT, ilist, copy);

    return true;
}

static bool
can_use_immed_for_opnd_reg(ptr_int_t val, reg_id_t dst_reg, reg_id_t opnd_reg)
{
    /* We can fold if opnd_reg is dst_reg or if it's the 64-bit version of
     * dst_reg and val is less than INT_MAX.  If it is larger, it will be sign
     * extended when the code requires zero extension. */
    return (opnd_reg == dst_reg ||
            IF_X64_ELSE((opnd_reg == reg_32_to_64(dst_reg) &&
                         val <= INT_MAX), false));
}

static bool
rewrite_reg_immed(void *dc, instr_t *instr, reg_id_t reg,
                  ptr_int_t val, bool check_only)
{
    opnd_t imm = opnd_create_immed_int(val, OPSZ_4);
    bool failed = false;
    int i;

    /* Avoid side-effects on original instruction if we're just checking. */
    if (check_only)
        instr = instr_clone(dc, instr);

    for (i = 0; i < instr_num_srcs(instr); i++) {
        opnd_t opnd = instr_get_src(instr, i);
        if (!opnd_uses_reg(opnd, reg))
            continue;
        if (opnd_is_reg(opnd)) {
            reg_id_t opnd_reg = opnd_get_reg(opnd);
            if (can_use_immed_for_opnd_reg(val, reg, opnd_reg)) {
                instr_set_src(instr, i, imm);
            } else {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "CLEANCALL: unable to fold due to subreg use at "PFX"\n",
                       instr_get_app_pc(instr));
                failed = true;
                break;
            }
        } else if (opnd_is_base_disp(opnd)) {
            ptr_int_t disp = opnd_get_disp(opnd);
            reg_id_t base = opnd_get_base(opnd);
            reg_id_t index = opnd_get_index(opnd);
            int scale = opnd_get_scale(opnd);
            opnd_size_t sz = opnd_get_size(opnd);
            if (can_use_immed_for_opnd_reg(val, reg, index)) {
                disp += val * scale;
                index = DR_REG_NULL;
                scale = 0;
            } else if (can_use_immed_for_opnd_reg(val, reg, base)) {
                disp += val;
                if (scale == 1) {
                    base = index;
                    index = DR_REG_NULL;
                    scale = 0;
                } else if (scale == 2) {
                    base = index;
                    index = index;
                    scale = 1;
                } else {
                    dr_log(dc, LOG_CLEANCALL, 3,
                           "CLEANCALL: can't fold immed with scale %d\n", scale);
                    failed = true;
                    break;
                }
            } else {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "CLEANCALL: unable to fold immed in subreg use at "PFX"\n",
                       instr_get_app_pc(instr));
                failed = true;
                break;
            }
            instr_set_src(instr, i, opnd_create_base_disp
                          (base, index, scale, disp, sz));
        } else {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "CLEANCALL: unrecognized use of reg at "PFX"\n",
                   instr_get_app_pc(instr));
            failed = true;
            break;
        }
        if (!check_only) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "CLEANCALL: folded immediate into src opnd at "PFX"\n",
                   instr_get_app_pc(instr));
        }
    }

    for (i = 0; i < instr_num_dsts(instr); i++) {
        opnd_t opnd = instr_get_src(instr, i);
        /* Only base-disp opnds read regs. */
        if (!opnd_is_base_disp(opnd))
            continue;
        if (opnd_uses_reg(opnd, reg)) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "CLEANCALL: can't yet fold immediate into dst opnd at "PFX"\n",
                   instr_get_app_pc(instr));
            failed = true;
            break;
        }
    }

    failed |= !can_encode(instr);

    if (check_only)
        instr_destroy(dc, instr);

    return !failed;
}

static bool
rewrite_live_range_immed(void *dc, instr_t *start, instr_t *end,
                         reg_id_t reg, ptr_int_t val, bool check_only)
{
    instr_t *instr;

    ASSERT(reg_get_size(reg) == OPSZ_4);
    for (instr = instr_get_next(start); instr != end;
         instr = instr_get_next(instr))
        if (!rewrite_reg_immed(dc, instr, reg, val, check_only))
            return false;
    return true;
}

static reg_id_t
pick_used_dead_reg(void *dc, instrlist_t *ilist, liveness_t *liveness,
                   instr_t *reuse_instr)
{
    bool reg_used[NUM_GP_REGS];
    instr_t *instr;
    uint i;
    reg_id_t reg;

    /* Compute registers used before reuse_instr. */
    memset(reg_used, 0, sizeof(reg_used));
    for (instr = instrlist_first(ilist); instr != reuse_instr;
         instr = instr_get_next(instr)) {
        for (i = 0; i < NUM_GP_REGS; i++) {
            if (instr_uses_reg(instr, DR_REG_XAX + (reg_id_t)i))
                reg_used[i] = true;
        }
    }
    /* Also count registers read from by reuse_instr. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (instr_reads_from_reg(reuse_instr, DR_REG_XAX + (reg_id_t)i))
            reg_used[i] = true;
    }

    /* Find first dead register that we've already used. */
    for (i = 0; i < NUM_GP_REGS; i++)
        if (reg_used[i] && IS_DEAD(DR_REG_XAX + (reg_id_t)i))
            break;
    if (i == NUM_GP_REGS)
        return DR_REG_NULL;
    reg = DR_REG_XAX + (reg_id_t)i;
    ASSERT(reg_used[i] && IS_DEAD(reg));
    return reg;
}

static reg_id_t
pick_dead_reg(void *dc, liveness_t *liveness)
{
    int i;

    for (i = 0; i < NUM_GP_REGS; i++) {
        reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
        if (IS_DEAD(reg))
            return reg;
    }
    return DR_REG_NULL;
}

/* If this is a reg-to-reg instruction that uses a new register when there are
 * existing dead registers that we've already used, try to reuse registers.
 * This creates less saves and restores.  */
static void
try_reuse_register(void *dc, instrlist_t *ilist, liveness_t *liveness,
                   instr_t *reuse_instr)
{
    opnd_t opnd = instr_get_dst(reuse_instr, 0);
    reg_id_t dst = opnd_get_reg(opnd);
    reg_id_t new;

    new = pick_used_dead_reg(dc, ilist, liveness, reuse_instr);
    if (new == DR_REG_NULL)
        return;

    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: trying to reuse dead register %s instead of %s at "PFX"\n",
           get_register_name(new), get_register_name(dst),
           instr_get_app_pc(reuse_instr));

    /* Rewrite dst and all further uses of dst to new. */
    if (rewrite_live_range(dc, reuse_instr, dst, new)) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "CLEANCALL: succeeded reusing dead register %s instead of %s at "
               PFX"\n",
               get_register_name(new), get_register_name(dst),
               instr_get_app_pc(reuse_instr));
    }
}

/* For movs from immediate to register, try to propagate immediate into
 * instructions that use it.
 */
static bool
fold_mov_immed(void *dc, instrlist_t *ilist, instr_t *mov_imm)
{
    opnd_t imm = instr_get_src(mov_imm, 0);
    opnd_t dst = instr_get_dst(mov_imm, 0);
    ptr_int_t val = opnd_get_immed_int(imm);
    reg_id_t reg = opnd_get_reg(dst);
    instr_t *end = NULL;
    DEBUG_DECLARE(bool ok;)

    /* For now we only deal with 32-bit immediates, since pointer values are
     * trickier to rewrite. */
    /* TODO(rnk): If pointer value is < 4 GB, can use reladdr memory operands
     * instead of materializing the pointer as an immediate.  */
    if (opnd_get_size(dst) != OPSZ_4)
        return false;

    if (!find_next_full_clobber(dc, mov_imm, reg, &end))
        return false;
    if (end != NULL)
        end = instr_get_next(end);

    if (end) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "CLEANCALL: mov imm live range ends at "PFX"\n",
               instr_get_app_pc(end));
    }
    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: attempting to fold mov_imm at "PFX"\n",
           instr_get_app_pc(mov_imm));
    if (!rewrite_live_range_immed(dc, mov_imm, end, reg, val, true))
        return false;
    IF_DEBUG(ok =) rewrite_live_range_immed(dc, mov_imm, end, reg, val, false);
    ASSERT(ok);
    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: folded immediate mov at "PFX"\n",
           instr_get_app_pc(mov_imm));
    return true;
}

static bool
addsub_to_lea(void *dc, const callee_info_t *ci, liveness_t *liveness,
              instr_t *instr, bool check_only)
{
    opnd_t src = instr_get_src(instr, 0);
    opnd_t dst = instr_get_dst(instr, 0);
    reg_id_t dst_reg;
    ASSERT(opnd_same(dst, instr_get_src(instr, 1)));
    if (check_only) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "CLEANCALL: attempting add -> lea for: ");
        instr_disassemble(dc, instr, dr_get_logfile(dc));
        dr_log(dc, LOG_CLEANCALL, 3, "\n");
    }

    /* Extract memory operands from add into temp register loads and stores. */
    if (opnd_is_memory_reference(src) || opnd_is_memory_reference(dst)) {
        reg_id_t tmp_reg;
        opnd_t tmp;
        opnd_t memref = opnd_is_memory_reference(src) ? src : dst;

        ASSERT(!(opnd_is_memory_reference(src) &&
                 opnd_is_memory_reference(dst)));
        tmp_reg = pick_used_dead_reg(dc, ci->ilist, liveness, instr);
        if (tmp_reg == DR_REG_NULL) {
            if (check_only) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "CLEANCALL: can't steal used dead reg, getting unused\n");
            }
            tmp_reg = pick_dead_reg(dc, liveness);
            if (tmp_reg == DR_REG_NULL) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "CLEANCALL: can't steal dead reg, add -> lea failed\n");
                return false;
            }
        }
        /* Match the register size to the memref size. */
        IF_X64(tmp_reg = reg_64_to_32(tmp_reg);)
        tmp_reg = reg_32_to_opsz(tmp_reg, opnd_get_size(memref));
        tmp = opnd_create_reg(tmp_reg);

        if (!check_only) {
            /* Insert load in either case. */
            PRE(ci->ilist, instr, INSTR_CREATE_mov_ld
                (GLOBAL_DCONTEXT, tmp, memref));
            /* Insert store if memref is dst. */
            if (opnd_is_memory_reference(dst)) {
                POST(ci->ilist, instr, INSTR_CREATE_mov_st
                     (GLOBAL_DCONTEXT, memref, tmp));
            }
        }

        /* Replace memref w/ tmp. */
        src = opnd_is_memory_reference(src) ? tmp : src;
        dst = opnd_is_memory_reference(dst) ? tmp : dst;
    }

    /* Should be of form: add r/i, r */
    dst_reg = opnd_get_reg(dst);
    if (opnd_is_immed_int(src)) {
        /* lea +/-IMM(%dst), %dst */
        if (!check_only) {
            ptr_int_t disp = opnd_get_immed_int(src);
            if (instr_get_opcode(instr) == OP_sub)
                disp = -disp;
            PRE(ci->ilist, instr, INSTR_CREATE_lea
                (GLOBAL_DCONTEXT, dst,
                 OPND_CREATE_MEM_lea(dst_reg, DR_REG_NULL, 0, disp)));
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
        }
    } else if (opnd_is_reg(src)) {
        reg_id_t src_reg = opnd_get_reg(src);
        if (instr_get_opcode(instr) == OP_sub && !IS_DEAD(src_reg)) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "CLEANCALL: sub -> lea failed, needs dead src reg\n");
            return false;
        }
        if (!check_only) {
            /* lea 0(%src, %dst, 1), %dst */
            PRE(ci->ilist, instr, INSTR_CREATE_lea
                (GLOBAL_DCONTEXT, dst,
                 OPND_CREATE_MEM_lea(dst_reg, src_reg, 1, 0)));
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
        }
    } else {
        dr_log(dc, LOG_CLEANCALL, 3,
               "CLEANCALL: add -> lea failed, unrecognized src opnd\n");
        return false;
    }
    return true;
}

static bool
optimize_avoid_flags(void *dc, const callee_info_t *ci, bool check_only)
{
    liveness_t liveness_;
    liveness_t *liveness = &liveness_;
    instrlist_t *ilist = ci->ilist;
    instr_t *instr, *prev_instr;

    liveness_init(liveness, ci);
    for (instr = instrlist_last(ilist); instr != NULL; instr = prev_instr) {
        prev_instr = instr_get_prev(instr);

        /* Update before analysis to consider registers live-in to this
         * instruction to be live.  We can't use them as temp registers. */
        liveness_update(liveness, instr);

        switch (instr_get_opcode(instr)) {
        case OP_sub:
        case OP_add:
            if (!addsub_to_lea(dc, ci, liveness, instr, check_only))
                return false;
            break;
        case OP_jz:
            /* TODO(rnk): Do cmp -> not+lea trick. */
            return false;
        default:
            if (TESTANY(EFLAGS_WRITE_6, instr_get_arith_flags(instr)) ||
                TESTANY(EFLAGS_READ_6, instr_get_arith_flags(instr))) {
                dr_log(dc, LOG_CLEANCALL, 3,
                       "CLEANCALL: flags required for instr at "PFX", "
                       "flags avoidance not profitable\n",
                       instr_get_app_pc(instr));
                return false;
            }
        }
    }
    return true;
}

/* Remove and destroy instructions whose only destinations are dead registers.
 * Assumes there are no calls or backwards branches in ilist.
 * TODO(rnk): What about other control flow?
 */
static void
optimize_callee(void *dc, const callee_info_t *ci)
{
    liveness_t liveness_;
    liveness_t *liveness = &liveness_;
    instrlist_t *ilist = ci->ilist;
    int i;
    instr_t *instr;
    instr_t *prev_instr;
    instr_t *next_instr;

    i = 0;
    for (instr = instrlist_first(ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        i++;
    }
    if (i > MAX_NUM_INLINE_INSTRS) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "CLEANCALL: refusing to attempt to optimize large func\n");
        return;
    }

    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: callee ilist before optimization:\n");
    DOLOG(3, LOG_CLEANCALL, {
        instrlist_disassemble(dc, NULL, ilist, dr_get_logfile(dc));
    });

    liveness_init(liveness, ci);

    for (instr = instrlist_last(ilist); instr != NULL; instr = prev_instr) {
        bool instr_is_live = false;
        prev_instr = instr_get_prev(instr);
        int opc;
        uint flags_written;

        /* If only dsts are regs, and all dsts are dead, can delete instr. */
        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t dst = instr_get_dst(instr, i);
            if (!opnd_is_reg(dst)) break;
            if (!IS_DEAD(opnd_get_reg(dst))) break;
        }
        /* If the loop exited before completion, then the destinations are
         * live. */
        if (i < instr_num_dsts(instr))
            instr_is_live |= true;
        /* If the instr writes live flags it's live. */
        flags_written =
            EFLAGS_WRITE_TO_READ(EFLAGS_WRITE_ALL & instr_get_eflags(instr));
        if (TESTANY(liveness->flags_live, flags_written))
            instr_is_live |= true;
        /* If the instr is cti or label it's live. */
        if (instr_is_cti(instr) || instr_is_label(instr))
            instr_is_live |= true;

        if (!instr_is_live) {
            dr_log(dc, LOG_CLEANCALL, 3,
                   "CLEANCALL: removing dead instr at "PFX".\n",
                   instr_get_app_pc(instr));
            remove_and_destroy(GLOBAL_DCONTEXT, ilist, instr);
            continue;
        }

        /* If this is a copy from a dead src reg, we may be able to reduce
         * register pressure by deleting it and using src direcly. */
        opc = instr_get_opcode(instr);
        if ((opc == OP_mov_ld || opc == OP_mov_st) &&
            opnd_is_reg(instr_get_src(instr, 0)) &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            reg_id_t src_reg = opnd_get_reg(instr_get_src(instr, 0));
            if (IS_DEAD(src_reg)) {
                if (propagate_copy(dc, ilist, instr)) {
                    /* Src reg is live now. */
                    SET_LIVE(src_reg, true);
                    continue;
                }
            }
        }

        liveness_update(liveness, instr);
    }

    /* Do a second pass and attempt to reuse registers.  We don't do this on
     * the first pass or we may attempt to reuse registers that we don't end up
     * using after DCE. */
    liveness_init(liveness, ci);
    for (instr = instrlist_last(ilist); instr != NULL; instr = prev_instr) {
        prev_instr = instr_get_prev(instr);

        /* If this is a reg-to-reg instruction that uses a new register when
         * there are existing dead registers that we've already used, try to
         * reuse registers.  This creates less saves and restores.  */
        if (!instr_is_cti(instr) &&
            instr_num_dsts(instr) == 1 &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            try_reuse_register(dc, ilist, liveness, instr);
        }

        liveness_update(liveness, instr);
    }

    /* Do another pass to see if we can rewrite arithmetic to avoid the use of
     * flags registers. */
    if (optimize_avoid_flags(dc, ci, true)) {
        IF_DEBUG(bool ok =) optimize_avoid_flags(dc, ci, false);
        ASSERT(ok);
    }

    /* Do a final forward pass to fold immediates moved through registers. */
    for (instr = instrlist_first(ilist); instr != NULL; instr = next_instr) {
        next_instr = instr_get_next(instr);
        if (instr_is_mov(instr) &&
            opnd_is_immed_int(instr_get_src(instr, 0)) &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            if (fold_mov_immed(dc, ilist, instr)) {
                remove_and_destroy(GLOBAL_DCONTEXT, ilist, instr);
            }
        }
    }

    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: callee ilist after optimization:\n");
    DOLOG(3, LOG_CLEANCALL, {
        instrlist_disassemble(dc, NULL, ilist, dr_get_logfile(dc));
    });
}

static void
analyze_callee_regs_usage(void *dcontext, callee_info_t *ci)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *instr;
    uint i;

    ci->num_xmms_used = 0;
    memset(ci->xmm_used, 0, sizeof(bool) * NUM_XMM_REGS);
    ci->num_regs_used = 0;
    memset(ci->reg_used, 0, sizeof(bool) * NUM_GP_REGS);
    ci->write_aflags = false;
    for (instr  = instrlist_first(ilist);
         instr != NULL;
         instr  = instr_get_next(instr)) {
        /* XXX: this is not efficient as instr_uses_reg will iterate over
         * every operands, and the total would be (NUM_REGS * NUM_OPNDS)
         * for each instruction. However, since this will be only called 
         * once for each clean call callee, it will have little performance
         * impact unless there are a lot of different clean call callees.
         */
        /* XMM registers usage */
        for (i = 0; i < NUM_XMM_REGS; i++) {
            if (!ci->xmm_used[i] &&
                instr_uses_reg(instr, (DR_REG_XMM0 + (reg_id_t)i))) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "CLEANCALL: callee "PFX" uses XMM%d at "PFX"\n", 
                       ci->start, i, instr_get_app_pc(instr));
                ci->xmm_used[i] = true;
                ci->num_xmms_used++;
            }
        }
        /* General purpose registers */
        for (i = 0; i < NUM_GP_REGS; i++) {
            if (!ci->reg_used[i] &&
                instr_uses_reg(instr, (DR_REG_XAX + (reg_id_t)i))) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "CLEANCALL: callee "PFX" uses REG %s at "PFX"\n", 
                       ci->start, get_register_name(DR_REG_XAX + (reg_id_t)i),
                    instr_get_app_pc(instr));
                ci->reg_used[i] = true;
                ci->num_regs_used++;
            }
        }
        /* callee update aflags */
        if (!ci->write_aflags) {
            if (TESTANY(EFLAGS_WRITE_6, instr_get_arith_flags(instr))) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "CLEANCALL: callee "PFX" updates aflags\n", ci->start);
                ci->write_aflags = true;
            }
        }
    }
    /* we treat %xsp separately. */
    if (ci->reg_used[DR_REG_XSP - DR_REG_XAX]) {
        ci->reg_used[DR_REG_XSP - DR_REG_XAX] = false;
        ci->num_regs_used--;
    }

    /* check if callee read aflags from caller */
    /* set it false for the case of empty callee. */
    ci->read_aflags = false;
    for (instr  = instrlist_first(ilist);
         instr != NULL;
         instr  = instr_get_next(instr)) {
        uint flags = instr_get_arith_flags(instr);
        if (TESTANY(EFLAGS_READ_6, flags)) {
            ci->read_aflags = true;
            break;
        }
        if (TESTALL(EFLAGS_WRITE_6, flags))
            break;
        if (instr_is_return(instr))
            break;
        /* TODO(rnk): Is this necessary?  Catches jrcxz. */
        if (instr_is_cti(instr)) {
            ci->read_aflags = true;
            break;
        }
    }
    if (ci->read_aflags) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: callee "PFX" reads aflags from caller\n", ci->start);
    }
}

/* Find all control flow instructions that leave the function.
 */
#define MAX_EXIT_POINTS 10
static int
analyze_find_exit_instrs(void *dc, callee_info_t *ci, instr_t **bots)
{
    int num_bots = 0;
    instr_t *instr;

    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        instr_t *exit_instr = NULL;
        if (instr_is_return(instr)) {
            exit_instr = instr;
        }
        if (instr_is_ubr(instr)) {
            opnd_t tgt = instr_get_target(instr);
            if (opnd_is_pc(tgt) && !tgt_pc_in_callee(opnd_get_pc(tgt), ci)) {
                exit_instr = instr;
            }
        }
        if (exit_instr != NULL) {
            if (num_bots >= MAX_EXIT_POINTS) {
                return num_bots;
            }
            bots[num_bots++] = exit_instr;
        }
    }

    return num_bots;
}

/* Returns true if the instruction is a control flow instruction or jump target.
 */
static bool
is_cti_or_label_instr(instr_t *instr)
{
    return (instr_is_cti(instr) || instr_is_label(instr));
}

/* Returns false if the instruction is a control flow instruction, a stack
 * instruction, or a jump target.  Otherwise returns true.
 */
static bool
is_boring_instr(instr_t *instr)
{
    if (instr_uses_reg(instr, DR_REG_XBP) || instr_uses_reg(instr, DR_REG_XSP))
        return false;
    return !is_cti_or_label_instr(instr);
}

/* Advance forward in the ilist until we find an instruction that modifies the
 * stack, does control flow, or is a label.
 */
static instr_t *
skip_boring_instrs_fwd(instr_t *top)
{
    for (; top != NULL; top = instr_get_next(top))
        if (!is_boring_instr(top))
            break;
    return top;
}

/* Advance backwards through the ilist until we find an instruction that
 * modifies the stack, does control flow, or is a label.
 */
static void
skip_boring_instrs_bwd(instr_t **bots, int num)
{
    int i;
    for (i = 0; i < num; i++) {
        instr_t *bot = bots[i];
        for (; bot != NULL; bot = instr_get_prev(bot))
            if (!is_boring_instr(bot))
                break;
        bots[i] = bot;
    }
}

typedef bool (*pred_fn_t)(instr_t *item, void *data);

/* Takes a predicate function pointer, a data value to pass to all predicate
 * calls, and an array of instr_t pointers and number of elements.  If the
 * predicate returns true for all instrs, return true, otherwise false.
 */
static bool
all_satisfy(pred_fn_t pred, void *data, instr_t **instrs, int num)
{
    int i;
    for (i = 0; i < num; i++) {
        if (!pred(instrs[i], data)) {
            return false;
        }
    }
    return true;
}

/* Similar to all_satisfy, except return true if any predicates succeed.
 */
static bool
any_satisfy(pred_fn_t pred, void *data, instr_t **instrs, int num)
{
    int i;
    for (i = 0; i < num; i++) {
        if (pred(instrs[i], data)) {
            return true;
        }
    }
    return false;
}

static bool
pred_not_null(instr_t *instr, void *data)
{
    return instr != NULL;
}

/* TODO(rnk): Only one use, maybe should re-inline it. */
static instr_t *
remove_top_bot_pairs(instrlist_t *ilist, instr_t *top, instr_t **bots,
                     int num_bots)
{
    instr_t *instr;
    int i;

    instr = instr_get_next(top);
    remove_and_destroy(GLOBAL_DCONTEXT, ilist, top);
    top = instr;
    for (i = 0; i < num_bots; i++) {
        instr = instr_get_prev(bots[i]);
        remove_and_destroy(GLOBAL_DCONTEXT, ilist, bots[i]);
        bots[i] = instr;
    }
    return top;
}

/* Checks if instruction is equivalent to "mov %src => %dst".  This is needed
 * because mov can be encoded with mov_ld or mov_st opcodes. */
static bool
is_mov_reg2reg(instr_t *instr, reg_id_t dst, reg_id_t src)
{
    int opc = instr_get_opcode(instr);
    return ((opc == OP_mov_ld || opc == OP_mov_st) &&
            opnd_same(instr_get_src(instr, 0), opnd_create_reg(src)) &&
            opnd_same(instr_get_dst(instr, 0), opnd_create_reg(dst)));
}

/* Remove two instructions from the same ilist so long as they aren't the same
 * instruction. */
static void
remove_both_instrs(void *dc, instrlist_t *ilist,
                   instr_t *instr1, instr_t *instr2)
{
    ASSERT(instr1 != NULL && instr2 != NULL);
    if (instr1 == instr2) {
        instr2 = NULL;
    }
    remove_and_destroy(dc, ilist, instr1);
    if (instr2 != NULL) {
        remove_and_destroy(dc, ilist, instr2);
    }
}

/* Find enter/leave pair.  Frame setup may maintain xbp as frame ptr or not.  If
 * maintaining frame ptr, there are two prologue and epilogue sequences.
 * Prologue:
 * enter $framesize, $0
 * push xbp ; ... ; mov xsp => xbp
 * Epilogue:
 * leave
 * mov xbp => xsp ; ... ; pop xbp
 *
 * We should match any combination of the above epilogues and prologues.
 * push+mov+leave seems most common.
 *
 * If omitting frame pointer, xbp is just callee-saved and xsp is adjusted
 * directly:
 * push xbp ; sub $framesize xsp ; ... ; add $framesize xbp ; pop xbp
 *
 * Returns new top after removing xbp save if found.
 *
 * Sets ci->frame_size if 'enter' is used to set up frame.
 */
static instr_t *
analyze_enter_leave(void *dc, callee_info_t *ci, instr_t *top,
                    instr_t **bots, int num_bots)
{
    opnd_t xbp, xsp;
    instr_t *push_xbp;    /* Push of xbp.  Can be push or enter. */
    instr_t *save_xsp;    /* Copy of xsp to xbp.  Can be mov or enter. */
    instr_t *restore_xsp; /* Restore of xsp from xbp.  Can be mov or leave. */
    instr_t *pop_xbp;     /* Pop of xbp.  Can be pop or leave. */
    /* Callees are single-entry, but may have multiple exits using different
     * prologues.  However, they should all undo the operations of the prologue.
     * These arrays track individual epilogue instructions for each exit point
     * of the callee.
     */
    instr_t *restore_xsps[MAX_EXIT_POINTS];
    instr_t *pop_xbps[MAX_EXIT_POINTS];
    int i;
    /* Reference instructions for comparison. */
    instr_t *cmp_push_xbp;
    instr_t *cmp_pop_xbp;

    /* for easy of comparison, create push xbp, pop xbp */
    xbp = opnd_create_reg(DR_REG_XBP);
    xsp = opnd_create_reg(DR_REG_XSP);
    cmp_push_xbp    = INSTR_CREATE_push(dc, xbp);
    cmp_pop_xbp     = INSTR_CREATE_pop (dc, xbp);

    /* Determine prologue style: do we push xbp, do we save xsp. */
    push_xbp = NULL;
    save_xsp = NULL;
    if (instr_get_opcode(top) == OP_enter) {
        /* Enter pushes xbp, copies xsp to xbp, and subtracts from xsp. */
        push_xbp = top;
        save_xsp = top;
        ci->frame_size = opnd_get_immed_int(instr_get_src(top, 0));
        ASSERT_CURIOSITY(opnd_get_immed_int(instr_get_src(top, 1)) == 0 &&
                         "Callee uses 'enter' with non-zero second argument?");
    } else if (instr_same(cmp_push_xbp, top)) {
        instr_t *maybe_save = skip_boring_instrs_fwd(instr_get_next(top));
        push_xbp = top;
        /* Instr is "mov xsp => xbp". */
        if (is_mov_reg2reg(maybe_save, DR_REG_XBP, DR_REG_XSP)) {
            save_xsp = maybe_save;
        }
    }
    if (push_xbp == NULL) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: callee does not save xbp before using stack at "PFX"\n",
               instr_get_app_pc(top));
    }

    /* Determine epilogue styles. */
    for (i = 0; i < num_bots; i++) {
        instr_t *bot = bots[i];
        pop_xbp = NULL;
        restore_xsp = NULL;
        if (bot == NULL) {
            /* Do nothing, callee does not touch xbp. */
        } else if (instr_get_opcode(bot) == OP_leave) {
            /* Leave does both. */
            pop_xbp = bot;
            restore_xsp = bot;
        } else if (instr_same(bot, cmp_pop_xbp)) {
            instr_t *maybe_restore = instr_get_prev(bot);
            pop_xbp = bot;
            skip_boring_instrs_bwd(&maybe_restore, 1);
            /* Instr is "mov xbp => xsp". */
            if (is_mov_reg2reg(maybe_restore, DR_REG_XSP, DR_REG_XBP)) {
                restore_xsp = maybe_restore;
            }
        }
        pop_xbps[i] = pop_xbp;
        restore_xsps[i] = restore_xsp;
    }

    dr_log(dc, LOG_CLEANCALL, 2,
           "CLEANCALL: prologue pushes %%xbp at "PFX", sets fp at "PFX"\n",
           push_xbp == NULL ? NULL : instr_get_app_pc(push_xbp),
           save_xsp == NULL ? NULL : instr_get_app_pc(save_xsp));
    for (i = 0; i < num_bots; i++) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: epilogue sets sp to fp at "PFX" pops %%xbp at "PFX"\n",
               restore_xsps[i] == NULL ? NULL : instr_get_app_pc(restore_xsps[i]),
               pop_xbps    [i] == NULL ? NULL : instr_get_app_pc(pop_xbps    [i]));
    }

    /* If prologue and epilogue style match, remove prologue and epilogue
     * instructions. */
    if (push_xbp != NULL && save_xsp != NULL &&
        all_satisfy(pred_not_null, NULL, pop_xbps, num_bots) &&
        all_satisfy(pred_not_null, NULL, restore_xsps, num_bots)) {
        /* Using xbp as frame pointer, delete both instrs if not same. */
        top = instr_get_next(save_xsp);
        ci->xbp_is_fp = true;
        remove_both_instrs(GLOBAL_DCONTEXT, ci->ilist, push_xbp, save_xsp);
        for (i = 0; i < num_bots; i++) {
            bots[i] = instr_get_prev(restore_xsps[i]); /* Earliest instr. */
            remove_both_instrs(GLOBAL_DCONTEXT, ci->ilist,
                               pop_xbps[i], restore_xsps[i]);
        }
    }
    else if (push_xbp != NULL && save_xsp == NULL &&
             all_satisfy(pred_not_null, NULL, pop_xbps, num_bots) &&
             !any_satisfy(pred_not_null, NULL, restore_xsps, num_bots)) {
        /* Not using xbp as frame pointer, delete push/pop instrs only. */
        top = instr_get_next(push_xbp);
        remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, push_xbp);
        for (i = 0; i < num_bots; i++) {
            /* New bottom is one earlier than pop. */
            bots[i] = instr_get_prev(pop_xbps[i]);
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, pop_xbps[i]);
        }
    }
    else {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: Unable to match prologue to epilogue.\n");
        push_xbp = NULL;
        save_xsp = NULL;
    }

    if (push_xbp != NULL) {
        /* Function preserves xbp. */
        ci->callee_save_regs[DR_REG_XBP - DR_REG_XAX] = true;
    }

    /* Destroy instructions used for comparison. */
    instr_destroy(dc, cmp_push_xbp);
    instr_destroy(dc, cmp_pop_xbp);

    return top;
}

/* We use push/pop pattern to detect callee saved registers, and assume that the
 * code later won't change those saved value on the stack.
 */
static void
analyze_callee_setup(void *dcontext, callee_info_t *ci)
{
    instrlist_t *ilist = ci->ilist;
    instr_t *top;
    instr_t *bots[MAX_EXIT_POINTS];
    int num_bots, i;

    ASSERT(ilist != NULL);
    /* Function prologues and epilogues are frequently rescheduled so that
     * register-to-register instructions are intermixed with both.  Our analysis
     * ignores instructions in the prologue and epilogue that don't touch the
     * stack and ignore them.
     *
     * Furthermore, while we assume a single entry point to the function,
     * multiple exit points are common, and they may be tail calls.  To handle
     * this, we maintain a short list of exit points and perform our matching on
     * each.
     */
    top = instrlist_first(ilist);
    if (top == NULL || instr_get_next(top) == NULL) {
        /* zero or one instruction only, no callee save */
        return;
    }
    num_bots = analyze_find_exit_instrs(dcontext, ci, bots);
    if (num_bots >= MAX_EXIT_POINTS) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: bailing from setup analysis, too many exits\n");
        return;
    }

    top = skip_boring_instrs_fwd(top);
    skip_boring_instrs_bwd(bots, num_bots);

    /* Skip rets and jmps.  Other analyses need to see them. */
    for (i = 0; i < num_bots; i++) {
        if (instr_is_return(bots[i]) || instr_is_ubr(bots[i])) {
            bots[i] = instr_get_prev(bots[i]);
        }
    }
    skip_boring_instrs_bwd(bots, num_bots);

    /* Find and delete enter/leave or equivalent instructions.  We assume these
     * come before register saves and restores. */
    top = analyze_enter_leave(dcontext, ci, top, bots, num_bots);

    /* get the rest callee save regs */
    /* XXX: the callee save may be corrupted by memory update on the stack. */
    /* XXX: the callee save may use mov instead of push/pop */
    while (top != NULL && all_satisfy(pred_not_null, NULL, bots, num_bots)) {
        opnd_t reg_opnd;
        reg_id_t reg_id;
        /* Ignore nops and boring instructions. */
        top = skip_boring_instrs_fwd(top);
        skip_boring_instrs_bwd(bots, num_bots);
        /* If not in the first/last bb, break. */
        if (top == NULL || is_cti_or_label_instr(top))
            break;
        if (!all_satisfy(pred_not_null, NULL, bots, num_bots) ||
            any_satisfy((pred_fn_t)is_cti_or_label_instr, NULL, bots, num_bots))
            break;

        /* If we're doing a push on entry, all pops must match. */
        if (instr_get_opcode(top) == OP_push) {
            instr_t *pop_reg;
            reg_opnd = instr_get_src(top, 0);
            reg_id = opnd_get_reg(reg_opnd);
            pop_reg = INSTR_CREATE_pop(dcontext, reg_opnd);
            if (!all_satisfy((pred_fn_t)instr_same, pop_reg, bots, num_bots)) {
                instr_destroy(dcontext, pop_reg);
                break;
            }
            instr_destroy(dcontext, pop_reg);
        } else {
            break;  /* top was not a register save. */
        }

        /* TODO(rnk): Check that register was not clobbered before push. */

        /* it is callee save reg. */
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: callee "PFX" callee-saves reg %s at "PFX"\n",
               ci->start, get_register_name(reg_id), instr_get_app_pc(top));
        ci->callee_save_regs[reg_id - DR_REG_XAX] = true;

        /* Remove & destroy the push/pop pairs, we do our own save/restore if
         * inlined. */
        top = remove_top_bot_pairs(ilist, top, bots, num_bots);
    }
}

static void
analyze_callee_tls(void *dcontext, callee_info_t *ci)
{
    /* access to TLS means we do need to swap/preserve TEB/PEB fields
     * for library isolation (errno, etc.)
     */
    instr_t *instr;
    int i;
    ci->tls_used = false;
    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        /* we assume any access via app's tls is to app errno. */
        for (i = 0; i < instr_num_srcs(instr); i++) {
            opnd_t opnd = instr_get_src(instr, i);
            if (opnd_is_far_base_disp(opnd))
                ci->tls_used = true;
        }
        for (i = 0; i < instr_num_dsts(instr); i++) {
            opnd_t opnd = instr_get_dst(instr, i);
            if (opnd_is_far_base_disp(opnd))
                ci->tls_used = true;
        }
    }
    if (ci->tls_used) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: callee "PFX" accesses far memory\n", ci->start);
    }
}

static bool
is_fastpath(instr_t *instr)
{
    int length = 0;
    for (; instr != NULL; instr = instr_get_next(instr)) {
        if (instr_is_return(instr)) {
            return true;
        }
        if (instr_is_cti(instr)) {
            return false;
        }
        if (length > 5) {
            return false;
        }
        length++;
    }
    return false;
}

/* Turns callees with a fast path into an ilist that can be inlined.  Relies on
 * control flow being resolved, but rets still being present.
 */
static void
analyze_callee_partial(void *dc, callee_info_t *ci)
{
    instr_t *first_cti;
    instr_t *tgt_instr;
    instr_t *fallthrough_instr;
    instr_t *fastpath_start;
    instr_t *slowpath_start;
    instr_t *instr;
    instr_t *next_instr;
    opnd_t tgt;
    bool taken_fast, fallthrough_fast;

    if (opt_cleancall < 2) {
        dr_log(dc, LOG_CLEANCALL, 3,
               "CLEANCALL: partial inlining disabled: opt_cleancall: %d.\n",
               opt_cleancall);
        ci->opt_partial = false;
    }

    /* Find first branch target or control flow instruction. */
    for (first_cti  = instrlist_first(ci->ilist);
         first_cti != NULL;
         first_cti  = instr_get_next(first_cti)) {
        if (instr_is_cti(first_cti)) {
            break;
        }
    }
    if (first_cti == NULL || instr_is_return(first_cti)) {
        return;  /* No control flow, can't partial inline. */
    }
    if (!instr_is_cbr(first_cti)) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: cannot partially inline calllee "PFX": "
               "first cti is not cbr at "PFX".\n",
               ci->start, instr_get_app_pc(first_cti));
        return;
    }

    /* Find target. */
    tgt = instr_get_target(first_cti);
    if (!opnd_is_instr(tgt)) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: cannot partially inline calllee "PFX": "
               "control flow to non-label at "PFX".\n",
               ci->start, instr_get_app_pc(first_cti));
        return;
    }
    tgt_instr = opnd_get_instr(tgt);
    fallthrough_instr = instr_get_next(first_cti);

    /* Find fastpath, taken or fallthrough. */
    taken_fast = is_fastpath(tgt_instr);
    fallthrough_fast = is_fastpath(fallthrough_instr);
    if (taken_fast && fallthrough_fast) {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: both paths at "PFX" and "PFX" are fast.\n",
               instr_get_app_pc(fallthrough_instr), instr_get_app_pc(tgt_instr));
        return;
    } else if (taken_fast) {
        fastpath_start = tgt_instr;
        slowpath_start = fallthrough_instr;
    } else if (fallthrough_fast) {
        fastpath_start = fallthrough_instr;
        slowpath_start = tgt_instr;
    } else {
        dr_log(dc, LOG_CLEANCALL, 2,
               "CLEANCALL: cannot partially inline calllee "PFX": "
               "neither path is fast for cbr at "PFX".\n",
               ci->start, instr_get_app_pc(first_cti));
        return;
    }

    /* Make sure slowpath_start is a label.  Should only happen if fallthrough
     * is slow.  We set a note on the label so different call sites can find it
     * to handle call site specific saves, restores, or arg setup. */
    if (!fallthrough_fast && !instr_is_label(slowpath_start)) {
        instr_t *label = INSTR_CREATE_label(GLOBAL_DCONTEXT);
        PRE(ci->ilist, slowpath_start, label);
        slowpath_start = label;
    }
    DR_ASSERT(instr_is_label(slowpath_start));

    /* Remove slowpath instructions. */
    for (instr = instr_get_next(slowpath_start); instr != NULL; instr = next_instr) {
        /* Skip fastpath code if we encounter it. */
        if (instr == fastpath_start) {
            while (!instr_is_return(instr)) {
                instr = instr_get_next(instr);
            }
            instr = instr_get_next(instr);
        }
        if (instr == NULL)
            break;
        next_instr = instr_get_next(instr);
        remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
    }

    /* Emit slowpath transition code and insert jmp into slowpath.  We need to
     * rematerialize args here, but those are different for every call site.
     */
    dr_log(dc, LOG_CLEANCALL, 2,
           "CLEANCALL: callee is a candidate for partial inlining.\n");
    ci->opt_partial = true;

    /* Circular dependency issue: we can't emit the slowpath for the partially
     * inlined code until we've done further analysis of register and stack
     * usage.  We leave this call to a label and fix it up later.
     */
    ci->partial_label = INSTR_CREATE_label(GLOBAL_DCONTEXT);
    POST(ci->ilist, slowpath_start, INSTR_CREATE_call
        (GLOBAL_DCONTEXT, opnd_create_instr(ci->partial_label)));
    /* XXX: Insert the label into the ilist to avoid memory leaks if inlining
     * fails.  This makes the disassembly confusing. */
    POST(ci->ilist, slowpath_start, ci->partial_label);

    /* Re-calculate the number of instructions, since we just deleted a bunch.
     */
    ci->num_instrs = 0;
    for (instr = instrlist_first(ci->ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        ci->num_instrs++;
    }
}

static bool
opnd_is_stack_access(callee_info_t *ci, opnd_t opnd)
{
    return (opnd_is_base_disp(opnd) &&
            (ci->xbp_is_fp ?
             opnd_get_base(opnd) == DR_REG_XBP :
             opnd_get_base(opnd) == DR_REG_XSP));
}

/* Look for stack accesses.  Convert them to use our scratch slots. */
static void
analyze_callee_stack_usage(void *dcontext, callee_info_t *ci)
{
    instr_t *instr;
    instr_t *next_instr;
    opnd_t opnd, mem_ref, slot;
    int i;

    mem_ref = opnd_create_null();
    ci->stack_complex = false;
    ci->has_locals = false;
    for (instr  = instrlist_first(ci->ilist);
         instr != NULL;
         instr  = next_instr) {
        uint opc = instr_get_opcode(instr);
        next_instr = instr_get_next(instr);

        if (opc == OP_ret) {
            /* Ignore writes to XSP from returns. */
            continue;
        }

        /* sanity checks on stack usage */
        if (instr_writes_to_reg(instr, DR_REG_XBP) && ci->xbp_is_fp) {
            /* xbp must not be changed if xbp is used for frame pointer */
            dr_log(dcontext, LOG_CLEANCALL, 1,
                   "CLEANCALL: callee "PFX" cannot be inlined: XBP is updated.\n",
                   ci->start);
            ci->stack_complex = true;
            break;
        } else if (instr_writes_to_reg(instr, DR_REG_XSP)) {
            /* TODO(rnk): For partial inlining, it would be best to pull this
             * analysis code out so we can know the frame size.  For now we
             * cannot partially inline callees that adjust the stack in the
             * prologue. */
            /* stack pointer update, we only allow:
             * lea [xsp, disp] => xsp
             * xsp + imm_int => xsp
             * xsp - imm_int => xsp
             */
            if (ci->has_locals) {
                /* we do not allow stack adjustment after accessing the stack */
                ci->stack_complex = true;
            }
            if (opc == OP_lea) {
                /* lea [xsp, disp] => xsp */
                opnd = instr_get_src(instr, 0);
                if (!opnd_is_base_disp(opnd)           ||
                    opnd_get_base(opnd)  != DR_REG_XSP ||
                    opnd_get_index(opnd) != DR_REG_NULL)
                    ci->stack_complex = true;
            } else if (opc == OP_sub || opc == OP_add) {
                /* xsp +/- int => xsp */
                if (!opnd_is_immed_int(instr_get_src(instr, 0)))
                    ci->stack_complex = true;
            } else if (opc == OP_call && ci->opt_partial) {
                opnd = instr_get_target(instr);
                if (opnd_is_instr(opnd) &&
                    opnd_get_instr(opnd) == ci->partial_label) {
                    /* Don't let the rest of this loop examine this call
                     * instruction for local variable reads/writes; it won't
                     * understand it. */
                    continue;
                }
            } else {
                /* other cases like push/pop are not allowed */
                ci->stack_complex = true;
            }
            if (!ci->stack_complex) {
                /* Delete frame adjust, we do our own. */
                dr_log(dcontext, LOG_CLEANCALL, 3,
                       "CLEANCALL: removing frame adjustment at "PFX".\n",
                       instr_get_app_pc(instr));
                remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
                continue;
            } else {
                dr_log(dcontext, LOG_CLEANCALL, 1,
                       "CLEANCALL: callee "PFX" cannot be inlined: "
                       "complicated stack pointer update at "PFX".\n",
                    ci->start, instr_get_app_pc(instr));
                break;
            }
        } else if (instr_reg_in_src(instr, DR_REG_XSP) ||
                   (instr_reg_in_src(instr, DR_REG_XBP) && ci->xbp_is_fp)) {
            /* Detect stack address leakage */
            /* lea [xsp/xbp] */
            if (opc == OP_lea)
                ci->stack_complex = true;
            /* any direct use reg xsp or xbp */
            for (i = 0; i < instr_num_srcs(instr); i++) {
                opnd_t src = instr_get_src(instr, i);
                if (opnd_is_reg(src)) {
                    reg_id_t src_reg = opnd_get_reg(src);
                    if (reg_overlap(DR_REG_XSP, src_reg)  ||
                        (reg_overlap(DR_REG_XBP, src_reg) && ci->xbp_is_fp))
                        break;
                }
            }
            if (i != instr_num_srcs(instr))
                ci->stack_complex = true;
            if (ci->stack_complex) {
                dr_log(dcontext, LOG_CLEANCALL, 1,
                       "CLEANCALL: callee "PFX" cannot be inlined: "
                       "stack pointer leaked "PFX".\n",
                    ci->start, instr_get_app_pc(instr));
                break;
            }
        }

        /* Check how many stack variables the callee has.  We will not inline
         * the callee if it has more than one stack variable.  */
        if (instr_reads_memory(instr)) {
            for (i = 0; i < instr_num_srcs(instr); i++) {
                opnd = instr_get_src(instr, i);
                if (!opnd_is_stack_access(ci, opnd))
                    continue;
                if (!ci->has_locals) {
                    /* We see the first one, rembmer it. */
                    mem_ref = opnd;
                    ci->has_locals = true;
                } else if (!opnd_same(opnd, mem_ref)) {
                    /* Check if it is the same stack var as the one we saw.
                     * If different, no inline. 
                     */
                    dr_log(dcontext, LOG_CLEANCALL, 1,
                           "CLEANCALL: callee "PFX" cannot be inlined: "
                           "more than one stack location is accessed "PFX".\n",
                        ci->start, instr_get_app_pc(instr));
                    break;
                }
                /* replace the stack location with the last stack slot. */
                slot = OPND_CREATE_MEMPTR(DR_REG_XSP, 0);
                opnd_set_size(&slot, opnd_get_size(mem_ref));
                instr_set_src(instr, i, slot);
            }
            if (i != instr_num_srcs(instr)) {
                ci->stack_complex = true;
                break;
            }
        }
        if (instr_writes_memory(instr)) {
            for (i = 0; i < instr_num_dsts(instr); i++) {
                opnd = instr_get_dst(instr, i);
                if (!opnd_is_stack_access(ci, opnd))
                    continue;
                if (!ci->has_locals) {
                    mem_ref = opnd;
                    ci->has_locals = true;
                } else if (!opnd_same(opnd, mem_ref)) {
                    /* currently we only allows one stack refs */
                    dr_log(dcontext, LOG_CLEANCALL, 1,
                           "CLEANCALL: callee "PFX" cannot be inlined: "
                           "more than one stack location is accessed "PFX".\n",
                        ci->start, instr_get_app_pc(instr));
                    break;
                }
                /* replace the stack location with the last stack slot. */
                slot = OPND_CREATE_MEMPTR(DR_REG_XSP, 0);
                opnd_set_size(&slot, opnd_get_size(mem_ref));
                instr_set_dst(instr, i, slot);
            }
            if (i != instr_num_dsts(instr)) {
                ci->stack_complex = true;
                break;
            }
        }
    }

    if (ci->has_locals) {
        ci->framesize += sizeof(reg_t);
    }
    ci->framesize = ALIGN_FORWARD_UINT(ci->framesize, 16);
}

static void
analyze_callee_inline(void *dcontext, callee_info_t *ci)
{
    bool opt_inline = true;
    instr_t *instr;

    /* a set of condition checks */
    if (opt_cleancall < 2) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: opt_cleancall: %d.\n", 
               ci->start, opt_cleancall);
        opt_inline = false;
    }
    if (ci->num_instrs > MAX_NUM_INLINE_INSTRS) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: num of instrs: %d.\n", 
               ci->start, ci->num_instrs);
        opt_inline = false;
    }
    if (!ci->is_leaf && !ci->opt_partial) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: "
               "has complex control flow.\n", ci->start);
        opt_inline = false;
    }
    if (ci->num_xmms_used != 0) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: uses XMM.\n", 
               ci->start);
        opt_inline = false;
    }
    if (ci->tls_used) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: accesses TLS.\n",
               ci->start);
        opt_inline = false;
    }
    if (ci->num_regs_used > NUM_SCRATCH_SLOTS) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: "
               "uses too many scratch slots: %d > %d.\n",
               ci->start, ci->num_regs_used, NUM_SCRATCH_SLOTS);
        opt_inline = false;
    }
    if (ci->stack_complex) {
        dr_log(dcontext, LOG_CLEANCALL, 1,
               "CLEANCALL: callee "PFX" cannot be inlined: "
               "complex stack usage.\n", ci->start);
        opt_inline = false;
    }

    /* Remove trailing ret if we're going to inline. */
    /* TODO(rnk): Handle ret not being last instruction and multiple rets.
     * Idea: Create label at last ilist instruction and insert short jumps to
     * it. */
    instr = instrlist_last(ci->ilist);
    if (instr != NULL) {
        if (instr_is_return(instr)) {
            remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist, instr);
        } else {
            dr_log(dcontext, LOG_CLEANCALL, 1,
                   "CLEANCALL: callee "PFX" cannot be inlined: "
                   "last instruction is not return.\n", ci->start);
            opt_inline = false;
        }
    }

    ci->opt_inline = opt_inline;
    if (!opt_inline) {
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
        ci->ilist = NULL;
        return;
    }

    /* If we succeeded in partial inlining, emit the slowpath and fix up the
     * jmp to the label. */
    if (ci->opt_partial) {
        app_pc slowpath_pc = emit_partial_slowpath(dcontext, ci);
        for (instr = instrlist_first(ci->ilist); instr != NULL;
             instr = instr_get_next(instr)) {
            if (instr_is_call(instr) &&
                opnd_is_instr(instr_get_target(instr)) &&
                opnd_get_instr(instr_get_target(instr)) == ci->partial_label) {
                /* TODO(rnk): Reachability on X64, or does MAP_32BIT take care
                 * of this? */
                instr_set_target(instr, opnd_create_pc(slowpath_pc));
                /* Label is no longer needed. */
                remove_and_destroy(GLOBAL_DCONTEXT, ci->ilist,
                                   ci->partial_label);
                ci->partial_label = NULL;
                break;
            }
        }
    }
}

static void
analyze_callee_ilist(void *dc, callee_info_t *ci)
{
    ASSERT(!ci->bailout && ci->ilist != NULL);
    if (opt_cleancall < 1) {
        /* Only optimization at opt 0 is skipping unused registers. */
        analyze_callee_regs_usage(dc, ci);
        instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
        ci->ilist = NULL;
    } else {
        resolve_internal_brs(dc, ci);
        if (ci->bailout) {
            /* Happens for internal branches.  We let indirect branches through
             * for further analysis. */
            instrlist_clear_and_destroy(GLOBAL_DCONTEXT, ci->ilist);
            ci->ilist = NULL;
            return;
        }
        rewrite_pic_code(dc, ci);
        analyze_callee_cti(dc, ci);
        analyze_callee_setup(dc, ci);
        if (opt_cleancall >= 3) {
            analyze_callee_partial(dc, ci);
            optimize_callee(dc, ci);
        }
        analyze_callee_regs_usage(dc, ci);
        analyze_callee_tls(dc, ci);
        analyze_callee_stack_usage(dc, ci);
        analyze_callee_inline(dc, ci);
    }
}

static void
analyze_clean_call_aflags(void *dcontext, clean_call_info_t *cci,
                          instr_t *where)
{
    callee_info_t *ci = cci->callee_info;
    instr_t *instr;

    /* If there's a flags read, we clear the flags.  If there's a write or read,
     * we save them, because a read creates a clear which is a write. */
    cci->skip_clear_eflags = !ci->read_aflags;
    cci->skip_save_aflags  = !(ci->write_aflags || ci->read_aflags);
    /* XXX: this is a more aggressive optimization by analyzing the ilist
     * to be instrumented. The client may change the ilist, which violate
     * the analysis result. For example, 
     * I do not need save the aflags now if an instruction
     * after "where" updating all aflags, but later the client can
     * insert an instruction reads the aflags before that instruction.
     */
    if (opt_cleancall > 1 && !cci->skip_save_aflags) {
        for (instr = where; instr != NULL; instr = instr_get_next(instr)) {
            uint flags = instr_get_arith_flags(instr);
            if (TESTANY(EFLAGS_READ_6, flags) || instr_is_cti(instr))
                break;
            if (TESTALL(EFLAGS_WRITE_6, flags)) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "CLEANCALL: inserting clean call "PFX
                       ", skip saving aflags.\n", ci->start);
                cci->skip_save_aflags = true;
                break;
            }
        }
    }
}

static void
analyze_clean_call_regs(void *dcontext, clean_call_info_t *cci)
{
    uint i;
    callee_info_t *info = cci->callee_info;

    /* 1. xmm registers */
    for (i = 0; i < NUM_XMM_REGS; i++) {
        if (info->xmm_used[i]) {
            cci->xmm_skip[i] = false;
        } else {
            cci->xmm_skip[i] = true;
            cci->num_xmms_skip++;
        }
    }
    if (opt_cleancall > 2 && cci->num_xmms_skip != NUM_XMM_REGS)
        cci->should_align = false;
    /* 2. general purpose registers */
    /* set regs not to be saved for clean call */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (info->reg_used[i]) {
            cci->reg_skip[i] = false;
        } else {
            cci->reg_skip[i] = true;
            cci->num_regs_skip++;
        }
    }
    /* we need save/restore rax if save aflags because rax is used */
    if (!cci->skip_save_aflags && cci->reg_skip[0]) {
        dr_log(dcontext, LOG_CLEANCALL, 3,
               "CLEANCALL: if inserting clean call "PFX
               ", cannot skip saving reg xax.\n", info->start);
        cci->reg_skip[0] = false;
        cci->num_regs_skip--;
    }
}

static void
analyze_clean_call_args(void *dcontext,
                        clean_call_info_t *cci,
                        opnd_t *args)
{
    uint i, j, num_regparm;
    callee_info_t *ci = cci->callee_info;

    num_regparm = cci->num_args < dr_num_reg_parm() ? cci->num_args : dr_num_reg_parm();
    /* If a param uses a reg, DR need restore register value, which assumes
     * the full context switch with dr_mcontext_t layout,
     * in which case we need keep dr_mcontext_t layout.
     */
    cci->save_all_regs = false;
    for (i = 0; i < cci->num_args; i++) {
        if (opnd_is_reg(args[i]))
            cci->save_all_regs = true;
        for (j = 0; j < num_regparm; j++) {
            if (opnd_uses_reg(args[i], dr_reg_parm(j)))
                cci->save_all_regs = true;
        }
    }
    if (!ci->is_leaf) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: callee "PFX" is not a leaf, "
               "save all regs in dr_mcontext_t layout.\n",
            ci->start);
        cci->save_all_regs = true;
    }
}

static bool
analyze_clean_call_inline(void *dcontext, clean_call_info_t *cci)
{
    uint num_slots;
    callee_info_t *info = cci->callee_info;
    bool opt_inline = true;

    if (opt_cleancall <= 1) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: fail inlining clean call "PFX", opt_cleancall %d.\n",
               info->start, opt_cleancall);
        opt_inline = false;
    }
    if (cci->num_args > IF_X64_ELSE(dr_num_reg_parm(), 1)) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: fail inlining clean call "PFX", "
               "number of args %d > IF_X64_ELSE(NUM_REGPARM, 1).\n",
            info->start, cci->num_args);
        opt_inline = false;
    }
    if (cci->save_fpstate) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: fail inlining clean call "PFX", saving fpstate.\n",
               info->start);
        opt_inline = false;
    }
    if (!info->opt_inline) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: fail inlining clean call "PFX", complex callee.\n",
               info->start);
        opt_inline = false;
    }
    if (opt_inline) {
        /* Now check if have enough scratch slots */
        num_slots = NUM_GP_REGS - cci->num_regs_skip;
        if (info->has_locals) {
            /* one more slot for local variable */
            num_slots++;
        }
        if (!cci->skip_save_aflags) {
            /* one more slot for store app's eflags */
            num_slots++;
        }
#ifndef X64
        if (cci->num_args > 0 && cci->reg_skip[0]) {
            /* we use eax to put arg into tls */
            cci->num_regs_skip--;
            cci->reg_skip[0] = false;
            num_slots++;
        }
#endif
        if (num_slots > NUM_SCRATCH_SLOTS) {
            dr_log(dcontext, LOG_CLEANCALL, 2,
                   "CLEANCALL: fail inlining clean call "PFX
                   " need %d slots > available slots.\n",
                   info->start, num_slots, NUM_SCRATCH_SLOTS);
            opt_inline = false;
        }
    }
    if (!opt_inline) {
        if (cci->save_all_regs) {
            dr_log(dcontext, LOG_CLEANCALL, 2,
                   "CLEANCALL: inserting clean call "PFX
                   ", save all regs in dr_mcontext_t layout.\n",
                   info->start);
            cci->num_regs_skip = 0;
            memset(cci->reg_skip, 0, sizeof(bool) * NUM_GP_REGS);
            cci->num_xmms_skip = 0;
            memset(cci->xmm_skip, 0, sizeof(bool) * NUM_XMM_REGS);
            cci->skip_save_aflags = false;
            cci->should_align = true;
        } else {
            uint i;
            for (i = 0; i < NUM_GP_REGS; i++) {
                if (!cci->reg_skip[i] && info->callee_save_regs[i]) {
                    cci->reg_skip[i] = true;
                    cci->num_regs_skip++;
                }
            }
        }
        if (cci->num_xmms_skip == NUM_XMM_REGS) {
            STATS_INC(cleancall_xmm_skipped);
        }
        if (cci->skip_save_aflags) {
            STATS_INC(cleancall_aflags_save_skipped);
        }
        if (cci->skip_clear_eflags) {
            STATS_INC(cleancall_aflags_clear_skipped);
        }
    } else {
        /* TODO(rnk): Which is right!? */
        /* Use global dcontext, since we apply callee optimizations again,
         * which assume instrs are allocated using global dcontext. */
        cci->ilist = instrlist_clone(dcontext, info->ilist);
    }
    return opt_inline;
}

static bool
analyze_clean_call(void *dcontext, clean_call_info_t *cci, instr_t *where,
                   void *callee, bool save_fpstate, uint num_args, opnd_t *args)
{
    callee_info_t *ci;

    CLIENT_ASSERT(callee != NULL, "Clean call target is NULL");
    /* 1. init clean_call_info */
    clean_call_info_init(cci, callee, save_fpstate, num_args);
    /* 2. check runtime optimization options */
    if (opt_cleancall == 0)
        return false;
    /* 3. search if callee was analyzed before */
    ci = callee_info_table_lookup(callee);
    /* 4. this callee is not seen before */
    if (ci == NULL) {
        STATS_INC(cleancall_analyzed);
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: analyze callee "PFX"\n", callee);
        /* 4.1. create func_info */
        ci = callee_info_create((app_pc)callee, num_args);
        /* 4.2. decode the callee */
        decode_callee_ilist(dcontext, ci);
        /* 4.3. analyze the instrlist */
        if (!ci->bailout)
            analyze_callee_ilist(dcontext, ci);
        /* 4.4. add info into callee list */
        ci = callee_info_table_add(ci);
    }
    if (ci->num_args != num_args) {
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: failing to inline callee "PFX": "
               "originally called with %d args now called with %d args\n",
               callee, ci->num_args, num_args);
        return false;
    }
    cci->callee_info = ci;
    if (ci->bailout) {
        /* TODO(rnk): Doesn't this write race with other readers?  Should
         * callee info be conservatively reinitialized on bailout in the
         * callee analysis code? */
        callee_info_init(ci);
        ci->start = (app_pc)callee;
        return false;
    }
    /* 5. aflags optimization analysis */
    analyze_clean_call_aflags(dcontext, cci, where);
    /* 6. register optimization analysis */
    analyze_clean_call_regs(dcontext, cci);
    /* 7. check arguments */
    analyze_clean_call_args(dcontext, cci, args);
    /* 8. inline optimization analysis */
    if (analyze_clean_call_inline(dcontext, cci)) {
        return true;
    }
    /* by default, no inline optimization */
    return false;
}

/* Maps DR_REG_* enums to offsets into dr_mcontext_t.  Relies on register enum
 * ordering in instr.h.  Indexed by DR_REG_X* - DR_REG_XAX. */
const uint reg_mc_offset[NUM_GP_REGS] = {
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
static opnd_t
mc_frame_opnd(uint framesize, uint mc_offset)
{
    uint frame_offset = framesize - sizeof(dr_mcontext_t) + mc_offset;
    return OPND_CREATE_MEMPTR(DR_REG_XSP, frame_offset);
}

/* Saves a register to the mcontext at the base of the stack.  Assumes XSP is
 * at dstack - framesize.
 */
static void
insert_mc_reg_save(void *dc, uint framesize, instrlist_t *ilist,
                   instr_t *where, reg_id_t reg)
{
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dc, mc_frame_opnd(framesize, reg_mc_offset[reg - DR_REG_XAX]),
         opnd_create_reg(reg)));
}

/* Loads a register from the mcontext at the base of the stack.  Assumes XSP is
 * at dstack - framesize.
 */
static void
insert_mc_reg_restore(void *dc, uint framesize, instrlist_t *ilist,
                      instr_t *where, reg_id_t reg)
{
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, opnd_create_reg(reg),
         mc_frame_opnd(framesize, reg_mc_offset[reg - DR_REG_XAX])));
}

/* Saves aflags to the mcontext at the base of the stack.  Assumes XSP is at
 * dstack - framesize.  Assumes that XAX is dead and can be used as scratch.
 */
static void
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
static void
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

static opnd_t
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
#define XMM_SAVED_REG_SIZE  32
    uint psz = sizeof(reg_t);
    uint priv_size = IF_X64_ELSE(16, 8) * psz + 2 * psz;
    priv_size += PRE_XMM_PADDING + NUM_XMM_SLOTS * XMM_SAVED_REG_SIZE;
    priv_size = ALIGN_FORWARD_UINT(priv_size + sizeof(int) + sizeof(bool), psz);
    return priv_size + 3 * psz;
}

static void
insert_inline_reg_save(void *dc, clean_call_info_t *cci,
                       instrlist_t *ilist, instr_t *where, opnd_t *args)
{
    callee_info_t *ci = cci->callee_info;
    int i;
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);

    /* Spill XSP to TLS_XAX_SLOT because it's not exposed to the client. */
    PRE(ilist, where, INSTR_CREATE_mov_st(dc, opnd_get_tls_xax(dc), xsp));

    /* Switch to dstack and make room for a partially initialized mcontext
     * structure. */
    PRE(ilist, where, INSTR_CREATE_mov_ld(dc, xsp, opnd_get_tls_dcontext(dc)));
    PRE(ilist, where, INSTR_CREATE_mov_ld
        (dc, xsp, OPND_CREATE_MEMPTR(DR_REG_XSP, get_dstack_offset())));
    PRE(ilist, where, INSTR_CREATE_lea
        (dc, opnd_create_reg(DR_REG_XSP),
         OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, -ci->framesize)));

    ASSERT(cci->num_xmms_skip == NUM_XMM_REGS);
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (!cci->reg_skip[i]) {
            reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: inlining clean call "PFX", saving reg %s.\n",
                   ci->start, get_register_name(reg));
            insert_mc_reg_save(dc, ci->framesize, ilist, where, reg);
        }
    }

    if (!cci->skip_save_aflags) {
        DR_ASSERT_MSG(!cci->reg_skip[0], "XAX must be saved to save aflags!");
        insert_mc_flags_save(dc, ci->framesize, ilist, where);
    }
}

static void
insert_inline_reg_restore(void *dc, clean_call_info_t *cci,
                          instrlist_t *ilist, instr_t *where)
{
    int i;
    callee_info_t *ci = cci->callee_info;
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);

    /* aflags is first because it uses XAX. */
    if (!cci->skip_save_aflags) {
        insert_mc_flags_restore(dc, ci->framesize, ilist, where);
    }

    /* Now restore all registers. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        if (!cci->reg_skip[i]) {
            reg_id_t reg = DR_REG_XAX + (reg_id_t)i;
            dr_log(dc, LOG_CLEANCALL, 2,
                   "CLEANCALL: inlining clean call "PFX", restoring reg %s.\n",
                   ci->start, get_register_name(reg));
            insert_mc_reg_restore(dc, ci->framesize, ilist, where, reg);
        }
    }

    /* Switch back to app stack. */
    PRE(ilist, where, INSTR_CREATE_mov_ld(dc, xsp, opnd_get_tls_xax(dc)));
}

static reg_id_t
shrink_reg_for_param(reg_id_t regular, opnd_t arg)
{
#ifdef X64
    if (opnd_get_size(arg) == OPSZ_4) { /* we ignore var-sized */
        /* PR 250976 #2: leave 64-bit only if an immed w/ top bit set (we
         * assume user wants sign-extension; that is after all what happens
         * on a push of a 32-bit immed) */
        if (!opnd_is_immed_int(arg) ||
            (opnd_get_immed_int(arg) & 0x80000000) == 0)
            return reg_64_to_32(regular);
    }
#endif
    return regular;
}

static void
insert_inline_arg_setup(void *dcontext, clean_call_info_t *cci,
                        instrlist_t *ilist, instr_t *where, opnd_t *args,
                        bool is_slowpath)
{
    reg_id_t reg;
    uint i;
    callee_info_t *ci = cci->callee_info;

    if (cci->num_args == 0)
        return;

    ASSERT(cci->num_args <= IF_X64_ELSE(dr_num_reg_parm(), 1));
    for (i = 0; i < cci->num_args; i++) {
        reg = IF_X64_ELSE(dr_reg_parm(i), DR_REG_XAX);
        if (!ci->reg_used[reg - DR_REG_XAX]) {
            if (!is_slowpath) {
                dr_log(dcontext, LOG_CLEANCALL, 2,
                       "CLEANCALL: skipping arg setup for dead reg %s\n",
                       get_register_name(reg));
                continue;
            }
        }
        reg = shrink_reg_for_param(reg, args[i]);
        dr_log(dcontext, LOG_CLEANCALL, 2,
               "CLEANCALL: inlining clean call "PFX", passing arg via reg %s.\n",
               ci->start, get_register_name(reg));
        if (opnd_is_immed_int(args[i])) {
            PRE(ilist, where, INSTR_CREATE_mov_imm
                (dcontext, opnd_create_reg(reg), args[i]));
        } else {
            PRE(ilist, where, INSTR_CREATE_mov_ld
                (dcontext, opnd_create_reg(reg), args[i]));
        }
    }
#ifndef X64
    ASSERT(!cci->reg_skip[0]);
    /* Move xax to the local variable stack slot.  We can use the local
     * variable stack slot because we only allow at most one local stack
     * access, so callee either does not use the argument, or the local stack
     * access is the arg.
     */
    dr_log(dcontext, LOG_CLEANCALL, 2,
           "CLEANCALL: inlining clean call "PFX", passing arg via slot %d.\n",
           ci->start, NUM_SCRATCH_SLOTS - 1);
    PRE(ilist, where, INSTR_CREATE_mov_st
        (dcontext, OPND_CREATE_MEMPTR(DR_REG_XSP, 0),
         opnd_create_reg(DR_REG_XAX)));
#endif
}

/* Insert call-site specific code for switching from the partially inlined fast
 * path to the out-of-line slowpath.  For example, arguments may need to be
 * rematerialized.
 */
static void
insert_inline_slowpath(void *dc, clean_call_info_t *cci, opnd_t *args)
{
    callee_info_t *ci = cci->callee_info;
    instrlist_t *ilist = cci->ilist;
    instr_t *instr;
    instr_t *slowpath_call = NULL;
    uint i;

    ASSERT(ci->opt_partial);
    for (instr = instrlist_first(ilist); instr != NULL;
         instr = instr_get_next(instr)) {
        /* If we find the slowpath, insert call-site specific setup. */
        if (ci->opt_partial && instr_is_call(instr) &&
            opnd_is_pc(instr_get_target(instr)) &&
            opnd_get_pc(instr_get_target(instr)) == ci->partial_pc) {
            slowpath_call = instr;
            break;
        }
    }
    ASSERT(slowpath_call != NULL);

    /* XXX: If the callee didn't use XAX and did use flags, we may have saved
     * XAX in order to save flags inline.  The slowpath assumes the inline code
     * always saves XAX if there is any flags usage.  If aflags were dead so we
     * didn't save flags and hence XAX, we will need to save XAX here. */
    if (!ci->reg_used[DR_REG_XAX - DR_REG_XAX] &&
        (ci->read_aflags || ci->write_aflags) &&
        cci->skip_save_aflags &&
        cci->reg_skip[DR_REG_XAX - DR_REG_XAX]) {
        insert_mc_reg_save(dc, ci->framesize, ilist, slowpath_call, DR_REG_XAX);
        insert_mc_reg_restore(dc, ci->framesize, ilist,
                              instr_get_next(slowpath_call), DR_REG_XAX);
    }

    /* Arg setup may use registers that weren't used inline, and therefore
     * won't be saved. */
    for (i = 0; i < cci->num_args; i++) {
        reg_id_t reg = IF_X64_ELSE(dr_reg_parm(i), DR_REG_XAX);
        if (!ci->reg_used[reg - DR_REG_XAX]) {
            insert_mc_reg_save(dc, ci->framesize, ilist, slowpath_call, reg);
            insert_mc_reg_restore(dc, ci->framesize, ilist,
                                  instr_get_next(slowpath_call), reg);
        }
    }

    /* Assert that reg_skip and reg_used agree, except in the case of XAX,
     * which we deal with above. */
    for (i = 0; i < NUM_GP_REGS; i++) {
        DR_ASSERT_MSG(ci->reg_used[i] == !cci->reg_skip[i] ||
                      (i == 0 && !cci->reg_skip[i]),
                      "reg_used and reg_skip don't agree!");
    }

    /* TODO(rnk): Could use an analysis to see which args are still live, since
     * chances are most are.  The slowpath is not executed frequently, but
     * optimizing it should reduce code size. */
    insert_inline_arg_setup(dc, cci, ilist, slowpath_call, args, true);
}

/* TODO(rnk): Copied from optimize_callee.  We duplicated it so we can
 * remove_and_destroy with our thread-local dcontext instead of the global
 * dcontext.
 */
static void
try_fold_immeds(void *dc, instrlist_t *ilist)
{
    instr_t *instr, *next_instr;
    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: ilist before fold_immeds:\n");
    DOLOG(3, LOG_CLEANCALL, {
        instrlist_disassemble(dc, NULL, ilist, dr_get_logfile(dc));
    });
    for (instr = instrlist_first(ilist); instr != NULL; instr = next_instr) {
        next_instr = instr_get_next(instr);
        if (instr_is_mov(instr) &&
            opnd_is_immed_int(instr_get_src(instr, 0)) &&
            opnd_is_reg(instr_get_dst(instr, 0))) {
            if (fold_mov_immed(dc, ilist, instr)) {
                remove_and_destroy(dc, ilist, instr);
            }
        }
    }
    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: ilist after fold_immeds:\n");
    DOLOG(3, LOG_CLEANCALL, {
        instrlist_disassemble(dc, NULL, ilist, dr_get_logfile(dc));
    });
}

static void
insert_inline_clean_call(void *dcontext, clean_call_info_t *cci,
                         instrlist_t *ilist, instr_t *where, opnd_t *args)
{
    callee_info_t *ci = cci->callee_info;
    instrlist_t *callee = cci->ilist;
    instr_t *instr;

    /* Insert argument setup.  Do some optimizations to try to fold the
     * arguments in. */
    /* TODO(rnk): We can't run the full set of optimizations because they
     * assume the ilist was allocated from GLOBAL_DCONTEXT, which this is not.
     */
    insert_inline_arg_setup(dcontext, cci, callee, instrlist_first(callee),
                            args, false);
    try_fold_immeds(dcontext, callee);
    if (ci->opt_partial) {
        insert_inline_slowpath(dcontext, cci, args);
    }

    ASSERT(cci->ilist != NULL);
    /* 0. update stats */
    STATS_INC(cleancall_inlined);
    /* 1. save registers */
    insert_inline_reg_save(dcontext, cci, ilist, where, args);
    /* 4. inline clean call ilist */
    instr = instrlist_first(callee);
    while (instr != NULL) {
        instrlist_remove(callee, instr);
        instr_set_translation(instr, NULL);
        /* The inlined code might cause access violation, so set to may fault.
         * XXX: anything else we should do?
         */
        instr_set_meta_may_fault(instr, true);
        instrlist_meta_preinsert(ilist, where, instr);
        instr = instrlist_first(callee);
    }
    instrlist_destroy(dcontext, callee);
    cci->ilist = NULL;
    /* 5. restore registers */
    insert_inline_reg_restore(dcontext, cci, ilist, where);
    /* XXX: the inlined code looks like this 
     *   mov    %rax -> %gs:0x00 
     *   mov    %rdi -> %gs:0x01 
     *   mov    $0x00000003 -> %edi 
     *   mov    <rel> 0x0000000072200c00 -> %rax 
     *   movsxd %edi -> %rdi 
     *   add    %rdi (%rax) -> (%rax) 
     *   mov    %gs:0x00 -> %rax 
     *   mov    %gs:0x01 -> %rdi 
     *   ...
     * we can do some constant propagation optimization here,
     * leave it for higher optimization level.
     */
}

/* Save or restore all application registers that weren't saved inline. */
static void
insert_slowpath_mc_regs(void *dc, callee_info_t *ci, bool save,
                        instrlist_t *ilist)
{
    reg_id_t reg;
    int i;
    uint framesize = ci->framesize + 16;  /* ret addr + alignment */

    for (reg = DR_REG_XAX; reg < DR_REG_XAX + NUM_GP_REGS; reg++) {
        bool is_arg;
        /* If the register was used inline it was saved inline. */
        if (ci->reg_used[reg - DR_REG_XAX])
            continue;
        /* XXX: If flags had to be saved, then XAX was saved inline and
         * clobbered, so we should *not* save it. */
        if (reg == DR_REG_XAX && (ci->read_aflags || ci->write_aflags))
            continue;
        /* If the register was used to pass an argument, it must have been
         * saved inline.  The callee may not actually *use* the argument
         * inline, so we have to check this case as well. */
        is_arg = false;
        for (i = 0; i < ci->num_args; i++) {
            if (dr_reg_parm(i) == reg) {
                is_arg = true;
                break;
            }
        }
        if (is_arg)
            continue;

        if (save)
            insert_mc_reg_save(dc, framesize, ilist, NULL, reg);
        else
            insert_mc_reg_restore(dc, framesize, ilist, NULL, reg);
    }
}

/* Save or restore flags if it wasn't saved inline. */
static void
insert_slowpath_mc_flags(void *dc, callee_info_t *ci, bool save,
                         instrlist_t *ilist)
{
    /* Save flags, but only if they weren't already saved, which happens if
     * there was either a read or a write (a read implies a clear, which is a
     * write). */
    /* TODO(rnk): Unduplicate this logic with cci->skip_save_aflags. */
    if (!(ci->read_aflags || ci->write_aflags)) {
        /* TODO(rnk): This clobbers XAX, which could be used to pass args into
         * slowpath. */
        /* The + 16 is for ret addr + alignment. */
        if (save)
            insert_mc_flags_save(dc, ci->framesize + 16, ilist, NULL);
        else
            insert_mc_flags_restore(dc, ci->framesize + 16, ilist, NULL);
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

static void
insert_slowpath_mc(void *dc, callee_info_t *ci, bool save,
                   instrlist_t *ilist)
{
    uint framesize = ci->framesize + 16;  /* ret addr + alignment */

    /* Save/restore flags uses XAX, so on save it must come after regs, and on
     * restore it must come first. */
    if (save) {
        insert_slowpath_mc_regs(dc, ci, save, ilist);
        insert_slowpath_mc_flags(dc, ci, save, ilist);
    } else {
        insert_slowpath_mc_flags(dc, ci, save, ilist);
        insert_slowpath_mc_regs(dc, ci, save, ilist);
    }

    /* Save XMM regs, if appropriate. */
    if (dr_mcontext_xmm_fields_valid()) {
        /* PR 264138: we must preserve xmm0-5 if on a 64-bit kernel */
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
            APP(ilist, instr_create_1dst_1src(dc, opcode, dst, src));
        }
    }

    /* FIXME i#433: need DR cxt switch and clean call to preserve ymm */
}

/* Emit the slowpath back to the callee for partially inlined functions.
 * Return the slowpath entry point. */
static app_pc
emit_partial_slowpath(void *dc, callee_info_t *ci)
{
    byte *entry;
    instrlist_t *ilist;
    opnd_t xsp = opnd_create_reg(DR_REG_XSP);
    uint realignment;

    dr_log(dc, LOG_CLEANCALL, 3,
           "CLEANCALL: emitting partial inline slowpath\n");

    /* Generate the clean call ilist.  Arguments should be materialized into
     * registers at the callsite.  Application register values are already on
     * the stack.
     */
    ilist = instrlist_create(dc);
    /* Re-align stack to 16 bytes to prepare for call. */
    realignment = (16 - sizeof(reg_t));
    APP(ilist, INSTR_CREATE_lea
        (dc, xsp, OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, -realignment)));
    insert_slowpath_mc(dc, ci, /*save=*/true, ilist);
    APP(ilist, INSTR_CREATE_call(dc, opnd_create_pc(ci->start)));
    insert_slowpath_mc(dc, ci, /*save=*/false, ilist);
    /* Un-align stack to get back to ret addr. */
    APP(ilist, INSTR_CREATE_lea
        (dc, xsp, OPND_CREATE_MEM_lea(DR_REG_XSP, DR_REG_NULL, 0, realignment)));
    APP(ilist, INSTR_CREATE_ret(dc));

    dr_log(dc, LOG_CACHE, 3, "drcalls: emitting partial slowpath\n");
    entry = code_cache_emit(dc, ilist);

    instrlist_clear_and_destroy(dc, ilist);

    ci->partial_pc = entry;
    return entry;
}
