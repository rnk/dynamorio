/* **********************************************************
 * Copyright (c) 2013 Google, Inc.  All rights reserved.
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

/* Intercepts module transitions for native execution for ELF modules.
 */

#include "../globals.h"
#include "../native_exec.h"
#include "module.h"
#include "instr.h"
#include "instr_create.h"
#include "disassemble.h"

/* According to the SysV amd64 psABI docs, there are three reserved entries
 * in the PLTGOT:
 * 1. offset to .dynamic section
 * 2. available for loader data, used for link map
 * 3. pointer to resolution stub, used for _dl_runtime_resolve
 *
 * We want to replace 3, _dl_runtime_resolve, with a stub in x86.asm.  See
 * Figure 5.2 for more details.
 *
 * FIXME: Find documentation that this is the same on ia32 and test it.
 */
enum { DL_RUNTIME_RESOLVE_IDX = 2 };

/* The loader's _dl_fixup. */
void *(*app_dl_fixup)(void *link_map, uint dynamic_index);

static bool
module_contains_pc(module_area_t *ma, app_pc pc)
{
    return (pc >= ma->start && pc < ma->end);
}

/* Creates a template stub copied repeatedly for each stub we need to create.
 */
static void
initialize_stub(native_stub_t *stub)
{
    dcontext_t *dc = GLOBAL_DCONTEXT;
    instrlist_t *ilist = instrlist_create(dc);
    app_pc code_end = stub->code+BUFFER_SIZE_BYTES(stub->code);
    app_pc next_pc;
    /* %r11 is scratch on x64 and the PLT resolver uses it, so we do too.  For
     * ia32, there are scratch regs, but the loader doesn't use them.  Presumably
     * it doesn't want to break special calling conventions, so we follow suit
     * and push onto the stack.
     */
#ifdef X64
    instrlist_append(ilist, INSTR_CREATE_mov_imm
                     (dc, opnd_create_reg(DR_REG_R11), OPND_CREATE_INTPTR(0)));
#else
    instrlist_append(ilist, INSTR_CREATE_push_imm
                     (dc, opnd_create_reg(DR_REG_R11), OPND_CREATE_INTPTR(0)));
#endif
    instrlist_append(ilist, INSTR_CREATE_jmp
                     (dc, opnd_create_pc(0)));
    next_pc = instrlist_encode_to_copy(dc, ilist, stub->code, NULL, code_end,
                                       false);
    SET_TO_NOPS(next_pc, code_end - next_pc);
    /* We need to get the offsets of the operands.  We assume the operands are
     * encoded as the last part of the instruction.
     */
    stub->tgt_offset = instr_length(dc, instrlist_first(ilist)) - sizeof(void*);
    stub->jmp_offset = instr_length(dc, instrlist_last(ilist)) - sizeof(uint);
    instrlist_clear_and_destroy(dc, ilist);
}

/* Redirect this jump slot to our interceptor stub.  See
 * module_relocate_symbol() in module.c for what a normal loader would do.
 */
void
hook_plt_slot(module_area_t *ma, os_privmod_data_t *opd,
              native_stub_t *stub_tmpl, ELF_REL_TYPE *rel)
{
    ELF_ADDR *r_addr;
    uint r_type;
    ptr_int_t addend = 0;
    app_pc resolved_addr;
    app_pc stub_pc;

    r_addr = (ELF_ADDR *)(rel->r_offset + opd->load_delta);
    r_type = (uint)ELF_R_TYPE(rel->r_info);
    if (r_type != ELF_R_JUMP_SLOT)
        return;  /* Not a PLT relocation. */
    resolved_addr = (app_pc) *r_addr;
    DOLOG(4, LOG_LOADER, {
        uint r_sym = ELF_R_SYM(rel->r_info);
        ELF_SYM_TYPE *sym = &((ELF_SYM_TYPE *)opd->os_data.dynsym)[r_sym];
        const char *name = (char *)opd->os_data.dynstr + sym->st_name;
        print_file(STDERR, "name: %s\n", name);
        print_file(STDERR, "ma: "PFX"-"PFX", addr: "PFX", %d\n",
                   ma->start, ma->end, resolved_addr, module_contains_pc(ma, resolved_addr));
        app_pc pc = resolved_addr;
        pc = disassemble(GLOBAL_DCONTEXT, pc, STDERR);
        pc = disassemble(GLOBAL_DCONTEXT, pc, STDERR);
        pc = disassemble(GLOBAL_DCONTEXT, pc, STDERR);
    });
    /* FIXME: Catch pointers to lazy PLT stub resolution. */
    if (module_contains_pc(ma, resolved_addr))
        return;  /* Ignore calls resolved to this module. */
    stub_pc = native_create_stub(stub_tmpl, resolved_addr,
                                 (app_pc) native_plt_call);

    /* We don't need to look at r_addend.  The loader already added it. */
    *r_addr = (ELF_ADDR) stub_pc;
}

void
hook_plt_imports(module_area_t *ma, os_privmod_data_t *opd, bool at_map)
{
    /* FIXME: Move this template into native_exec.c. */
    native_stub_t stub;
    initialize_stub(&stub);

    /* We only care about jump slot relocations. */
    if (opd->jmprel != NULL) {
        app_pc jmprel = opd->jmprel;
        app_pc jmprelend = opd->jmprel + opd->pltrelsz;
        bool is_rela = (opd->pltrel == DT_RELA);
        size_t relentsz = (is_rela ? sizeof(ELF_RELA_TYPE) : sizeof(ELF_REL_TYPE));
        ASSERT(opd->pltrel == DT_REL || opd->pltrel == DT_RELA);
        print_file(STDERR, "jmprel: %p, jmprelend: %p\n", jmprel, jmprelend);
        for (jmprel = opd->jmprel; jmprel < jmprelend; jmprel += relentsz) {
            ELF_REL_TYPE *rel = (ELF_REL_TYPE *) jmprel;
            hook_plt_slot(ma, opd, &stub, rel, is_rela);
        }
    }
}

/* If we are not at_map, then we assume the module has been relocated.
 * FIXME: Catch transitions out of un-relocated modules.
 */
void
module_hook_transitions(module_area_t *ma, bool at_map)
{
    os_privmod_data_t opd;
    app_pc relro_base;
    size_t relro_size;
    bool got_unprotected = false;
    app_pc *pltgot;

    memset(&opd, 0, sizeof(opd));
    module_walk_program_headers(ma->start, ma->end - ma->start, at_map,
                                NULL, NULL, NULL, &opd.os_data);
    module_get_os_privmod_data(ma->start, ma->end - ma->start,
                               !at_map/*relocated*/, &opd);

    /* _dl_runtime_resolve is typically inside the relro region, so we must
     * unprotect it.
     */
    if (!at_map && module_get_relro(ma->start, &relro_base, &relro_size)) {
        os_set_protection(relro_base, relro_size, MEMPROT_READ|MEMPROT_WRITE);
    }

    pltgot = (app_pc *) opd.pltgot;
    pltgot[DL_RUNTIME_RESOLVE_IDX] = (app_pc) _dynamorio_runtime_resolve;
    hook_plt_imports(ma, at_map);

    if (got_unprotected) {
        /* XXX: This may not be symmetric, but we trust PT_GNU_RELRO for now. */
        os_set_protection(relro_base, relro_size, MEMPROT_READ);
    }
}

#if 0
/* NOCHECKIN */

void
find_dl_fixup(dcontext_t *dcontext, app_pc resolver)
{
    instr_t instr;
    int max_decodes = 30;
    int i = 0;
    app_pc pc = resolver;

    instr_init(dcontext, &instr);
    while (pc != NULL && i < max_decodes) {
        pc = decode(dcontext, pc, &instr);
        if (instr_get_opcode(&instr) == OP_call) {
            opnd_t tgt = instr_get_target(&instr);
            app_dl_fixup = (void *(*)(void *, uint)) opnd_get_pc(tgt);
            break;
        } else if (instr_is_cti(&instr)) {
            break;
        }
        instr_reset(dcontext, &instr);
    }
    instr_free(dcontext, &instr);
}

#endif
