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

/* Macros copied from DR core for compatibility. */

#ifndef DRCALLS_EXT_COMPAT_H
#define DRCALLS_EXT_COMPAT_H

/* Standard alignment hack. */
#define ALIGN_FORWARD(x, alignment) \
    ((((ptr_uint_t)x) + ((alignment)-1)) & (~((alignment)-1)))
#define ALIGN_FORWARD_UINT(x, alignment) \
    ((((uint)x) + ((alignment)-1)) & (~((alignment)-1)))

#define MAX(x, y) ((x) >= (y) ? (x) : (y))

#define TESTANY(mask, var) (((mask) & (var)) != 0)
#define TESTALL(mask, var) (((mask) & (var)) == (mask))

/* Make code more readable by shortening long lines.  We mark all as meta to
 * avoid client interface asserts.
 */
#define POST instrlist_meta_postinsert
#define PRE  instrlist_meta_preinsert
#define APP  instrlist_meta_append

#ifdef DEBUG
# define ASSERT DR_ASSERT
/* TODO(rnk): Maybe expose curiosity assertions to clients. */
# define ASSERT_CURIOSITY DR_ASSERT
# define CLIENT_ASSERT DR_ASSERT_MSG
#else
# undef DR_ASSERT
# undef DR_ASSERT_MSG
# define DR_ASSERT(cond)
# define DR_ASSERT_MSG(cond, msg)
# define ASSERT(cond)
# define ASSERT_CURIOSITY(cond)
# define CLIENT_ASSERT(cond, msg)
#endif

#ifdef DEBUG
/* TODO(rnk): It'd be nice if DOLOG actually checked the mask and level. */
# define DOLOG(level, mask, stmt) stmt
# define DEBUG_DECLARE(decl) decl
# define IF_DEBUG(stmt) stmt
# define IF_DEBUG(stmt) stmt
#else
# define DOLOG(level, mask, stmt)
# define DEBUG_DECLARE(decl)
# define IF_DEBUG(stmt)
# define IF_DEBUG(stmt)
#endif

/* TODO(rnk): We use this to allocate thread-shared instrlists, but it violates
 * the abstraction barrier. */
#define GLOBAL_DCONTEXT  ((void*)(ptr_uint_t)-1)

/* TODO(rnk): Expose TRY/EXCEPT/FINALLY utils to clients. */
#define TRY_EXCEPT(dc, try_stmt, except_stmt) try_stmt

/* TODO(rnk): Clients can't do this, but maybe it's OK for us to do it since
 * we're an extension. */
#define TLS_XAX_SLOT 0

#define STATS_INC(stat)

/* TODO(rnk): DR should export this. */
#ifdef X64
# define NUM_XMM_REGS 16
# define NUM_GP_REGS  16
#else
# define NUM_XMM_REGS 8
# define NUM_GP_REGS  8
#endif

#ifdef WINDOWS
# define NUM_XMM_SAVED 6 /* xmm0-5; for 32-bit we have space for xmm0-7 */
#else
# ifdef X64
#  define NUM_XMM_SAVED 16 /* xmm0-15 */
# else
/* i#139: save xmm0-7 registers in 32-bit Linux. */
#  define NUM_XMM_SAVED 8 /* xmm0-7 */
# endif
#endif

#define YMM_ENABLED() (proc_has_feature(FEATURE_AVX))
#define REG_SAVED_XMM0 (YMM_ENABLED() ? DR_REG_YMM0 : DR_REG_XMM0)
#define XMM_SAVED_REG_SIZE  32

#define MIN(x,y) ((x) <= (y) ? (x) : (y))

#endif /* DRCALLS_EXT_COMPAT_H */
