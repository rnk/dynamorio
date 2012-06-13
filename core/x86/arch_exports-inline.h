#include "instr.h"

int
instr_length_outofline(dcontext_t *dcontext, instr_t *instr);

FOO int
instr_length(dcontext_t *dcontext, instr_t *instr)
{
    if (__builtin_expect(instr_needs_encoding(instr), 0))
        return instr_length_outofline(dcontext, instr);
    return instr->length;
}

/* returns an empty instr_t object */
FOO instr_t*
instr_create(dcontext_t *dcontext)
{
    instr_t *instr = (instr_t*) heap_alloc(dcontext, sizeof(instr_t)
                                           HEAPACCT(ACCT_IR));
    instr_init(dcontext, instr);
    return instr;
}

/* zeroes out the fields of instr */
FOO void
instr_init(dcontext_t *dcontext, instr_t *instr)
{
    memset((void *)instr, 0, sizeof(instr_t));
    IF_X64(instr_set_x86_mode(instr, get_x86_mode(dcontext)));
}

/* Frees all dynamically allocated storage that was allocated by instr
 * Also zeroes out instr's fields
 * This instr must have been initialized before!
 */
FOO void 
instr_reset(dcontext_t *dcontext, instr_t *instr)
{
    instr_free(dcontext, instr);
    instr_init(dcontext, instr);
}

/* deletes the instr_t object with handle "inst" and frees its storage */
FOO void
instr_destroy(dcontext_t *dcontext, instr_t *instr)
{
    instr_free(dcontext, instr);

    /* CAUTION: assumes that instr is not part of any instrlist */
    heap_free(dcontext, instr, sizeof(instr_t) HEAPACCT(ACCT_IR));
}

/* set the note field of instr to value */
FOO void 
instr_set_note(instr_t *instr, void *value)
{
    instr->note = value;
}

/* return the note field of instr */
FOO void *
instr_get_note(instr_t *instr)
{
    return instr->note;
}

/* return instr->next */
FOO instr_t*
instr_get_next(instr_t *instr)
{
    return instr->next;
}

/* return instr->prev */
FOO instr_t*
instr_get_prev(instr_t *instr)
{
    return instr->prev;
}

/* set instr->next to next */
FOO void
instr_set_next(instr_t *instr, instr_t *next)
{
    instr->next = next;
}

/* set instr->prev to prev */
FOO void
instr_set_prev(instr_t *instr, instr_t *prev)
{
    instr->prev = prev;
}
