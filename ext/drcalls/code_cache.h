#ifndef DRCALLS_CODE_CACHE_H
#define DRCALLS_CODE_CACHE_H

void code_cache_init(void);
void code_cache_destroy(void);
app_pc code_cache_emit(void *dc, instrlist_t *ilist);

#endif /* DRCALLS_CODE_CACHE_H */
