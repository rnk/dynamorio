/* *******************************************************************************
 * Copyright (c) 2012 Google Inc.  All rights reserved.
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
 * * Neither the name of Google, Inc nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL GOOGLE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/* Standard string manipulation routines.  We reimplement these here to avoid
 * extra dependence on libc.
 */

#include "globals.h"

#include <string.h>

char *
strncpy(char *dst, const char *src, size_t n)
{
    ssize_t i;
    for (i = 0; i < n && src[i] != '\0'; i++) {
        dst[i] = src[i];
    }
    for (; i < n; i++) {
        dst[i] = '\0';
    }
    return dst;
}

char *
strchr(const char *s, int c)
{
    ssize_t i;
    for (i = 0; true; i++) {
        if (s[i] == c)
            return (char*)&s[i];
        if (s[i] == '\0')
            return NULL;
    }
}

char *
index(const char *s, int c)
{
    return strchr(s, c);
}

char *
strrchr(const char *s, int c)
{
    ssize_t i;
    char *last_c = NULL;
    for (i = 0; s[i] != '\0'; i++) {
        if (s[i] == c)
            last_c = (char*)&s[i];
    }
    return last_c;
}

char *
rindex(const char *s, int c)
{
    return strrchr(s, c);
}

size_t
strlen(const char *s)
{
    return (size_t)(strchr(s, '\0') - s);
}

int
strncmp(const char *a, const char *b, size_t n)
{
    size_t i;
    int c = 0;
    for (i = 0; i < n; i++) {
        c = a[i] - b[i];
        if (c != 0)
            break;
    }
    return c;
}

char *
strstr(const char *haystack, const char *needle)
{
    size_t len = strlen(needle);
    char *cur = (char*)haystack;
    while (*cur != '\0' && strncmp(cur, needle, len) != 0)
        cur++;
    if (*cur == '\0')
        return NULL;
    return cur;
}

