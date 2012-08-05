/* **********************************************************
 * Copyright (c) 2011-2012 Google, Inc.  All rights reserved.
 * Copyright (c) 2008-2010 VMware, Inc.  All rights reserved.
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
 * * Neither the name of VMware, Inc. nor the names of its contributors may be
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

#include <stdlib.h>  /* for malloc */
#include <stdio.h>
#include <string.h>
#include "configure.h" /* since not including share.h */
#include "utils.h"
#include "globals_shared.h"
#include "utils.h"
#include "lib/dr_config.h"
#include "our_tchar.h"

#ifdef WINDOWS
# include <windows.h>
# include <io.h> /* for _get_osfhandle */
# include "processes.h"
# include "mfapi.h" /* for PLATFORM_WIN_2000 */

# define RELEASE32_DLL   _TEXT("\\lib32\\release\\dynamorio.dll")
# define DEBUG32_DLL     _TEXT("\\lib32\\debug\\dynamorio.dll")
# define RELEASE64_DLL   _TEXT("\\lib64\\release\\dynamorio.dll")
# define DEBUG64_DLL     _TEXT("\\lib64\\debug\\dynamorio.dll")
# define LOG_SUBDIR      _TEXT("\\logs")
# define LIB32_SUBDIR    _TEXT("\\lib32")
# define PREINJECT32_DLL _TEXT("\\lib32\\drpreinject.dll")
# define PREINJECT64_DLL _TEXT("\\lib64\\drpreinject.dll")

# define EXPORT __declspec(dllexport)
#else
# include <sys/stat.h>
# include <sys/types.h>
# include <unistd.h>

# define RELEASE32_DLL   "/lib32/release/libdynamorio.so"
# define DEBUG32_DLL     "/lib32/debug/libdynamorio.so"
# define RELEASE64_DLL   "/lib64/release/libdynamorio.so"
# define DEBUG64_DLL     "/lib64/debug/libdynamorio.so"
# define LOG_SUBDIR      "/logs"
# define LIB32_SUBDIR    "/lib32/"

# define EXPORT __attribute__((visibility("default")))
#endif

/* The minimum option size is 3, e.g., "-x ".  Note that we need the
 * NULL term too so "-x -y" needs 6 characters.
*/
#define MAX_NUM_OPTIONS (DR_MAX_OPTIONS_LENGTH / 3)

/* Data structs to hold info about the DYNAMORIO_OPTION registry entry */
typedef struct _client_opt_t {
    TCHAR *path;
    client_id_t id;
    TCHAR *opts;
} client_opt_t;

typedef struct _opt_info_t {
    dr_operation_mode_t mode;
    TCHAR *extra_opts[MAX_NUM_OPTIONS];
    size_t num_extra_opts;
    /* note that clients are parsed and stored in priority order */
    client_opt_t *client_opts[MAX_CLIENT_LIBS];
    size_t num_clients;
} opt_info_t;

/* Does a straight copy when TCHAR is char, and a widening conversion when
 * TCHAR is wchar_t.  Does not null terminate for use in buffered printing.
 */
static void
convert_to_tchar(TCHAR *dst, const char *src, size_t dst_sz)
{
#ifdef _UNICODE
    _snwprintf(dst, dst_sz, L"%S", src);
#else
    strncpy(dst, src, dst_sz);
#endif
}

/* Function to iterate over the options in a DYNAMORIO_OPTIONS string.
 * For the purposes of this function, we're not differentiating
 * between an option and an option argument.  We're simply looking for
 * space-separated strings while taking into account that some strings
 * can be quoted.  'ptr' should point to the current location in the
 * options string; the option is copied to 'token'
 */
static TCHAR *
get_next_token(TCHAR* ptr, TCHAR *token)
{
    /* advance to next non-space character */
    while (*ptr == L' ') {
        ptr++;
    }

    /* check for end-of-string */
    if (*ptr == L'\0') {
        token[0] = L'\0';
        return NULL;
    }

    /* for quoted options, copy until the closing quote */
    if (*ptr == L'\"') {
        *token++ = *ptr++;
        while (*ptr != L'\0') {
            *token++ = *ptr++;
            if (ptr[-1] == L'\"' && ptr[-2] != L'\\') {
                break;
            }
        }
    }
    
    /* otherwise copy until the next space character */
    else {
        while (*ptr != L' ' && *ptr != L'\0') {
            *token++ = *ptr++;
        }
    }

    *token = L'\0';
    return ptr;
}


/* Allocate a new client_opt_t */
static client_opt_t *
new_client_opt(const TCHAR *path, client_id_t id, const TCHAR *opts)
{
    size_t len;
    client_opt_t *opt = (client_opt_t *)malloc(sizeof(client_opt_t));
    if (opt == NULL) {
        return NULL;
    }

    opt->id = id;

    len = MIN(MAXIMUM_PATH-1, _tcslen(path));
    opt->path = malloc((len+1) * sizeof(opt->path[0]));
    _tcsncpy(opt->path, path, len);
    opt->path[len] = L'\0';

    len = MIN(DR_MAX_OPTIONS_LENGTH-1, _tcslen(opts));
    opt->opts = malloc((len+1) * sizeof(opt->opts[0]));
    _tcsncpy(opt->opts, opts, len);
    opt->opts[len] = L'\0';

    return opt;
}

/* Free a client_opt_t */
static void
free_client_opt(client_opt_t *opt)
{
    if (opt == NULL) {
        return;
    }
    if (opt->path != NULL) {
        free(opt->path);
    }
    if (opt->opts != NULL) {
        free(opt->opts);
    }
}


/* Add another client to an opt_info_t struct */
static dr_config_status_t
add_client_lib(opt_info_t *opt_info, client_id_t id, size_t pri,
               const TCHAR *path, const TCHAR *opts)
{
    size_t i;
    
    if (opt_info->num_clients >= MAX_CLIENT_LIBS) {
        return DR_FAILURE;
    }

    if (pri > opt_info->num_clients) {
        return DR_ID_INVALID;
    }

    /* shift existing entries to make space for the new client info */
    for (i=opt_info->num_clients; i>pri; i--) {
        opt_info->client_opts[i] = opt_info->client_opts[i-1];
    }

    opt_info->client_opts[pri] = new_client_opt(path, id, opts);
    opt_info->num_clients++;

    return DR_SUCCESS;
}


static dr_config_status_t
remove_client_lib(opt_info_t *opt_info, client_id_t id)
{
    size_t i, j;
    for (i=0; i<opt_info->num_clients; i++) {
        if (opt_info->client_opts[i]->id == id) {
            free_client_opt(opt_info->client_opts[i]);

            /* shift remaining entries down */
            for (j=i; j<opt_info->num_clients-1; j++) {
                opt_info->client_opts[j] = opt_info->client_opts[j+1];
            }

            opt_info->num_clients--;
            return DR_SUCCESS;
        }
    }

    return DR_ID_INVALID;
}


/* Add an 'extra' option (non-client related option) to an opt_info_t struct */
static dr_config_status_t
add_extra_option(opt_info_t *opt_info, const TCHAR *opt)
{
    if (opt != NULL && opt[0] != L'\0') {
        size_t idx, len;
        idx = opt_info->num_extra_opts;
        if (idx >= MAX_NUM_OPTIONS) {
            return DR_FAILURE;
        }
        
        len = MIN(DR_MAX_OPTIONS_LENGTH-1, _tcslen(opt));
        opt_info->extra_opts[idx] = malloc
            ((len+1) * sizeof(opt_info->extra_opts[idx][0]));
        _tcsncpy(opt_info->extra_opts[idx], opt, len);
        opt_info->extra_opts[idx][len] = L'\0';

        opt_info->num_extra_opts++;
    }

    return DR_SUCCESS;
}

static dr_config_status_t
add_extra_option_char(opt_info_t *opt_info, const char *opt)
{
    if (opt != NULL && opt[0] != '\0') {
        TCHAR wbuf[DR_MAX_OPTIONS_LENGTH];
        convert_to_tchar(wbuf, opt, DR_MAX_OPTIONS_LENGTH);
        NULL_TERMINATE_BUFFER(wbuf);
        return add_extra_option(opt_info, wbuf);
    }

    return DR_SUCCESS;
}


/* Free allocated memory in an opt_info_t */
static void
free_opt_info(opt_info_t *opt_info)
{
    size_t i;
    for (i=0; i<opt_info->num_clients; i++) {
        free_client_opt(opt_info->client_opts[i]);
    }
    for (i=0; i<opt_info->num_extra_opts; i++) {
        free(opt_info->extra_opts[i]);
    }
}

/***************************************************************************
 * i#85/PR 212034 and i#265/PR 486139: use config files
 *
 * The API uses char* here but be careful b/c this file is build w/ UNICODE
 * so we use *A versions of Windows API routines.
 * Also note that I left many types as wchar_t for less change in handling
 * PARAMS_IN_REGISTRY and config files: eventually should convert all
 * to char.
 */

#ifdef PARAMS_IN_REGISTRY
# define IF_REG_ELSE(x, y) x
# define PARAM_STR(name) L_IF_WIN(name)
#else
# define PARAM_STR(name) name
# define IF_REG_ELSE(x, y) y
#endif

#ifndef PARAMS_IN_REGISTRY
# define LOCAL_CONFIG_ENV "USERPROFILE"
# ifdef WINDOWS
#  define LOCAL_CONFIG_SUBDIR "dynamorio"
# else
#  define LOCAL_CONFIG_SUBDIR ".dynamorio"
# endif
# define GLOBAL_CONFIG_SUBDIR "config"
# define CFG_SFX_64 "config64"
# define CFG_SFX_32 "config32"
# ifdef X64
#  define CFG_SFX CFG_SFX_64
# else
#  define CFG_SFX CFG_SFX_32
# endif

static const char *
get_config_sfx(dr_platform_t dr_platform)
{
    if (dr_platform == DR_PLATFORM_DEFAULT)
        return CFG_SFX;
    else if (dr_platform == DR_PLATFORM_32BIT)
        return CFG_SFX_32;
    else if (dr_platform == DR_PLATFORM_64BIT)
        return CFG_SFX_32;
    else
        DO_ASSERT(false);
    return "";
}

static bool
get_config_dir(bool global, char *fname, size_t fname_len)
{
    char dir[MAXIMUM_PATH];
    const char *subdir;
    if (global) {
#ifdef WINDOWS
        _snprintf(dir, BUFFER_SIZE_ELEMENTS(dir), TSTR_FMT, get_dynamorio_home());
        subdir = GLOBAL_CONFIG_SUBDIR;
#else
        /* XXX: Port utils.c. */
        return false;
#endif
    } else {
#ifdef WINDOWS
        int len = GetEnvironmentVariableA(LOCAL_CONFIG_ENV, dir,
                                          BUFFER_SIZE_ELEMENTS(dir));
        if (len <= 0)
            return false;
#else
        char *home = getenv("HOME");
        strncpy(dir, home, BUFFER_SIZE_ELEMENTS(dir));
        NULL_TERMINATE_BUFFER(dir);
#endif
        subdir = LOCAL_CONFIG_SUBDIR;
    }
    _snprintf(fname, fname_len, "%s/%s", dir, subdir);
    fname[fname_len - 1] = '\0';
    return true;
}

/* No support yet here to create some types of files the core supports:
 * - system config dir by reading home reg key: plan is to
 *   add a global setting to use that, so no change to params in API
 * - default0.config
 */
static bool
get_config_file_name(const char *process_name,
                     process_id_t pid,
                     bool global,
                     dr_platform_t dr_platform,
                     char *fname,
                     size_t fname_len)
{
    size_t dir_len;
    if (!get_config_dir(global, fname, fname_len)) {
        DO_ASSERT(false && "get_config_dir failed");
        return false;
    }
#ifdef WINDOWS
    /* make sure subdir exists*/
    if (!CreateDirectoryA(fname, NULL) &&
        GetLastError() != ERROR_ALREADY_EXISTS) {
        DO_ASSERT(false && "failed to create subdir");
        return false;
    }
#else
    {
        struct stat st;
        mkdir(fname, 0770);
        if (stat(fname, &st) != 0 || !S_ISDIR(st.st_mode)) {
            DO_ASSERT(false && "failed to create subdir");
            return false;
        }
    }
#endif
    dir_len = strlen(fname);
    if (pid > 0) {
        /* <root>/appname.<pid>.1config */
        _snprintf(fname + dir_len, fname_len - dir_len, "/%s.%d.1%s", 
                  process_name, pid, get_config_sfx(dr_platform));
    } else {
        /* <root>/appname.config */
        _snprintf(fname + dir_len, fname_len - dir_len, "/%s.%s",
                  process_name, get_config_sfx(dr_platform));
    }
    fname[fname_len - 1] = '\0';
    return true;
}

static FILE *
open_config_file(const char *process_name,
                 process_id_t pid,
                 bool global,
                 dr_platform_t dr_platform,
                 bool read, bool write, bool overwrite)
{
    TCHAR wfname[MAXIMUM_PATH];
    char fname[MAXIMUM_PATH];
    char mode[3];
    int i = 0;
    DO_ASSERT(read || write);
    if (read)
        mode[i++] = 'r';
    if (write)
        mode[i++] = 'w';
    mode[i] = '\0';
    if (!get_config_file_name(process_name, pid, global, dr_platform,
                             fname, BUFFER_SIZE_ELEMENTS(fname))) {
        DO_ASSERT(false && "get_config_file_name failed");
        return NULL;
    }

    /* XXX: Checking existance and then opening is racy. */
    convert_to_tchar(wfname, fname, BUFFER_SIZE_ELEMENTS(wfname));
    NULL_TERMINATE_BUFFER(wfname);
    if (!overwrite && file_exists(wfname))
        return NULL;
    return fopen(fname, mode);
}

/* Copies the value for var, converted to a TCHAR, into val.  If elide is true,
 * also overwrites var and its value in the file with all lines subsequent,
 * allowing for a simple append to change the value (the file must have been
 * opened with both read and write access).
 */
static bool
read_config_ex(FILE *f, const char *var, TCHAR *val, size_t val_len,
               bool elide)
{
    bool found = false;
    /* could use _open_osfhandle() to get fd and _fdopen() to get FILE* and then
     * use fgets: but then have to close with fclose on FILE and thus messy w/
     * callers holding HANDLE instead.  already have code for line reading so
     * sticking w/ HANDLE.  FIXME: share code w/ core/config.c
     */
#   define BUFSIZE (MAX_CONFIG_VALUE+128)
    char buf[BUFSIZE];
    char *line, *newline = NULL;
    ssize_t bufread = 0, bufwant;
    ssize_t len;
    /* each time we start from beginning: we assume a small file */
    if (f == NULL)
        return false;
    if (fseek(f, 0, SEEK_SET))
        return false;
    while (true) {
        /* break file into lines */
        if (newline == NULL) {
            bufwant = BUFSIZE-1;
            bufread = fread(buf, 1, bufwant, f);
            DO_ASSERT(bufread >= 0 && bufread <= bufwant);
            if (bufread <= 0)
                break;
            buf[bufread] = '\0';
            newline = strchr(buf, '\n');
            line = buf;
        } else {
            line = newline + 1;
            newline = strchr(line, '\n');
            if (newline == NULL) {
                /* shift 1st part of line to start of buf, then read in rest */
                /* the memory for the processed part can be reused  */
                bufwant = line - buf;
                DO_ASSERT(bufwant <= bufread);
                len = bufread - bufwant; /* what is left from last time */
                /* using memmove since strings can overlap */
                if (len > 0)
                    memmove(buf, line, len);
                bufread = fread(buf+len, 1, bufwant, f);
                DO_ASSERT(bufread >= 0 && bufread <= bufwant);
                if (bufread <= 0)
                    break;
                bufread += len; /* bufread is total in buf */
                buf[bufread] = '\0';
                newline = strchr(buf, '\n');
                line = buf;
            }
        }
        /* buffer is big enough to hold at least one line */
        DO_ASSERT(newline != NULL);
        *newline = '\0';
        /* handle \r\n line endings */
        if (newline > line && *(newline-1) == '\r')
            *(newline-1) = '\0';
        /* now we have one line */
        /* we support blank lines and comments */
        if (line[0] == '\0' || line[0] == '#')
            continue;
        if (strstr(line, var) == line) {
            char *eq = strchr(line, '=');
            if (eq != NULL &&
                /* we don't have any vars that are prefixes of others so we
                 * can do a hard match on whole var.
                 * for parsing simplicity we don't allow whitespace before '='.
                 */
                eq == line + strlen(var)) {
                /* we can only copy this much at once.  alternative is to
                 * create a new file and rename it at the end.
                 */
                bufwant = (newline + 1 - line);
                if (val != NULL) {
                    convert_to_tchar(val, eq + 1, val_len);
                }
                found = true;
                if (!elide)
                    break; /* done */
                /* now start shifting.  could be more efficient by
                 * writing what's already in buffer, writing 2x the 1st
                 * time (though check buf size), but keeping simple for now
                 */
                /* assuming file never going to need 64-bit => low dword never -1
                 * so return value of INVALID_SET_FILE_POINTER always means error
                 */
                if (fseek(f, (ssize_t) -((buf+bufread) - newline), SEEK_CUR) != 0) {
                    DO_ASSERT(false);
                    return false;
                }
                while (true) {
                    bool done = false;
                    bufread = fread(buf, 1, bufwant, f);
                    DO_ASSERT(bufread >= 0 && bufread <= bufwant);
                    if (bufread <= 0)
                        done = true;
                    if (fseek(f, (ssize_t) -(bufwant+bufread), SEEK_CUR) != 0) {
                        DO_ASSERT(false);
                        return false;
                    }
                    if (done) {
                        /* pointer goes back to end somehow in write_config_param */
#ifdef WINDOWS
                        /* XXX: Can't find a way to truncate the file at the
                         * current position using the FILE API.
                         */
                        SetEndOfFile((HANDLE)_get_osfhandle(_fileno(f)));
#else
                        off_t pos = lseek(fileno(f), 0, SEEK_CUR);
                        DO_ASSERT(pos != -1);
                        ftruncate(fileno(f), (off_t)pos);
#endif
                        break;
                    }
                    len = fwrite(buf, 1, bufread, f);
                    DO_ASSERT(len == bufread);
                    if (fseek(f, bufwant, SEEK_CUR) != 0) {
                        DO_ASSERT(false);
                        return false;
                    }
                }
            }
        }
    }
    return found;
}

/* for simplest coexistence with PARAMS_IN_REGISTRY taking in TCHAR and
 * converting to char.  not very efficient though.
 */
static dr_config_status_t
write_config_param(FILE *f, const char *var, const TCHAR *val)
{
    size_t written;
    int len;
    char buf[MAX_CONFIG_VALUE];
    DO_ASSERT(f != NULL);
    len = _snprintf(buf, BUFFER_SIZE_ELEMENTS(buf), "%s="TSTR_FMT"\n", var, val);
    /* don't remove the newline: better to truncate options than to have none (i#547) */
    buf[BUFFER_SIZE_ELEMENTS(buf) - 2] = '\n';
    buf[BUFFER_SIZE_ELEMENTS(buf) - 1] = '\0';
    written = fwrite(buf, 1, strlen(buf), f);
    DO_ASSERT(written == strlen(buf));
    if (len < 0)
        return DR_CONFIG_STRING_TOO_LONG;
    if (written != strlen(buf))
        return DR_CONFIG_FILE_WRITE_FAILED;
    return DR_SUCCESS;
}

static bool
read_config_param(FILE *f, const char *var, TCHAR *val, size_t val_len)
{
    return read_config_ex(f, var, val, val_len, false);
}

#else /* !PARAMS_IN_REGISTRY */

static dr_config_status_t
write_config_param(ConfigGroup *policy, const TCHAR *var, const TCHAR *val)
{
    set_config_group_parameter(policy, var, val);
    return DR_SUCCESS;
}

static bool
read_config_param(FILE *f, const char *var, const TCHAR *val, size_t val_len)
{
    TCHAR *ptr = get_config_group_parameter(proc_policy, L_DYNAMORIO_VAR_OPTIONS);
    if (ptr == NULL)
        return false;
    _sntprintf(val, val_len, _TEXT("%s"), ptr);
    return true;
}

#endif /* PARAMS_IN_REGISTRY */

/***************************************************************************/

/* Read a DYNAMORIO_OPTIONS string from 'wbuf' and populate an opt_info_t struct */
static dr_config_status_t
read_options(opt_info_t *opt_info, IF_REG_ELSE(ConfigGroup *proc_policy, FILE *f))
{
    TCHAR buf[MAX_CONFIG_VALUE];
    TCHAR *ptr, token[DR_MAX_OPTIONS_LENGTH], tmp[DR_MAX_OPTIONS_LENGTH];
    opt_info_t null_opt_info = {0,};
    size_t len;
    
    *opt_info = null_opt_info;

    if (!read_config_param(IF_REG_ELSE(proc_policy, f), PARAM_STR(DYNAMORIO_VAR_OPTIONS),
                           buf, BUFFER_SIZE_ELEMENTS(buf)))
        return DR_FAILURE;
    ptr = buf;

    /* We'll be safe and not trust that get_config_group_parameter
     * returns a nice NULL-terminated string with no more than
     * DR_MAX_OPTIONS_LENGTH characters.  Making sure we have such a
     * string makes get_next_token() simpler.  A more efficient
     * approach would be to keep track of a string length and pass
     * that to get_next_token().
     */
    len = MIN(DR_MAX_OPTIONS_LENGTH-1, _tcslen(ptr));
    _tcsncpy(tmp, ptr, len);
    tmp[len] = L'\0';

    opt_info->mode = DR_MODE_NONE;

    ptr = tmp;
    while (ptr != NULL) {
        ptr = get_next_token(ptr, token);
        
        /*
         * look for the mode 
         */
        if (_tcscmp(token, _TEXT("-code_api")) == 0) {
            if (opt_info->mode != DR_MODE_NONE) {
                goto error;
            }
            opt_info->mode = DR_MODE_CODE_MANIPULATION;
        }
#ifdef MF_API
        else if (_tcscmp(token, _TEXT("-security_api")) == 0) {
            if (opt_info->mode != DR_MODE_NONE) {
                goto error;
            }
            opt_info->mode = DR_MODE_MEMORY_FIREWALL;
        }
#endif
        else if (_tcscmp(token, _TEXT("-probe_api")) == 0) {
#ifdef PROBE_API
            /* nothing; we assign the mode when we see -code_api */
#else
            /* we shouldn't see -probe_api */
            goto error;
#endif
        }
#ifdef PROBE_API
        else if (_tcscmp(token, _TEXT("-hotp_only")) == 0) {
            if (opt_info->mode != DR_MODE_NONE) {
                goto error;
            }
            opt_info->mode = DR_MODE_PROBE;
        }
#endif
        
        /* 
         * look for client options
         */
        else if (_tcscmp(token, _TEXT("-client_lib")) == 0) {
            TCHAR *path_str, *id_str, *opt_str;
            client_id_t id;

            ptr = get_next_token(ptr, token);
            if (ptr == NULL) {
                goto error;
            }

            /* handle enclosing quotes */
            path_str = token;
            if (path_str[0] == L'\"') {
                size_t last;

                path_str++;
                last = _tcslen(path_str)-1;
                if (path_str[last] != L'\"') {
                    goto error;
                }
                path_str[last] = L'\0';
            }

            /* -client_lib options should have the form path;ID;options.
             * Client priority is left-to-right.
             */
            id_str = _tcsstr(path_str, _TEXT(";"));
            if (id_str == NULL) {
                goto error;
            }

            *id_str = L'\0';
            id_str++;

            opt_str = _tcsstr(id_str, _TEXT(";"));
            if (opt_str == NULL) {
                goto error;
            }

            *opt_str = L'\0';
            opt_str++;

            /* client IDs are in hex */
            id = _tcstoul(id_str, NULL, 16);

            /* add the client info to our opt_info structure */
            if (add_client_lib(opt_info, id, opt_info->num_clients, 
                               path_str, opt_str) != DR_SUCCESS) {
                goto error;
            }
        }

        /*
         * Any remaining options are not related to clients.  Put all
         * these options (and their arguments) in one array.
         */
        else {
            if (add_extra_option(opt_info, token) != DR_SUCCESS) {
                goto error;
            }
        }
    }

    return DR_SUCCESS;

 error:
    free_opt_info(opt_info);
    *opt_info = null_opt_info;
    return DR_FAILURE;
}


/* Write the options stored in an opt_info_t to 'wbuf' in the form expected
 * by the DYNAMORIO_OPTIONS registry entry.
 */
static void
write_options(opt_info_t *opt_info, TCHAR *wbuf)
{
    size_t i;
    char *mode_str = "";
    ssize_t len;
    ssize_t sofar = 0;
    ssize_t bufsz = DR_MAX_OPTIONS_LENGTH;

    /* NOTE - mode_str must come first since we want to give
     * client-supplied options the chance to override (for
     * ex. -stack_size which -code_api sets, see PR 247436
     */
    switch (opt_info->mode) {
#ifdef MF_API
        case DR_MODE_MEMORY_FIREWALL:
            mode_str = "-security_api";
            break;
#endif
        case DR_MODE_CODE_MANIPULATION:
#ifdef PROBE_API
            mode_str = "-code_api -probe_api";
#else
            mode_str = "-code_api";
#endif
            break;
#ifdef PROBE_API
        case DR_MODE_PROBE:
            mode_str = "-probe_api -hotp_only";
            break;
#endif
        case DR_MODE_DO_NOT_RUN:
            /* this is a mode b/c can't add dr_register_process param w/o breaking
             * backward compat, so just ignore in terms of options, user has to
             * re-reg anyway to re-enable and can specify mode then.
             */
            mode_str = "";
            break;
        default:
#ifndef CLIENT_INTERFACE
            /* no API's so no added options */
            mode_str = "";
            break;
#endif
            DO_ASSERT(false);
    }

    len = _sntprintf(wbuf + sofar, bufsz - sofar, _TEXT("%s"), mode_str);
    if (len >= 0 && len <= bufsz - sofar)
        sofar += len;

    /* extra options */
    for (i=0; i<opt_info->num_extra_opts; i++) {
        /* FIXME: Note that we're blindly allowing any options
         * provided so we can allow users to specify "undocumented"
         * options.  Maybe we should be checking that the options are
         * actually valid?
         */
        len = _sntprintf(wbuf + sofar, bufsz - sofar, _TEXT(" %s"),
                         opt_info->extra_opts[i]);
        if (len >= 0 && len <= bufsz - sofar)
            sofar += len;
    }

    /* client lib options */
    for (i=0; i<opt_info->num_clients; i++) {
        client_opt_t *client_opts = opt_info->client_opts[i];
        len = _sntprintf(wbuf + sofar, bufsz - sofar,
                         _TEXT(" -client_lib \"%s;%x;%s\""),
                         client_opts->path, client_opts->id, client_opts->opts);
        if (len >= 0 && len <= bufsz - sofar)
            sofar += len;
    }

    wbuf[DR_MAX_OPTIONS_LENGTH-1] = L'\0';
}

#ifdef PARAMS_IN_REGISTRY
/* caller must call free_config_group() on the returned ConfigGroup */
static ConfigGroup *
get_policy(dr_platform_t dr_platform)
{
    ConfigGroup *policy;

    /* PR 244206: set the registry view before any registry access.
     * If we are ever part of a persistent agent maybe we should
     * restore to the default platform at the end of this routine?
     */
    set_dr_platform(dr_platform);

    if (read_config_group(&policy, L_PRODUCT_NAME, TRUE) != ERROR_SUCCESS) {
        return NULL;
    }

    return policy;
}

/* As a sub policy only the parent policy (from get_policy()) need be freed */
static ConfigGroup * 
get_proc_policy(ConfigGroup *policy, const char *process_name)
{
    ConfigGroup *res = NULL;
    if (policy != NULL) {
        TCHAR wbuf[MAXIMUM_PATH];
        convert_to_tchar(wbuf, process_name, MAXIMUM_PATH);
        NULL_TERMINATE_BUFFER(wbuf);
        res = get_child(wbuf, policy);
    }
    return res;
}                
#endif

static bool
platform_is_64bit(dr_platform_t platform)
{
    return (platform == DR_PLATFORM_64BIT
            IF_X64(|| platform == DR_PLATFORM_DEFAULT));
}

#ifdef WINDOWS
static void
get_syswide_path(TCHAR *wbuf,
                 const char *dr_root_dir)
{
    TCHAR path[MAXIMUM_PATH];
    int len;
    if (!platform_is_64bit(get_dr_platform()))
        _sntprintf(path, MAXIMUM_PATH, _TEXT("%S")PREINJECT32_DLL, dr_root_dir);
    else
        _sntprintf(path, MAXIMUM_PATH, _TEXT("%S")PREINJECT64_DLL, dr_root_dir);
    path[MAXIMUM_PATH - 1] = '\0';
    /* spaces are separator in AppInit so use short path */
    len = GetShortPathName(path, wbuf, MAXIMUM_PATH);
    DO_ASSERT(len > 0);
    wbuf[MAXIMUM_PATH - 1] = '\0';
}

EXPORT
dr_config_status_t
dr_register_syswide(dr_platform_t dr_platform,
                    const char *dr_root_dir)
{
    TCHAR wbuf[MAXIMUM_PATH];
    set_dr_platform(dr_platform);
    /* Set the appinit key */
    get_syswide_path(wbuf, dr_root_dir);
    /* Always overwrite, in case we have an older drpreinject version in there */
    if (set_custom_autoinjection(wbuf, APPINIT_OVERWRITE) != ERROR_SUCCESS ||
        (is_vista() && set_loadappinit() != ERROR_SUCCESS)) {
        return DR_FAILURE;
    }
    return DR_SUCCESS;
}

EXPORT
dr_config_status_t
dr_unregister_syswide(dr_platform_t dr_platform,
                      const char *dr_root_dir)
{
    TCHAR wbuf[MAXIMUM_PATH];
    set_dr_platform(dr_platform);
    /* Set the appinit key */
    get_syswide_path(wbuf, dr_root_dir);
    if (unset_custom_autoinjection(wbuf, APPINIT_OVERWRITE) != ERROR_SUCCESS)
        return DR_FAILURE;
    /* We leave Vista loadappinit on */
    return DR_SUCCESS;
}

EXPORT
bool
dr_syswide_is_on(dr_platform_t dr_platform,
                 const char *dr_root_dir)
{
    TCHAR wbuf[MAXIMUM_PATH];
    set_dr_platform(dr_platform);
    /* Set the appinit key */
    get_syswide_path(wbuf, dr_root_dir);
    return CAST_TO_bool(is_custom_autoinjection_set(wbuf));
}
#endif

EXPORT
dr_config_status_t
dr_register_process(const char *process_name,
                    process_id_t pid,
                    bool global,
                    const char *dr_root_dir,
                    dr_operation_mode_t dr_mode,
                    bool debug,
                    dr_platform_t dr_platform,
                    const char *dr_options)
{
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy, *proc_policy;
#else
    FILE *f;
#endif
    TCHAR wbuf[MAX(MAXIMUM_PATH,DR_MAX_OPTIONS_LENGTH)];
    IF_WINDOWS(DWORD platform;)
    opt_info_t opt_info = {0,};
    dr_config_status_t status;

#ifdef PARAMS_IN_REGISTRY
    /* PR 244206: set the registry view before any registry access.
     * If we are ever part of a persistent agent maybe we should
     * restore to the default platform at the end of this routine?
     */
    set_dr_platform(dr_platform);

    /* create top-level Determina/SecureCore key */
    if (create_root_key() != ERROR_SUCCESS) {
        return DR_FAILURE;
    }
    if (read_config_group(&policy, L_PRODUCT_NAME, TRUE) != ERROR_SUCCESS) {
        return DR_FAILURE;
    }

    /* create process key */
    convert_to_tchar(wbuf, process_name, MAXIMUM_PATH);
    NULL_TERMINATE_BUFFER(wbuf);
    proc_policy = get_child(wbuf, policy);
    if (proc_policy == NULL) {
        proc_policy = new_config_group(wbuf);
        add_config_group(policy, proc_policy);
    }
    else {
        return DR_PROC_REG_EXISTS;
    }
#else
    f = open_config_file(process_name, pid, global, dr_platform,
                         false/*!read*/, true/*write*/,
                         pid != 0/*overwrite for pid-specific*/);
    if (f == NULL) {
# ifdef WINDOWS
        int err = GetLastError();
        if (err == ERROR_ALREADY_EXISTS)
            return DR_PROC_REG_EXISTS;
        else
# endif
            return DR_FAILURE;
    }
#endif

    /* set the rununder string */
    _sntprintf(wbuf, MAXIMUM_PATH, (dr_mode == DR_MODE_DO_NOT_RUN) ? _TEXT("0") : _TEXT("1"));
    NULL_TERMINATE_BUFFER(wbuf);
    status = write_config_param(IF_REG_ELSE(proc_policy, f),
                                PARAM_STR(DYNAMORIO_VAR_RUNUNDER), wbuf);
    DO_ASSERT(status == DR_SUCCESS);
    if (status != DR_SUCCESS)
        return status;

    /* set the autoinject string (i.e., path to dynamorio.dll */
    if (debug) {
        if (!platform_is_64bit(get_dr_platform()))
            _sntprintf(wbuf, MAXIMUM_PATH, _TEXT(TSTR_FMT)DEBUG32_DLL, dr_root_dir);
        else
            _sntprintf(wbuf, MAXIMUM_PATH, _TEXT(TSTR_FMT)DEBUG64_DLL, dr_root_dir);
    } else {
        if (!platform_is_64bit(get_dr_platform()))
            _sntprintf(wbuf, MAXIMUM_PATH, _TEXT(TSTR_FMT)RELEASE32_DLL, dr_root_dir);
        else
            _sntprintf(wbuf, MAXIMUM_PATH, _TEXT(TSTR_FMT)RELEASE64_DLL, dr_root_dir);
    }
    NULL_TERMINATE_BUFFER(wbuf);
    status = write_config_param(IF_REG_ELSE(proc_policy, f),
                                PARAM_STR(DYNAMORIO_VAR_AUTOINJECT), wbuf);
    DO_ASSERT(status == DR_SUCCESS);
    if (status != DR_SUCCESS)
        return status;

    /* set the logdir string */
    /* XXX i#886: should we expose this in the dr_register_process() params (and
     * thus dr_process_is_registered() and dr_registered_process_iterator_next())?
     * We now have a -logdir runtime option so we don't need to expose it for full
     * functionality anymore but it would serve to reduce the length of option
     * strings to have more control over the default.  Linux dr{config,run} does
     * allow such control today.
     */
    _sntprintf(wbuf, MAXIMUM_PATH, _TEXT(TSTR_FMT)LOG_SUBDIR, dr_root_dir);
    NULL_TERMINATE_BUFFER(wbuf);
    status = write_config_param(IF_REG_ELSE(proc_policy, f),
                                PARAM_STR(DYNAMORIO_VAR_LOGDIR), wbuf);
    DO_ASSERT(status == DR_SUCCESS);
    if (status != DR_SUCCESS)
        return status;

    /* set the options string last for faster updating w/ config files */
    opt_info.mode = dr_mode;
    /* XXX: Expose LD_PRELOAD vs. early injection. */
    add_extra_option_char(&opt_info, "-early_inject");
    add_extra_option_char(&opt_info, dr_options);
    write_options(&opt_info, wbuf);
    status = write_config_param(IF_REG_ELSE(proc_policy, f),
                                PARAM_STR(DYNAMORIO_VAR_OPTIONS), wbuf);
    free_opt_info(&opt_info);
    DO_ASSERT(status == DR_SUCCESS);
    if (status != DR_SUCCESS)
        return status;

#ifdef PARAMS_IN_REGISTRY
    /* write the registry */
    if (write_config_group(policy) != ERROR_SUCCESS) {
        DO_ASSERT(false);
        return DR_FAILURE;
    }
#else
    fclose(f);
#endif

    /* If on win2k, copy drearlyhelper?.dll to system32 
     * FIXME: this requires admin privs!  oh well: only issue is early inject
     * on win2k...
     */
#ifdef WINDOWS
    if (get_platform(&platform) == ERROR_SUCCESS && platform == PLATFORM_WIN_2000) {
        _sntprintf(wbuf, MAXIMUM_PATH, _TEXT(TSTR_FMT)LIB32_SUBDIR, dr_root_dir);
        NULL_TERMINATE_BUFFER(wbuf);
        copy_earlyhelper_dlls(wbuf);
    }
#endif

    return DR_SUCCESS;
}


EXPORT
dr_config_status_t
dr_unregister_process(const char *process_name,
                      process_id_t pid,
                      bool global,
                      dr_platform_t dr_platform)
{
#ifndef PARAMS_IN_REGISTRY
    char fname[MAXIMUM_PATH];
    if (get_config_file_name(process_name, pid, global, dr_platform,
                             fname, BUFFER_SIZE_ELEMENTS(fname))) {
        if (remove(fname) == 0)
            return DR_SUCCESS;
    }
    return DR_FAILURE;
#else
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
    TCHAR wbuf[MAXIMUM_PATH];
    dr_config_status_t status = DR_SUCCESS;

    if (proc_policy == NULL) {
        status = DR_PROC_REG_INVALID;
        goto exit;
    }

    /* remove it */
    convert_to_tchar(wbuf, process_name, MAXIMUM_PATH);
    NULL_TERMINATE_BUFFER(wbuf);
    remove_child(wbuf, policy);
    policy->should_clear = TRUE;
    
    /* write the registry */
    if (write_config_group(policy) != ERROR_SUCCESS) {
        status = DR_FAILURE;
        goto exit;
    }

    /* FIXME PR 232738: we should remove the drdearlyhelp?.dlls and preinject
     * from system32, and remove the base reg keys, if dr_unregister_process()
     * removes the last registered process.
     */

 exit:
    if (policy != NULL)
        free_config_group(policy);
    return status;
#endif
}

/* For !PARAMS_IN_REGISTRY, process_name is NOT filled in! */
static void
read_process_policy(IF_REG_ELSE(ConfigGroup *proc_policy, FILE *f),
                    char *process_name /* OUT */,
                    char *dr_root_dir /* OUT */,
                    dr_operation_mode_t *dr_mode /* OUT */,
                    bool *debug /* OUT */,
                    char *dr_options /* OUT */)
{
    TCHAR autoinject[MAX_CONFIG_VALUE];
    opt_info_t opt_info;
    
    if (dr_mode != NULL)
        *dr_mode = DR_MODE_NONE;
    if (dr_root_dir != NULL)
        *dr_root_dir = '\0';
    if (dr_options != NULL)
        *dr_options = '\0';

#ifdef PARAMS_IN_REGISTRY
    if (process_name != NULL)
        *process_name = '\0';
    if (process_name != NULL && proc_policy->name != NULL) {
        SIZE_T len = MIN(_tcslen(proc_policy->name), MAXIMUM_PATH-1);
        _snprintf(process_name, len, TSTR_FMT, proc_policy->name);
        process_name[len] = '\0';
    }
#else
    /* up to caller to fill in! */
#endif

    if (dr_root_dir != NULL && 
        read_config_param(IF_REG_ELSE(proc_policy, f),
                          PARAM_STR(DYNAMORIO_VAR_AUTOINJECT),
                          autoinject, BUFFER_SIZE_ELEMENTS(autoinject))) {
        TCHAR *vers = _tcsstr(autoinject, RELEASE32_DLL);
        if (vers == NULL) {
            vers = _tcsstr(autoinject, DEBUG32_DLL);
        }
        if (vers == NULL) {
            vers = _tcsstr(autoinject, RELEASE64_DLL);
        }
        if (vers == NULL) {
            vers = _tcsstr(autoinject, DEBUG64_DLL);
        }
        if (vers != NULL) {
            size_t len = MIN(MAXIMUM_PATH-1, vers - autoinject);
            _snprintf(dr_root_dir, len, TSTR_FMT, autoinject);
            dr_root_dir[len] = '\0';
        }
        else {
            dr_root_dir[0] = '\0';
        }
    }
    
    if (read_options(&opt_info, IF_REG_ELSE(proc_policy, f)) != DR_SUCCESS) {
        /* note: read_options() frees any memory it allocates if it fails */
        return;
    }

    if (dr_mode != NULL) {
        *dr_mode = opt_info.mode;
        if (read_config_param(IF_REG_ELSE(proc_policy, f),
                              PARAM_STR(DYNAMORIO_VAR_RUNUNDER),
                              autoinject, BUFFER_SIZE_ELEMENTS(autoinject))) {
            if (_tcscmp(autoinject, _TEXT("0")) == 0)
                *dr_mode = DR_MODE_DO_NOT_RUN;
        }
    }

    if (debug != NULL) {
        if (_tcsstr(autoinject, DEBUG32_DLL) != NULL ||
            _tcsstr(autoinject, DEBUG64_DLL) != NULL) {
            *debug = true;
        }
        else {
            *debug = false;
        }
    }
         
    if (dr_options != NULL) {
        uint i;
        size_t len_remain = DR_MAX_OPTIONS_LENGTH-1, cur_off = 0;
        dr_options[0] = '\0';
        for (i = 0; i < opt_info.num_extra_opts; i++) {
            size_t len;
            if (i > 0 && len_remain > 0) {
                len_remain--;
                dr_options[cur_off++] = ' ';
            }
            len = MIN(len_remain, _tcslen(opt_info.extra_opts[i]));
            _snprintf(dr_options+cur_off, len, TSTR_FMT, opt_info.extra_opts[i]);
            cur_off += len;
            len_remain -= len;
            dr_options[cur_off] = '\0';
        }
    }

    free_opt_info(&opt_info);
}

#ifdef WINDOWS

struct _dr_registered_process_iterator_t {
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy;
    ConfigGroup *cur;
#else
    bool has_next;
    HANDLE find_handle;
    /* because UNICODE is defined we have to use the wide version of
     * FindFirstFile and convert back and forth
     */
    WIN32_FIND_DATA find_data;
    /* FindFirstFile only fills in the basename */
    char dir[MAXIMUM_PATH];
    char fname[MAXIMUM_PATH];
#endif
};

EXPORT
dr_registered_process_iterator_t *
dr_registered_process_iterator_start(dr_platform_t dr_platform,
                                     bool global)
{
    dr_registered_process_iterator_t *iter = (dr_registered_process_iterator_t *)
        malloc(sizeof(dr_registered_process_iterator_t));
#ifdef PARAMS_IN_REGISTRY
    iter->policy = get_policy(dr_platform);
    if (iter->policy != NULL)
        iter->cur = iter->policy->children;
    else
        iter->cur = NULL;
#else
    if (!get_config_dir(global, iter->dir, BUFFER_SIZE_ELEMENTS(iter->dir))) {
        iter->has_next = false;
        return iter;
    }
    _sntprintf(iter->fname, BUFFER_SIZE_ELEMENTS(iter->fname),
               _TEXT("%S/*.%S"), iter->dir, get_config_sfx(dr_platform));
    NULL_TERMINATE_BUFFER(iter->fname);
    iter->find_handle = FindFirstFile(iter->fname, &iter->find_data);
    iter->has_next = (iter->find_handle != INVALID_HANDLE_VALUE);
#endif
    return iter;
}

EXPORT
bool
dr_registered_process_iterator_hasnext(dr_registered_process_iterator_t *iter)
{
#ifdef PARAMS_IN_REGISTRY
    return iter->policy != NULL && iter->cur != NULL;
#else
    return iter->has_next;
#endif
}

EXPORT
bool
dr_registered_process_iterator_next(dr_registered_process_iterator_t *iter,
                                    char *process_name /* OUT */,
                                    char *dr_root_dir /* OUT */,
                                    dr_operation_mode_t *dr_mode /* OUT */,
                                    bool *debug /* OUT */,
                                    char *dr_options /* OUT */)
{
#ifdef PARAMS_IN_REGISTRY
    read_process_policy(iter->cur, process_name, dr_root_dir, dr_mode, debug, dr_options);
    iter->cur = iter->cur->next;
    return true;
#else
    bool ok = true;
    FILE *f;
    _sntprintf(iter->fname, BUFFER_SIZE_ELEMENTS(iter->fname),
               _TEXT("%S/%s"), iter->dir, iter->find_data.cFileName);
    NULL_TERMINATE_BUFFER(iter->fname);
    f = CreateFile(iter->fname, FILE_GENERIC_READ, FILE_SHARE_READ, NULL,
                   OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (process_name != NULL) {
        TCHAR *end;
        end = _tcsstr(iter->find_data.cFileName, _TEXT(".config"));
        if (end == NULL) {
            process_name[0] = '\0';
            ok = false;
        } else {
# ifdef UNICODE
            _snprintf(process_name, end - iter->find_data.cFileName, "%S",
                      iter->find_data.cFileName);
# else
            _snprintf(process_name, end - iter->find_data.cFileName, "%s",
                      iter->find_data.cFileName);
# endif
        }
    }
    if (!FindNextFile(iter->find_handle, &iter->find_data))
        iter->has_next = false;
    if (f == NULL || !ok)
        return false;
    read_process_policy(f, process_name, dr_root_dir, dr_mode, debug, dr_options);
    fclose(f);
    return true;
#endif
}

EXPORT
void
dr_registered_process_iterator_stop(dr_registered_process_iterator_t *iter)
{
#ifdef PARAMS_IN_REGISTRY
    if (iter->policy != NULL)
        free_config_group(iter->policy);
#else
    if (iter->find_handle != INVALID_HANDLE_VALUE)
        FindClose(iter->find_handle);
#endif
    free(iter);
}

#endif /* WINDOWS */

EXPORT
bool
dr_process_is_registered(const char *process_name,
                         process_id_t pid,
                         bool global,
                         dr_platform_t dr_platform /* OUT */,
                         char *dr_root_dir /* OUT */,
                         dr_operation_mode_t *dr_mode /* OUT */,
                         bool *debug /* OUT */,
                         char *dr_options /* OUT */)
{
    bool result = false;
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
#else
    FILE *f = open_config_file(process_name, pid, global, dr_platform,
                                true/*read*/, false/*!write*/, false/*!overwrite*/);
#endif
    if (IF_REG_ELSE(proc_policy == NULL, f == NULL))
        goto exit;
    result = true;
    
    read_process_policy(IF_REG_ELSE(proc_policy, f), NULL,
                        dr_root_dir, dr_mode, debug, dr_options);

 exit:
#ifdef PARAMS_IN_REGISTRY
    if (policy != NULL)
        free_config_group(policy);
#else
    if (f != NULL)
        fclose(f);
#endif
    return result;
}

struct _dr_client_iterator_t {
    opt_info_t opt_info;
    uint cur;
    bool valid;
};

EXPORT
dr_client_iterator_t *
dr_client_iterator_start(const char *process_name,
                         process_id_t pid,
                         bool global,
                         dr_platform_t dr_platform)
{
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
#else
    FILE *f = open_config_file(process_name, pid, global, dr_platform,
                                true/*read*/, false/*!write*/, false/*!overwrite*/);
#endif
    dr_client_iterator_t *iter = (dr_client_iterator_t *)
        malloc(sizeof(dr_client_iterator_t));
    
    iter->valid = false;
    iter->cur = 0;
    if (IF_REG_ELSE(proc_policy == NULL, f == NULL))
        return iter;
    if (read_options(&iter->opt_info, IF_REG_ELSE(proc_policy, f)) != DR_SUCCESS)
        return iter;
    iter->valid = true;

    return iter;
}

EXPORT
bool
dr_client_iterator_hasnext(dr_client_iterator_t *iter)
{
    return iter->valid && iter->cur < iter->opt_info.num_clients;
}

EXPORT
void
dr_client_iterator_next(dr_client_iterator_t *iter,
                        client_id_t *client_id, /* OUT */
                        size_t *client_pri,     /* OUT */
                        char *client_path,      /* OUT */
                        char *client_options    /* OUT */)
{
    client_opt_t *client_opt = iter->opt_info.client_opts[iter->cur];

    if (client_pri != NULL)
        *client_pri = iter->cur;

    if (client_path != NULL) {
        size_t len = MIN(MAXIMUM_PATH-1, _tcslen(client_opt->path));
        _snprintf(client_path, len, TSTR_FMT, client_opt->path);
        client_path[len] = '\0';
    }

    if (client_id != NULL)
        *client_id = client_opt->id;

    if (client_options != NULL) {
        size_t len = MIN(DR_MAX_OPTIONS_LENGTH-1, _tcslen(client_opt->opts));
        _snprintf(client_options, len, TSTR_FMT, client_opt->opts);
        client_options[len] = '\0';
    }
    
    iter->cur++;
}           

EXPORT
void
dr_client_iterator_stop(dr_client_iterator_t *iter)
{
    if (iter->valid)
        free_opt_info(&iter->opt_info);
    free(iter);
}

EXPORT
size_t
dr_num_registered_clients(const char *process_name,
                          process_id_t pid,
                          bool global,
                          dr_platform_t dr_platform)
{
    opt_info_t opt_info;
    size_t num = 0;
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
#else
    FILE *f = open_config_file(process_name, pid, global, dr_platform,
                                true/*read*/, false/*!write*/, false/*!overwrite*/);
#endif
    if (IF_REG_ELSE(proc_policy == NULL, f == NULL))
        goto exit;

    if (read_options(&opt_info, IF_REG_ELSE(proc_policy, f)) != DR_SUCCESS)
        goto exit;

    num = opt_info.num_clients;
    free_opt_info(&opt_info);
    
 exit:
#ifdef PARAMS_IN_REGISTRY
    if (policy != NULL)
        free_config_group(policy);
#else
    fclose(f);
#endif
    return num;
}


EXPORT
dr_config_status_t
dr_get_client_info(const char *process_name,
                   process_id_t pid,
                   bool global,
                   dr_platform_t dr_platform,
                   client_id_t client_id,
                   size_t *client_pri,
                   char *client_path,
                   char *client_options)
{
    opt_info_t opt_info;
    dr_config_status_t status;
    size_t i;
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
#else
    FILE *f = open_config_file(process_name, pid, global, dr_platform,
                                true/*read*/, false/*!write*/, false/*!overwrite*/);
#endif
    if (IF_REG_ELSE(proc_policy == NULL, f == NULL)) {
        status = DR_PROC_REG_INVALID;
        goto exit;
    }

    status = read_options(&opt_info, IF_REG_ELSE(proc_policy, f));
    if (status != DR_SUCCESS)
        goto exit;

    for (i=0; i<opt_info.num_clients; i++) {
        if (opt_info.client_opts[i]->id == client_id) {
            client_opt_t *client_opt = opt_info.client_opts[i];

            if (client_pri != NULL) {
                *client_pri = i;
            }

            if (client_path != NULL) {
                size_t len = MIN(MAXIMUM_PATH-1, _tcslen(client_opt->path));
                _snprintf(client_path, len, TSTR_FMT, client_opt->path);
                client_path[len] = '\0';
            }

            if (client_options != NULL) {
                size_t len = MIN(DR_MAX_OPTIONS_LENGTH-1, _tcslen(client_opt->opts));
                _snprintf(client_options, len, TSTR_FMT, client_opt->opts);
                client_options[len] = '\0';
            }

            status = DR_SUCCESS;
            goto exit;
        }
    }

    status = DR_ID_INVALID;

 exit:
#ifdef PARAMS_IN_REGISTRY
    if (policy != NULL)
        free_config_group(policy);
#else
    fclose(f);
#endif
    return status;
}


EXPORT
dr_config_status_t
dr_register_client(const char *process_name,
                   process_id_t pid,
                   bool global,
                   dr_platform_t dr_platform,
                   client_id_t client_id,
                   size_t client_pri,
                   const char *client_path,
                   const char *client_options)
{
    TCHAR new_opts[DR_MAX_OPTIONS_LENGTH];
    TCHAR wpath[MAXIMUM_PATH], woptions[DR_MAX_OPTIONS_LENGTH];
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
#else
    FILE *f = open_config_file(process_name, pid, global, dr_platform,
                                true/*read*/, true/*write*/, false/*!overwrite*/);
#endif
    dr_config_status_t status;
    opt_info_t opt_info;
    bool opt_info_alloc = false;
    size_t i;

    if (IF_REG_ELSE(proc_policy == NULL, f == NULL)) {
        status = DR_PROC_REG_INVALID;
        goto exit;
    }

    status = read_options(&opt_info, IF_REG_ELSE(proc_policy, f));
    if (status != DR_SUCCESS) {
        goto exit;
    }
    opt_info_alloc = true;

    for (i=0; i<opt_info.num_clients; i++) {
        if (opt_info.client_opts[i]->id == client_id) {
            status = DR_ID_CONFLICTING;
            goto exit;
        }
    }

    if (client_pri > opt_info.num_clients) {
        status = DR_PRIORITY_INVALID;
        goto exit;
    }

    convert_to_tchar(wpath, client_path, MAXIMUM_PATH);
    NULL_TERMINATE_BUFFER(wpath);
    convert_to_tchar(woptions, client_options, DR_MAX_OPTIONS_LENGTH);
    NULL_TERMINATE_BUFFER(woptions);

    status = add_client_lib(&opt_info, client_id, client_pri, wpath, woptions);
    if (status != DR_SUCCESS) {
        goto exit;
    }

    /* write the registry */
    write_options(&opt_info, new_opts);
#ifndef PARAMS_IN_REGISTRY
    /* shift rest of file up, overwriting old value, so we can append new value */
    read_config_ex(f, DYNAMORIO_VAR_OPTIONS, NULL, 0, true);
#endif
    status = write_config_param(IF_REG_ELSE(proc_policy, f),
                                PARAM_STR(DYNAMORIO_VAR_OPTIONS), new_opts);
    if (status != DR_SUCCESS)
        goto exit;

#ifdef PARAMS_IN_REGISTRY
    if (write_config_group(policy) != ERROR_SUCCESS) {
        status = DR_FAILURE;
        goto exit;
    }
#endif
    status = DR_SUCCESS;

 exit:
#ifdef PARAMS_IN_REGISTRY
    if (policy != NULL)
        free_config_group(policy);
#else
    if (f != NULL)
        fclose(f);
#endif
    if (opt_info_alloc)
        free_opt_info(&opt_info);
    return status;
}


EXPORT
dr_config_status_t
dr_unregister_client(const char *process_name,
                     process_id_t pid,
                     bool global,
                     dr_platform_t dr_platform,
                     client_id_t client_id)
{
    TCHAR new_opts[DR_MAX_OPTIONS_LENGTH];
#ifdef PARAMS_IN_REGISTRY
    ConfigGroup *policy = get_policy(dr_platform);
    ConfigGroup *proc_policy = get_proc_policy(policy, process_name);
#else
    FILE *f = open_config_file(process_name, pid, global, dr_platform,
                                true/*read*/, true/*write*/, false/*!overwrite*/);
#endif
    dr_config_status_t status;
    opt_info_t opt_info;
    bool opt_info_alloc = false;

    if (IF_REG_ELSE(proc_policy == NULL, f == NULL)) {
        status = DR_PROC_REG_INVALID;
        goto exit;
    }

    status = read_options(&opt_info, IF_REG_ELSE(proc_policy, f));
    if (status != DR_SUCCESS) {
        goto exit;
    }
    opt_info_alloc = true;

    status = remove_client_lib(&opt_info, client_id);
    if (status != DR_SUCCESS) {
        goto exit;
    }

    /* write the registry */
    write_options(&opt_info, new_opts);
#ifndef PARAMS_IN_REGISTRY
    /* shift rest of file up, overwriting old value, so we can append new value */
    read_config_ex(f, DYNAMORIO_VAR_OPTIONS, NULL, 0, true);
#endif
    write_config_param(IF_REG_ELSE(proc_policy, f), PARAM_STR(DYNAMORIO_VAR_OPTIONS),
                       new_opts);

#ifdef PARAMS_IN_REGISTRY
    if (write_config_group(policy) != ERROR_SUCCESS) {
        status = DR_FAILURE;
        goto exit;
    }
#endif

    status = DR_SUCCESS;

 exit:
#ifdef PARAMS_IN_REGISTRY
    if (policy != NULL)
        free_config_group(policy);
#else
    if (f != NULL)
        fclose(f);
#endif
    if (opt_info_alloc)
        free_opt_info(&opt_info);
    return status;
}

/* XXX: Support the nudge API on Linux. */
#ifdef WINDOWS

typedef struct {
    const char *process_name; /* if non-null nudges processes with matching name */
    bool all; /* if set attempts to nudge all processes */
    client_id_t client_id;
    uint64 argument;
    int count; /* number of nudges successfully delivered */
    DWORD res; /* last failing error code */
    DWORD timeout; /* amount of time to wait for nudge to finish */
} pw_nudge_callback_data_t;

static
BOOL pw_nudge_callback(process_info_t *pi, void **param)
{
    char buf[MAXIMUM_PATH];
    pw_nudge_callback_data_t *data = (pw_nudge_callback_data_t *)param;

    if (pi->ProcessID == 0)
        return true; /* skip system process */

    buf[0] = '\0';
    if (pi->ProcessName != NULL)
        _snprintf(buf, MAXIMUM_PATH, "%S", pi->ProcessName);
    NULL_TERMINATE_BUFFER(buf);
    if (data->all || (data->process_name != NULL &&
                      strnicmp(data->process_name, buf, MAXIMUM_PATH) == 0)) {
        DWORD res = generic_nudge(pi->ProcessID, true, NUDGE_GENERIC(client),
                                  data->client_id, data->argument, data->timeout);
        if (res == ERROR_SUCCESS || res == ERROR_TIMEOUT) {
            data->count++;
            if (res == ERROR_TIMEOUT && data->timeout != 0)
                data->res = ERROR_TIMEOUT;
        }
        else if (res != ERROR_MOD_NOT_FOUND /* so failed for a good reason */)
            data->res = res;
    }

    return true;
}

/* TODO: must be careful in invoking the correct VIPA's nudge handler,
 * particularly a problem with multiple agents, but can be a problem even in a
 * single agent if some other dll exports dr_nudge_handler() (remote
 * contingency).
 */

EXPORT
dr_config_status_t
dr_nudge_process(const char *process_name, client_id_t client_id, uint64 arg,
                 uint timeout_ms, int *nudge_count /*OUT */)
{
    pw_nudge_callback_data_t data = {0};
    data.process_name = process_name;
    data.client_id = client_id;
    data.argument = arg;
    data.timeout = timeout_ms;
    data.res = ERROR_SUCCESS;
    process_walk(&pw_nudge_callback, (void **)&data);
    if (nudge_count != NULL)
        *nudge_count = data.count;

    if (data.res == ERROR_SUCCESS)
        return DR_SUCCESS;
    else if (data.res == ERROR_TIMEOUT)
        return DR_NUDGE_TIMEOUT;
    else
        return DR_FAILURE;
}

EXPORT
dr_config_status_t
dr_nudge_pid(process_id_t process_id, client_id_t client_id, uint64 arg, uint timeout_ms)
{
    DWORD res = generic_nudge(process_id, true, NUDGE_GENERIC(client),
                              client_id, arg, timeout_ms);

    if (res == ERROR_SUCCESS)
        return DR_SUCCESS;
    if (res == ERROR_MOD_NOT_FOUND)
        return DR_NUDGE_PID_NOT_INJECTED;
    if (res == ERROR_TIMEOUT && timeout_ms != 0)
        return DR_NUDGE_TIMEOUT;
    return DR_FAILURE;
}

EXPORT
dr_config_status_t
dr_nudge_all(client_id_t client_id, uint64 arg, uint timeout_ms, int *nudge_count /*OUT*/)
{
    pw_nudge_callback_data_t data = {0};
    data.all = true;
    data.client_id = client_id;
    data.argument = arg;
    data.timeout = timeout_ms;
    data.res = ERROR_SUCCESS;
    process_walk(&pw_nudge_callback, (void **)&data);
    if (nudge_count != NULL)
        *nudge_count = data.count;

    if (data.res == ERROR_SUCCESS)
        return DR_SUCCESS;
    if (data.res == ERROR_TIMEOUT)
        return DR_NUDGE_TIMEOUT;
    return DR_FAILURE;
}

#endif /* WINDOWS */
