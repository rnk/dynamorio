/* **********************************************************
 * Copyright (c) 2005-2010 VMware, Inc.  All rights reserved.
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


/*
 * 
 * configuartion interface definitions
 *
 *
 * this is intended to serve as a replacement for the cluttered
 *  policy.c code for reading and writing core parameters. OOP
 *  style has been kept in mind to allow easy porting to c++ 
 *  if/when we decide to make that switch.
 *
 * usage:
 *  a ConfigGroup is more or less equivalent to a registry key: it can
 *  hold name-value pairs and 'children' ConfigGroups. ConfigGroup 
 *  paths are ":" separated, and for registry purposes are assumed to
 *  be rooted at HKLM/Software/Determina. the idea is that if we move
 *  away from the registry (eg, due to moving to another platform or
 *  to config files for core params), this interface should still be
 *  usable, and the only change would be in {read,write}_config_group.
 *
 *  there are also direct-access single-parameter config
 *  functions; these allow for arbitray registry read/write. however, 
 *  unless otherwise specified with the 'absolute' parameter, these
 *  are still based at the determina key. 
 *
 *
 */

#ifndef _DETERMINA_CONFIG_H_
#define _DETERMINA_CONFIG_H_

#include "lib/globals_shared.h"
#include "our_tchar.h"

#ifdef __cplusplus
extern "C"{
#endif 

/* we have circular dep issues if we include share.h */
/* XXX: Do we really? */
//typedef ptr_uint_t process_id_t;

#ifndef WINDOWS
# define BOOL bool
#endif

#define MAX_PARAM_LEN 1024
#define CONFIG_PATH_SEPARATOR L':'
#define LIST_SEPARATOR_CHAR L';'
#define APPINIT_SEPARATOR_CHAR L','
#define CONFIGURATION_ROOT_REGISTRY_KEY L"Software\\"L_EXPAND_LEVEL(COMPANY_NAME)

/* this provides a hook for forced parameter deletion, even if
 * "should_clear" is FALSE. */
#define L_DELETE_PARAMETER_KEY L"__DELETE_PARAMETER_KEY"


/* note this does NOT include the null terminator: the limit
 *  is 31 chars. */
#define APPINIT_SYSTEM32_LENGTH_LIMIT  31

typedef struct NameValuePairNode_ {
    struct NameValuePairNode_ *next;
    TCHAR *name;
    TCHAR *value;
} NameValuePairNode;

typedef struct ConfigGroup_ {
    struct ConfigGroup_ *next;
    struct ConfigGroup_ *children;
    /* used internally only for efficiency */
    struct ConfigGroup_ *lastchild;
    NameValuePairNode *params;
    BOOL should_clear;
    TCHAR *name;
} ConfigGroup;


/* group config management */

#ifdef WINDOWS
int
get_key_handle(HKEY *key, HKEY parent, const TCHAR *path, 
               BOOL absolute, int flags);

int
recursive_delete_key(HKEY parent, const TCHAR *keyname, ConfigGroup *filter);
#endif

ConfigGroup *
new_config_group(const TCHAR *name);

ConfigGroup *
copy_config_group(ConfigGroup *config, BOOL deep);

/* name is the subkey name from the system registry root (e.g., 
 *  HKLM/Software/Determina). */
int
read_config_group(ConfigGroup **configptr, const TCHAR *name, 
                  BOOL read_children_recursively);

void
set_should_clear(ConfigGroup *config, BOOL should_clear);

void
remove_children(ConfigGroup *config);

void
remove_child(const TCHAR *child, ConfigGroup *config);

TCHAR *
get_config_group_parameter(ConfigGroup *config, const TCHAR *name);

void
set_config_group_parameter(ConfigGroup *config, 
                           const TCHAR *name, const TCHAR *value);

void
remove_config_group_parameter(ConfigGroup *config, const TCHAR *name);

void
add_config_group(ConfigGroup *parent, ConfigGroup *new_child);

ConfigGroup *
get_child(const TCHAR *name, ConfigGroup *c);

int
write_config_group(ConfigGroup *config);

void
free_config_group(ConfigGroup *configptr);

void
dump_nvp(NameValuePairNode *nvpn);

void
dump_config_group(char *prefix, char *incr, ConfigGroup *c, BOOL traverse);

BOOL
get_config_group_parameter_bool(ConfigGroup *config, const TCHAR *name);

int
get_config_group_parameter_int(ConfigGroup *config, const TCHAR *name);

void
get_config_group_parameter_scrambled(ConfigGroup *config, const TCHAR *name,
                                     TCHAR *buffer, uint maxchars);

void
set_config_group_parameter_bool(ConfigGroup *config, const TCHAR *name,
                                BOOL value);

void
set_config_group_parameter_int(ConfigGroup *config, const TCHAR *name,
                               int value);

void
set_config_group_parameter_ascii(ConfigGroup *config, const TCHAR *name,
                                 char *value);

void
set_config_group_parameter_scrambled(ConfigGroup *config, const TCHAR *name,
                                     const TCHAR *value);


/* single parameter config functions */

int
set_config_parameter(const TCHAR *path, BOOL absolute, 
                     const TCHAR *name, const TCHAR *value);

int
get_config_parameter(const TCHAR *path, BOOL absolute, 
                     const TCHAR *name, TCHAR *value, int maxlen);

#ifdef WINDOWS
int
read_reg_string(HKEY subkey, const TCHAR *keyname, TCHAR *value, int valchars);

/* if value is NULL, the value will be deleted */
int 
write_reg_string(HKEY subkey, const TCHAR *keyname, const TCHAR *value);
#endif

/* identifies processes relative to a ConfigGroup */
ConfigGroup *
get_process_config_group(ConfigGroup *config, process_id_t pid);

BOOL
is_parent_of_qualified_config_group(ConfigGroup *config);

/* tries both with and without no_strip */
ConfigGroup *
get_qualified_config_group(ConfigGroup *config, 
                           const TCHAR *exename, const TCHAR *cmdline);

/* some list management and utility routines */
/* all lists are ;-separated */
/* comparisons are case insensitive */
/* filename comparisons are independent of path */

TCHAR *
new_file_list(size_t initial_chars);

void
free_file_list(TCHAR *list);

/* 
 * given a ;-separated list and a filename, return a pointer to 
 *  the filename in the list, if it appears. comparisons are 
 *  case insensitive and independent of path; eg, 
 *     get_entry_location("c:\\foo\\bar.dll;blah;...", "D:\\Bar.DLL")
 *  would return a pointer to the beginning of the list.
 */
TCHAR *
get_entry_location(const TCHAR *list, const TCHAR *filename, TCHAR separator);

BOOL
is_in_file_list(const TCHAR *list, const TCHAR *filename, TCHAR separator);

/* frees the old list and returns a newly alloc'ed one */
TCHAR *
add_to_file_list(TCHAR *list, const TCHAR *filename, 
                 BOOL check_for_duplicates, BOOL add_to_front, 
                 BOOL overwrite_existing, TCHAR separator);

void
remove_from_file_list(TCHAR *list, const TCHAR *filename, TCHAR separator);

BOOL
blacklist_filter(TCHAR *list, const TCHAR *blacklist, 
                 BOOL check_only, TCHAR separator);

BOOL
whitelist_filter(TCHAR *list, const TCHAR *whitelist, 
                 BOOL check_only, TCHAR separator);

int
set_autoinjection_ex(BOOL inject, int flags, 
                     const TCHAR *blacklist, 
                     const TCHAR *whitelist, int *list_error, 
                     const TCHAR *custom_preinject_name, 
                     TCHAR *current_list, size_t maxchars);

int
set_custom_autoinjection(const TCHAR *preinject, int flags);

int
set_autoinjection();

int
unset_custom_autoinjection(const TCHAR *preinject, int flags);

int
unset_autoinjection();

BOOL
is_autoinjection_set();

BOOL
is_custom_autoinjection_set(const TCHAR *preinject);

int
set_loadappinit();

int
unset_loadappinit();

BOOL
is_loadappinit_set();

int
create_eventlog(const TCHAR *dll_path);

int
destroy_eventlog(void);

BOOL
is_vista(void);

BOOL
is_win7(void);

int
copy_earlyhelper_dlls(const TCHAR *dir);

#ifdef __cplusplus
}
#endif

#endif /* _DETERMINA_CONFIG_H_ */
