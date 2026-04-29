/*===========================================================================*/
/**
 * @file dict.c
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Dictionaries implementation based in hash tables
 *
 * @section ABBR ABBREVIATIONS:
 *   - @todo List any abbreviations, precede each with a dash ('-').
 *
 * @section TRACE TRACEABILITY INFO:
 *   - Design Document(s):
 *     - @todo Update list of design document(s).
 *
 *   - Requirements Document(s):
 *     - @todo Update list of requirements document(s)
 *
 *   - Applicable Standards (in order of precedence: highest first):
 *     - @todo Update list of other applicable standards
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dict.h"

/*===========================================================================*
 * Local Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Object Declarations
 *===========================================================================*/

/*===========================================================================*
 * Local Variables Definitions
 *===========================================================================*/

/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/

/*===========================================================================*
 * Local Inline Function Definitions and Function-Like Macros
 *===========================================================================*/

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/
/*****************************************************************************
 * Name         hash_key
 * Description  hash function for strings based in djb2 algorithm, created by
 *              Daniel J. Bernstein.
 *****************************************************************************/
static uint32_t hash_key(const char *str)
{
    uint32_t hash = 5381;
    int c;
    while ((c = *str++))
    {
        hash = ((hash << 5) + hash) + c;
    }
    return hash;
}

/*****************************************************************************
 * Name         dict_init
 * Description  Initializes a dictionary instance with provided storage and configuration.
 *****************************************************************************/
bool dict_init(dict_t *dict, void *storage, uint16_t max_entries,
               uint16_t key_max_len, dict_free_fn free_fn)
{
    if (!dict || !storage || max_entries == 0)
        return false;

    dict->entries = (dict_entry_t *)storage;
    dict->max_entries = max_entries;
    dict->key_max_len = key_max_len;
    dict->count = 0;
    dict->free_fn = free_fn;

    // Initialize all entries
    char *ptr = (char *)storage;
    char *key_storage = ptr + sizeof(dict_entry_t) * max_entries;

    for (int i = 0; i < max_entries; i++)
    {
        dict->entries[i].occupied = false;
        dict->entries[i].key = key_storage + (key_max_len * i);
        dict->entries[i].value = NULL;
        dict->entries[i].key[0] = '\0';
    }

    return true;
}

/*****************************************************************************
 * Name         dict_calc_storage_size_req
 * Description  Calculates the required storage size for a dictionary.
 *****************************************************************************/
size_t dict_calc_storage_size_req(uint16_t max_entries, uint16_t key_max_len)
{
    return sizeof(dict_entry_t) * max_entries + (key_max_len * max_entries);
}

/*****************************************************************************
 * Name         dict_set
 * Description  Sets or updates the value associated with a key.
 *****************************************************************************/
bool dict_set(dict_t *dict, const char *key, void *value)
{
    if (!dict || !key)
        return false;
    if (strlen(key) >= dict->key_max_len)
        return false;

    uint32_t hash = hash_key(key);
    uint32_t idx = hash % dict->max_entries;
    uint32_t start_idx = idx;

    do
    {
        if (!dict->entries[idx].occupied)
        { 
            /* If the entry is available copy the key and value */
            strncpy(dict->entries[idx].key, key, dict->key_max_len - 1);
            dict->entries[idx].key[dict->key_max_len - 1] = '\0';
            dict->entries[idx].value = value;
            dict->entries[idx].occupied = true;
            dict->count++;
            return true;
        }
        else if (strcmp(dict->entries[idx].key, key) == 0)
        {   
            /* If the key is the same, free the prev value and update the new one */
            if (dict->free_fn && dict->entries[idx].value)
            {
                dict->free_fn(dict->entries[idx].value);
            }
            dict->entries[idx].value = value;
            return true;
        }
        idx = (idx + 1) % dict->max_entries;
    } while (idx != start_idx);

    return false; // Dictionary full
}

/*****************************************************************************
 * Name         dict_get
 * Description  Retrieves the value associated with the given key.
 *****************************************************************************/
void *dict_get(dict_t *dict, const char *key)
{
    if (!dict || !key)
        return NULL;

    uint32_t hash = hash_key(key);
    uint32_t idx = hash % dict->max_entries;
    uint32_t start_idx = idx;

    do
    {
        if (!dict->entries[idx].occupied)
            return NULL;
        if (strcmp(dict->entries[idx].key, key) == 0)
            return dict->entries[idx].value;
        idx = (idx + 1) % dict->max_entries;
    } while (idx != start_idx);

    return NULL;
}

/*****************************************************************************
 * Name         dict_contains
 * Description  Checks if the dictionary contains the specified key.
 *****************************************************************************/
bool dict_contains(dict_t *dict, const char *key)
{
    return dict_get(dict, key) != NULL;
}

/*****************************************************************************
 * Name         dict_delete
 * Description  Removes the entry for the specified key.
 *****************************************************************************/
bool dict_delete(dict_t *dict, const char *key)
{
    if (!dict || !key)
        return false;

    uint32_t hash = hash_key(key);
    uint32_t idx = hash % dict->max_entries;
    uint32_t start_idx = idx;

    do
    {
        if (!dict->entries[idx].occupied)
            return false;
        if (strcmp(dict->entries[idx].key, key) == 0)
        {
            if (dict->free_fn && dict->entries[idx].value)
                dict->free_fn(dict->entries[idx].value);
            dict->entries[idx].occupied = false;
            dict->entries[idx].key[0] = '\0';
            dict->entries[idx].value = NULL;
            dict->count--;
 
            /******************************************************************************
            * Rehash subsequent entries to maintain correct probe sequence after deletion:
            * In linear probing, keys that collided may be displaced from their "ideal" slot.
            * If we simply mark a deleted slot as empty, a lookup for a displaced key could fail.
            * This loop moves any following entries that should come before the deleted slot
            * back into the empty slot, preserving the invariant that all keys are reachable
            * by probing from their hashed index.
            *******************************************************************************/
            uint32_t next_idx = (idx + 1) % dict->max_entries;
            while (dict->entries[next_idx].occupied)
            {
                uint32_t ideal_idx = hash_key(dict->entries[next_idx].key) % dict->max_entries;
                if ((idx < next_idx && (ideal_idx <= idx || ideal_idx > next_idx)) ||
                    (idx > next_idx && (ideal_idx <= idx && ideal_idx > next_idx)))
                {
                    strcpy(dict->entries[idx].key, dict->entries[next_idx].key);
                    dict->entries[idx].value = dict->entries[next_idx].value;
                    dict->entries[idx].occupied = true;
                    dict->entries[next_idx].occupied = false;
                    dict->entries[next_idx].key[0] = '\0';
                    dict->entries[next_idx].value = NULL;
                    idx = next_idx;
                }
                next_idx = (next_idx + 1) % dict->max_entries;
            }
            return true;
        }
        idx = (idx + 1) % dict->max_entries;
    } while (idx != start_idx);

    return false;
}

/*****************************************************************************
 * Name         dict_foreach
 * Description  Iterates over all dictionary entries and calls a callback function.
 *****************************************************************************/
void dict_foreach(dict_t *dict, dict_iter_fn fn)
{
    if (!dict || !fn)
        return;
    for (int i = 0; i < dict->max_entries; i++)
    {
        if (dict->entries[i].occupied)
            fn(dict->entries[i].key, dict->entries[i].value);
    }
}

/*****************************************************************************
 * Name         dict_size
 * Description  Returns the number of entries currently stored in the dictionary.
 *****************************************************************************/
uint16_t dict_size(dict_t *dict)
{
    return dict ? dict->count : 0;
}

/*****************************************************************************
 * Name         dict_clear
 * Description  Removes all entries from the dictionary.
 *****************************************************************************/
void dict_clear(dict_t *dict)
{
    if (!dict)
        return;
    for (int i = 0; i < dict->max_entries; i++)
    {
        if (dict->entries[i].occupied)
        {
            if (dict->free_fn && dict->entries[i].value)
                dict->free_fn(dict->entries[i].value);
            dict->entries[i].occupied = false;
            dict->entries[i].key[0] = '\0';
            dict->entries[i].value = NULL;
        }
    }
    dict->count = 0;
}
