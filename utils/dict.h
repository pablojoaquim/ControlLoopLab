#ifndef DICT_H
#define DICT_H

/*===========================================================================*/
/**
 * @file dict.h
 *
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Dictionary (hash table) implementation using **linear probing** with fixed-size storage.
 *
 * This dictionary stores key-value pairs with:
 *   - Preallocated memory: all entries and keys are allocated in a single block,
 *     avoiding dynamic allocation per key/value.
 *   - Linear probing: hash collisions are resolved by scanning the next available slot.
 *   - Optional custom free function: to safely release dynamically allocated values.
 *
 * Key details:
 *   - dict_init initializes the dictionary using a user-provided storage buffer.
 *     Each entry's key pointer is set to a slice of this contiguous key storage:
 *       `entries[i].key = key_storage + (key_max_len * i);`
 *   - dict_set inserts or updates a key. If the key exists, the old value is freed via free_fn.
 *   - dict_delete marks the entry as empty and rehashes following entries to maintain probe sequences.
 *   - dict_foreach iterates over all occupied entries.
 *
 * This design is suitable for embedded systems or environments without dynamic memory, 
 * providing predictable memory usage and avoiding fragmentation.
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
 * Header Files (Common to C and C++)
 *===========================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
/*===========================================================================*
 * Header Files (C++ only)
 *===========================================================================*/
#endif

/*===========================================================================*
 * Exported Preprocessor #define Constants
 *===========================================================================*/

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/
/*****************************************************************************
 * @typedef    dict_free_fn
 * @brief      Callback used to free the used memory by data
 * @param[in]  value   Pointer to the data stored in the node.
 * @return     None
 ******************************************************************************/
typedef void (*dict_free_fn)(void *value);

/*****************************************************************************
 * @typedef    dict_iter_fn
 * @brief      Callback called by the foreach function to analyze each element
 * @param[in]  data   Pointer to the data stored in the list node.
 * @return     None
 ******************************************************************************/
typedef void (*dict_iter_fn)(const char *key, void *value);

/* The item struct */
typedef struct {
    char *key;
    void *value;
    bool occupied;
} dict_entry_t;

/* The dictionary struct */
typedef struct {
    dict_entry_t *entries;
    uint16_t max_entries;
    uint16_t key_max_len;
    uint16_t count;
    dict_free_fn free_fn;  // Custom free function for values
} dict_t;

/*===========================================================================*
 * Exported Classes (C++ only)
 *===========================================================================*/
#ifdef __cplusplus
// @todo: Add C++ class declarations here.
#endif

/*===========================================================================*
 * Exported C Function Prototypes
 *===========================================================================*/
#ifdef __cplusplus
extern "C"
{
#endif
// @todo: Add pure C function prototypes here.
/*****************************************************************************
 * @fn         dict_init
 * @brief      Initializes a dictionary instance with provided storage and configuration.
 * @param[in]  dict         Pointer to the dictionary instance.
 * @param[in]  storage      Pointer to pre-allocated storage for dictionary entries.
 * @param[in]  max_entries  Maximum number of entries the dictionary can hold.
 * @param[in]  key_max_len  Maximum length of each key (including null terminator).
 * @param[in]  free_fn      Function to free values when deleted (may be NULL).
 * @return     true if initialization succeeds, false otherwise.
 ******************************************************************************/
bool dict_init(dict_t *dict, void *storage, uint16_t max_entries, 
               uint16_t key_max_len, dict_free_fn free_fn);

/*****************************************************************************
 * @fn         dict_storage_size
 * @brief      Calculates the required storage size for a dictionary with the
 *             specified configuration.
 * @param[in]  max_entries  Maximum number of entries.
 * @param[in]  key_max_len  Maximum length of each key.
 * @return     Required storage size in bytes.
 ******************************************************************************/
size_t dict_calc_storage_size_req(uint16_t max_entries, uint16_t key_max_len);

/*****************************************************************************
 * @fn         dict_set
 * @brief      Sets or updates the value associated with a key in the dictionary.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @param[in]  key   Null-terminated string key.
 * @param[in]  value Pointer to the value to store.
 * @return     true if the operation succeeds, false otherwise.
 ******************************************************************************/
bool dict_set(dict_t *dict, const char *key, void *value);

/*****************************************************************************
 * @fn         dict_get
 * @brief      Retrieves the value associated with a given key.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @param[in]  key   Null-terminated string key.
 * @return     Pointer to the value if found, NULL otherwise.
 ******************************************************************************/
void* dict_get(dict_t *dict, const char *key);

/*****************************************************************************
 * @fn         dict_contains
 * @brief      Checks if the dictionary contains the specified key.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @param[in]  key   Null-terminated string key.
 * @return     true if the key exists, false otherwise.
 ******************************************************************************/
bool dict_contains(dict_t *dict, const char *key);

/*****************************************************************************
 * @fn         dict_delete
 * @brief      Removes the entry associated with the specified key.
 *             Calls the free function if defined.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @param[in]  key   Null-terminated string key.
 * @return     true if the entry was removed, false otherwise.
 ******************************************************************************/
bool dict_delete(dict_t *dict, const char *key);

/*****************************************************************************
 * @fn         dict_size
 * @brief      Returns the number of entries currently stored in the dictionary.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @return     Number of entries.
 ******************************************************************************/
uint16_t dict_size(dict_t *dict);

/*****************************************************************************
 * @fn         dict_foreach
 * @brief      Iterates over all entries in the dictionary and calls the provided
 *             callback function for each entry.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @param[in]  fn    Function to be executed for each key/value pair.
 * @return     None.
 ******************************************************************************/
void dict_foreach(dict_t *dict, dict_iter_fn fn);

/*****************************************************************************
 * @fn         dict_clear
 * @brief      Removes all entries from the dictionary.
 *             Calls free_fn for each value if defined.
 * @param[in]  dict  Pointer to the dictionary instance.
 * @return     None.
 ******************************************************************************/
void dict_clear(dict_t *dict);

#ifdef __cplusplus
} /* extern "C" */
#endif

/*===========================================================================*
 * Exported C++ Function Prototypes
 *===========================================================================*/
#ifdef __cplusplus
// @todo: Add pure C++ function prototypes here.

#endif /* __cplusplus */

/*===========================================================================*/
/*===========================================================================*/
#endif /* DICT_H */
