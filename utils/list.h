#ifndef LIST_H
#define LIST_H

/*===========================================================================*/
/**
 * @file list.h
 *
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * API for the Simple Linked Lists implementation
 *
 * @todo Add full description here
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
#include <stddef.h>
#include <stdbool.h>

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
typedef struct ListNode
{
    void *data;
    struct ListNode *next;
} ListNode;

typedef struct
{
    ListNode *head;
    ListNode *tail;
    size_t size;

    /* pool of nodes preallocated by the user */
    ListNode *pool;
    size_t pool_capacity;

    /* nodes available */
    ListNode *free_list;

} List;

/*****************************************************************************
 * @typedef    list_match_fn
 * @brief      Callback used by list_contains() to compare node data against
 *             a user-provided criteria.
 * @param[in]  node_data   Pointer to the data stored in the list node.
 * @param[in]  criteria    Pointer to the matching criteria.
 * @return     true if node_data matches criteria, false otherwise.
 ******************************************************************************/
typedef bool (*list_match_fn)(const void *node_data, const void *criteria);

/*****************************************************************************
 * @typedef    list_free_fn
 * @brief      Callback used to free the used memory by data
 * @param[in]  data   Pointer to the data stored in the list node.
 * @return     None
 ******************************************************************************/
typedef void (*list_free_fn)(void *data);

/*****************************************************************************
 * @typedef    list_iter_fn
 * @brief      Callback called by the foreach function to analyze each element
 * @param[in]  data   Pointer to the data stored in the list node.
 * @return     None
 ******************************************************************************/
typedef void (*list_iter_fn)(void *data);

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
 * @fn         list_init
 * @brief      Initializes a list structure, setting it to an empty state.
 * @param [in] list   Pointer to the list instance to initialize.
 * @param[in]  pool     Pointer to the List of nodes preallocated by the user
 * @param[in]  capacity Nodes available for the Linked list
 * @return     True if the list is init ok, False otherwise.
 ******************************************************************************/
bool list_init(List *list, ListNode *pool, size_t capacity);

/*****************************************************************************
 * @fn         bool list_is_empty(const List *list);
 * @brief      Checks whether the linked list contains no elements.
 * @param[in]  list     Pointer to the List instance.
 * @return     true if the list is empty, false otherwise.
 ******************************************************************************/
bool list_is_empty(const List *list);

/*****************************************************************************
 * @fn         list_size
 * @brief      Retrieves the number of elements currently stored in the
 *             linked list.
 * @param[in]  list   Pointer to the List instance.
 * @return     The number of elements in the list.
 ******************************************************************************/
size_t list_size(const List *list);

/*****************************************************************************
 * @fn         list_foreach
 * @brief      Iterates over all elements stored in the linked list
 *             and invokes the supplied callback for each node's data.
 * @param[in]  list   Pointer to the List instance.
 * @param[in]  fn     Function to be executed for each node; receives node data.
 * @return     None.
 ******************************************************************************/
void list_foreach(List *list, list_iter_fn fn);

/*****************************************************************************
 * @fn         list_push_head
 * @brief      Inserts a new element at the beginning of the list.
 * @param[in]  list   Pointer to the list instance.
 * @param[in]  data   Pointer to the data to be stored in the new node.
 * @return     true if the data is stored ok, false otherwise.
 ******************************************************************************/
bool list_push_head(List *list, void *data);

/*****************************************************************************
 * @fn         list_push_tail
 * @brief      Inserts a new element at the end of the list.
 * @param[in]  list   Pointer to the list instance.
 * @param[in]  data   Pointer to the data to be stored in the new node.
 * @return     true if the data is stored ok, false otherwise.
 ******************************************************************************/
bool list_push_tail(List *list, void *data);

/*****************************************************************************
 * @fn         list_pop_head
 * @brief      Removes and returns the element stored at the beginning
 *             of the list.
 * @param[in]  list   Pointer to the list instance.
 * @return     Pointer to the removed data, or NULL if the list is empty.
 ******************************************************************************/
void *list_pop_head(List *list);

/*****************************************************************************
 * @fn         list_pop_tail
 * @brief      Removes and returns the element stored at the end
 *             of the list.
 * @param[in]  list   Pointer to the list instance.
 * @return     Pointer to the removed data, or NULL if the list is empty.
 ******************************************************************************/
void *list_pop_tail(List *list);

/*****************************************************************************
 * @fn         list_contains
 * @brief      Checks whether the list contains at least one element that
 *             matches the given criteria, using a user-provided callback.
 * @param[in]  list      Pointer to the list instance.
 * @param[in]  criteria  Pointer to the user-provided comparison data.
 * @param[in]  match     Function used to compare node_data vs criteria.
 * @return     true if a matching element is found, false otherwise.
 ******************************************************************************/
bool list_contains(const List *list, const void *criteria, list_match_fn match);

/*****************************************************************************
 * @fn         list_find
 * @brief      Searches the list for the first node whose data matches the
 *             provided criteria using the given comparison callback.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  criteria  Pointer to the user-provided comparison data.
 * @param[in]  match     Function used to compare node_data vs criteria.
 * @return     Pointer to the matching data, or NULL if no match is found.
 ******************************************************************************/
void *list_find(const List *list, const void *criteria, list_match_fn match);

/*****************************************************************************
 * @fn         list_remove
 * @brief      Removes the first node whose data matches the provided criteria
 *             using the comparison callback.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  criteria  Pointer to the user-provided comparison data.
 * @param[in]  match     Function used to determine if a node matches criteria.
 * @param[in]  free_fn   Function used to free node data (may be NULL).
 * @return     true if a node was removed, false otherwise.
 ******************************************************************************/
bool list_remove(List *list, const void *criteria, list_match_fn match, list_free_fn free_fn);

/*****************************************************************************
 * @fn         list_clear
 * @brief      Removes all nodes from the list.
 *             If free_fn is provided, it is called for each node’s data.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  free_fn   Function used to free node data (may be NULL).
 * @return     None.
 ******************************************************************************/
void list_clear(List *list, list_free_fn free_fn);

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
#endif /* LIST_H */
