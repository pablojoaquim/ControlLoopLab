#ifndef DLIST_H
#define DLIST_H

/*===========================================================================*/
/**
 * @file dlist.h
 *
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * API for the Double Linked Lists implementation
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
typedef struct DListNode
{
    void *data;
    struct DListNode *next;
    struct DListNode *prev;
} DListNode;

typedef struct
{
    DListNode *head;
    DListNode *tail;
    size_t size;

    /* Prealocated pool of nodes */
    DListNode *pool;
    size_t pool_capacity;

    /* nodes available */
    DListNode *free_list;

} DList;

/*****************************************************************************
 * @typedef    dlist_match_fn
 * @brief      Callback used by list_contains() to compare node data against
 *             a user-provided criteria.
 * @param[in]  node_data   Pointer to the data stored in the list node.
 * @param[in]  criteria    Pointer to the matching criteria.
 * @return     true if node_data matches criteria, false otherwise.
 ******************************************************************************/
typedef bool (*dlist_match_fn)(const void *node_data, const void *criteria);

/*****************************************************************************
 * @typedef    dlist_free_fn
 * @brief      Callback used to free the used memory by data
 * @param[in]  data   Pointer to the data stored in the list node.
 * @return     None
 ******************************************************************************/
typedef void (*dlist_free_fn)(void *data);

/*****************************************************************************
 * @typedef    dlist_iter_fn
 * @brief      Callback called by the foreach function to analyze each element
 * @param[in]  data   Pointer to the data stored in the list node.
 * @return     None
 ******************************************************************************/
typedef void (*dlist_iter_fn)(void *data);

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
 * @fn         dlist_init
 * @brief      Initializes a DList structure, setting it to an empty state.
 * @param [in] list   Pointer to the DList instance to initialize.
 * @return     True if the list is init ok, False otherwise.
 ******************************************************************************/
bool dlist_init(DList *list, DListNode *pool, size_t capacity);

/*****************************************************************************
 * @fn         dlist_is_empty
 * @brief      Checks whether the doubly-linked list contains no elements.
 * @param[in]  list   Pointer to the DList instance.
 * @return     true if the list is empty, false otherwise.
 ******************************************************************************/
bool dlist_is_empty(const DList *list);

/*****************************************************************************
 * @fn         dlist_size
 * @brief      Retrieves the number of elements currently stored in the
 *             doubly-linked list.
 * @param[in]  list   Pointer to the DList instance.
 * @return     The number of elements in the list.
 ******************************************************************************/
size_t dlist_size(const DList *list);

/*****************************************************************************
 * @fn         dlist_foreach
 * @brief      Iterates over all elements stored in the doubly-linked list
 *             and invokes the supplied callback for each node's data.
 * @param[in]  list   Pointer to the DList instance.
 * @param[in]  fn     Function to be executed for each node; receives node data.
 * @return     None.
 ******************************************************************************/
void dlist_foreach(DList *list, dlist_iter_fn fn);

/*****************************************************************************
 * @fn         dlist_push_head
 * @brief      Inserts a new element at the beginning of the list.
 * @param[in]  list   Pointer to the DList instance.
 * @param[in]  data   Pointer to the data to be stored in the new node.
 * @return     true if the data is stored ok, false otherwise.
 ******************************************************************************/
bool dlist_push_head(DList *list, void *data);

/*****************************************************************************
 * @fn         dlist_push_tail
 * @brief      Inserts a new element at the end of the list.
 * @param[in]  list   Pointer to the DList instance.
 * @param[in]  data   Pointer to the data to be stored in the new node.
 * @return     true if the data is stored ok, false otherwise.
 ******************************************************************************/
bool dlist_push_tail(DList *list, void *data);

/*****************************************************************************
 * @fn         dlist_pop_head
 * @brief      Removes and returns the element stored at the beginning
 *             of the list.
 * @param[in]  list   Pointer to the DList instance.
 * @return     Pointer to the removed data, or NULL if the DList is empty.
 ******************************************************************************/
void *dlist_pop_head(DList *list);

/*****************************************************************************
 * @fn         dlist_pop_tail
 * @brief      Removes and returns the element stored at the end
 *             of the list.
 * @param[in]  list   Pointer to the DList instance.
 * @return     Pointer to the removed data, or NULL if the DList is empty.
 ******************************************************************************/
void *dlist_pop_tail(DList *list);

/*****************************************************************************
 * @fn         dlist_contains
 * @brief      Checks whether the list contains at least one element that
 *             matches the given criteria, using a user-provided callback.
 * @param[in]  list      Pointer to the DList instance.
 * @param[in]  criteria  Pointer to the user-provided comparison data.
 * @param[in]  match     Function used to compare node_data vs criteria.
 * @return     true if a matching element is found, false otherwise.
 ******************************************************************************/
bool dlist_contains(const DList *list, const void *criteria, dlist_match_fn match);

/*****************************************************************************
 * @fn         dlist_find
 * @brief      Searches the list for the first node whose data matches the
 *             provided criteria using the given comparison callback.
 * @param[in]  list      Pointer to the DList instance.
 * @param[in]  criteria  Pointer to the user-provided comparison data.
 * @param[in]  match     Function used to compare node_data vs criteria.
 * @return     Pointer to the matching data, or NULL if no match is found.
 ******************************************************************************/
void *dlist_find(const DList *list, const void *criteria, dlist_match_fn match);

/*****************************************************************************
 * @fn         dlist_remove
 * @brief      Removes the first node whose data matches the provided criteria
 *             using the comparison callback.
 * @param[in]  list      Pointer to the DList instance.
 * @param[in]  criteria  Pointer to the user-provided comparison data.
 * @param[in]  match     Function used to determine if a node matches criteria.
 * @param[in]  free_fn   Function used to free node data (may be NULL).
 * @return     true if a node was removed, false otherwise.
 ******************************************************************************/
bool dlist_remove(DList *list, const void *criteria, dlist_match_fn match, dlist_free_fn free_fn);

/*****************************************************************************
 * @fn         dlist_clear
 * @brief      Removes all nodes from the list.
 *             If free_fn is provided, it is called for each node’s data.
 * @param[in]  list      Pointer to the DList instance.
 * @param[in]  free_fn   Function used to free node data (may be NULL).
 * @return     None.
 ******************************************************************************/
void dlist_clear(DList *list, dlist_free_fn free_fn);

/*****************************************************************************
 * @fn         dlist_insert_before
 * @brief      Inserts a new node before the given position node.
 * @param[in]  list   Pointer to the DList instance.
 * @param[in]  pos    Pointer to the node before which the new node is inserted.
 * @param[in]  data   Pointer to the data to store in the new node.
 * @return     True if the node is inserted, False otherwise.
 ******************************************************************************/
bool dlist_insert_before(DList *list, DListNode *pos, void *data);

/*****************************************************************************
 * @fn         dlist_insert_after
 * @brief      Inserts a new node after the given position node.
 * @param[in]  list   Pointer to the DList instance.
 * @param[in]  pos    Pointer to the node after which the new node is inserted.
 * @param[in]  data   Pointer to the data to store in the new node.
 * @return     True if the node is inserted, False otherwise.
 ******************************************************************************/
bool dlist_insert_after(DList *list, DListNode *pos, void *data);

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
#endif /* DLIST_H */
