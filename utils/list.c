/*===========================================================================*/
/**
 * @file list.c
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Simple Linked Lists implementation
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
#include <stdlib.h>
#include "list.h"

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
 * Name         list_init
 * Description  Initialize the list structure, leaving it empty.
 *****************************************************************************/
bool list_init(List *list, ListNode *pool, size_t capacity)
{
    if (!list || !pool || capacity == 0)
        return false;
    list->head = NULL;
    list->tail = NULL;
    list->size = 0;

    list->pool = pool;
    list->pool_capacity = capacity;

    /* Init the free list */
    list->free_list = &pool[0];
    for (size_t i = 0; i < capacity - 1; i++)
        pool[i].next = &pool[i + 1];
    pool[capacity - 1].next = NULL;

    return true;
}

/*****************************************************************************
 * Name         list_is_empty
 * Description  Returns true if the list has no elements.
 *****************************************************************************/
bool list_is_empty(const List *list)
{
    return list->size == 0;
}

/*****************************************************************************
 * Name         list_size
 * Description  Returns the number of elements currently stored in the list.
 *****************************************************************************/
size_t list_size(const List *list)
{
    return list->size;
}

/*****************************************************************************
 * Name         alloc_node
 * Description  Request a node from the pool
 *****************************************************************************/
static ListNode *alloc_node(List *list)
{
    if (!list->free_list)
        return NULL;  // no more nodes free

    ListNode *node = list->free_list;
    list->free_list = node->next;
    return node;
}

/*****************************************************************************
 * Name         free_node
 * Description  Returns a node to the pool
 *****************************************************************************/
static void free_node(List *list, ListNode *node)
{
    node->next = list->free_list;
    list->free_list = node;
}

/*****************************************************************************
 * Name         list_push_head
 * Description  Insert a new element at the beginning of the list.
 *****************************************************************************/
bool list_push_head(List *list, void *data)
{
    ListNode *node = alloc_node(list);
    if (!node)
        return false;  // Pool is full
    node->data = data;
    node->next = list->head;
    list->head = node;
    if (!list->tail)
        list->tail = node;
    list->size++;
    return true;
}

/*****************************************************************************
 * Name         list_push_tail
 * Description  Insert a new element at the end of the list.
 *****************************************************************************/
bool list_push_tail(List *list, void *data)
{
    ListNode *node = alloc_node(list);
    if (!node)
        return false;   // Pool is full
    node->data = data;
    node->next = NULL;

    if (list->tail)
        list->tail->next = node;
    else
        list->head = node;

    list->tail = node;
    list->size++;

    return true;
}

/*****************************************************************************
 * Name         list_pop_head
 * Description  Remove and return the element at the beginning of the list.
 *              Returns NULL if the list is empty.
 *****************************************************************************/
void *list_pop_head(List *list)
{
    if (!list->head)
        return NULL;

    ListNode *node = list->head;
    void *data = node->data;

    list->head = node->next;
    if (!list->head)
        list->tail = NULL;

    free_node(list, node);
    list->size--;
    return data;
}

/*****************************************************************************
 * Name         list_pop_tail
 * Description  Remove and return the element at the end of the list.
 *              Returns NULL if the list is empty.
 *****************************************************************************/
void *list_pop_tail(List *list)
{
    if (!list->tail)
        return NULL;

    ListNode *cur = list->head;
    ListNode *prev = NULL;

    while (cur->next)
    {
        prev = cur;
        cur = cur->next;
    }

    void *data = cur->data;

    if (prev)
        prev->next = NULL;
    else
        list->head = NULL;

    list->tail = prev;

    free_node(list, cur);
    list->size--;
    return data;
}

/*****************************************************************************
 * Name         list_contains
 * Description  Check if the list contains an element matching the criteria
 *              using the provided comparison callback.
 *              Returns true if at least one match is found.
 *****************************************************************************/
bool list_contains(const List *list, const void *criteria, list_match_fn match)
{
    return list_find(list, criteria, match) != NULL;
}

/*****************************************************************************
 * @fn         list_find
 * @brief      Searches the list for the first node whose data matches the
 *             provided criteria using the comparison callback.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  criteria  Pointer to the user-defined criteria to match.
 * @param[in]  match     Function used to compare node data against criteria.
 * @return     Pointer to the matching data if found, NULL otherwise.
 ******************************************************************************/
void *list_find(const List *list, const void *criteria, list_match_fn match)
{
    ListNode *cur = list->head;
    while (cur)
    {
        if (match(cur->data, criteria))
            return cur->data;
        cur = cur->next;
    }
    return NULL;
}

/*****************************************************************************
 * @fn         list_remove
 * @brief      Removes the first node whose data matches the provided criteria
 *             using the comparison callback. Optionally frees node data using
 *             free_fn.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  criteria  Pointer to the user-defined criteria to match.
 * @param[in]  match     Function used to determine if node data matches criteria.
 * @param[in]  free_fn   Function used to free node data (may be NULL).
 * @return     true if a node was removed, false otherwise.
 ******************************************************************************/
bool list_remove(List *list, const void *criteria, list_match_fn match, list_free_fn free_fn)
{
    ListNode *cur = list->head;
    ListNode *prev = NULL;

    while (cur)
    {
        if (match(cur->data, criteria))
        {
            if (prev)
                prev->next = cur->next;
            else
                list->head = cur->next;

            if (cur == list->tail)
                list->tail = prev;

            if (free_fn)
                free_fn(cur->data);
            free_node(list, cur);
            list->size--;
            return true;
        }
        prev = cur;
        cur = cur->next;
    }
    return false;
}

/*****************************************************************************
 * @fn         list_clear
 * @brief      Removes all nodes from the list. If free_fn is provided, it is
 *             called for each node's data. The list is reinitialized afterwards.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  free_fn   Function used to free node data (may be NULL).
 * @return     None.
 ******************************************************************************/
void list_clear(List *list, list_free_fn free_fn)
{
    ListNode *cur = list->head;
    while (cur)
    {
        ListNode *next = cur->next;
        if (free_fn)
            free_fn(cur->data);
        free_node(list, cur);
        cur = next;
    }
    list->head = list->tail = NULL;
    list->size = 0;
}

/*****************************************************************************
 * @fn         list_foreach
 * @brief      Iterates over all elements stored in the list and invokes the
 *             supplied callback for each node's data.
 * @param[in]  list      Pointer to the List instance.
 * @param[in]  fn        Function to be executed for each node; receives node data.
 * @return     None.
 ******************************************************************************/
void list_foreach(List *list, list_iter_fn fn)
{
    if (!list || !fn)
        return;

    ListNode *cur = list->head;
    while (cur)
    {
        fn(cur->data);
        cur = cur->next;
    }
}
