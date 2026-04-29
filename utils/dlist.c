/*===========================================================================*/
/**
 * @file dlist.c
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Double Linked Lists implementation
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
#include "dlist.h"

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
 * Name         dlist_init
 * Description  Initialize the list structure, leaving it empty.
 *****************************************************************************/
bool dlist_init(DList *list, DListNode *pool, size_t capacity)
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
 * Name         dlist_is_empty
 * Description  Returns true if the doubly-linked list has no elements.
 *****************************************************************************/
bool dlist_is_empty(const DList *list)
{
    return list->size == 0;
}

/*****************************************************************************
 * Name         dlist_size
 * Description  Returns the number of elements currently stored in the list.
 *****************************************************************************/
size_t dlist_size(const DList *list)
{
    return list->size;
}

/*****************************************************************************
 * Name         alloc_node
 * Description  Request a node from the pool
 *****************************************************************************/
static DListNode *alloc_node(DList *list)
{
    if (!list->free_list)
        return NULL;

    DListNode *node = list->free_list;
    list->free_list = node->next;

    return node;
}

/*****************************************************************************
 * Name         free_node
 * Description  Returns a node to the pool
 *****************************************************************************/
static void free_node(DList *list, DListNode *node)
{
    node->next = list->free_list;
    list->free_list = node;
}

/*****************************************************************************
 * Name         dlist_foreach
 * Description  Iterate through all nodes in the list and call the provided
 *              callback function for each element.
 *****************************************************************************/
void dlist_foreach(DList *list, dlist_iter_fn fn)
{
    if (!list || !fn)
        return;

    DListNode *cur = list->head;
    while (cur)
    {
        fn(cur->data);
        cur = cur->next;
    }
}

/*****************************************************************************
 * Name         dlist_push_head
 * Description  Insert a new element at the beginning of the list.
 *****************************************************************************/
bool dlist_push_head(DList *list, void *data)
{
    DListNode *node = alloc_node(list);
    if (!node)
        return false;

    node->data = data;
    node->prev = NULL;
    node->next = list->head;

    if (list->head)
        list->head->prev = node;
    else
        list->tail = node;

    list->head = node;
    list->size++;
    return true;
}

/*****************************************************************************
 * Name         dlist_push_tail
 * Description  Insert a new element at the end of the list.
 *****************************************************************************/
bool dlist_push_tail(DList *list, void *data)
{
    DListNode *node = alloc_node(list);
    if (!node)
        return false;    
    node->data = data;
    node->next = NULL;
    node->prev = list->tail;

    if (list->tail)
        list->tail->next = node;
    else
        list->head = node;

    list->tail = node;
    list->size++;
    return true;
}

/*****************************************************************************
 * Name         dlist_pop_head
 * Description  Remove and return the element at the beginning of the list.
 *              Returns NULL if the list is empty.
 *****************************************************************************/
void *dlist_pop_head(DList *list)
{
    if (!list->head)
        return NULL;

    DListNode *node = list->head;
    void *data = node->data;

    list->head = node->next;
    if (list->head)
        list->head->prev = NULL;
    else
        list->tail = NULL;

    free_node(list, node);
    list->size--;
    return data;
}

/*****************************************************************************
 * Name         dlist_pop_tail
 * Description  Remove and return the element at the end of the list.
 *              Returns NULL if the list is empty.
 *****************************************************************************/
void *dlist_pop_tail(DList *list)
{
    if (!list->tail)
        return NULL;

    DListNode *node = list->tail;
    void *data = node->data;

    list->tail = node->prev;
    if (list->tail)
        list->tail->next = NULL;
    else
        list->head = NULL;

    free_node(list, node);
    list->size--;
    return data;
}

/*****************************************************************************
 * Name         dlist_contains
 * Description  Check if the list contains an element matching the criteria
 *              using the provided comparison callback.
 *              Returns true if at least one match is found.
 *****************************************************************************/
bool dlist_contains(const DList *list, const void *criteria, dlist_match_fn match)
{
    return dlist_find(list, criteria, match) != NULL;
}

/*****************************************************************************
 * Name         dlist_find
 * Description  Search the list for the first node whose data matches the
 *              given criteria using the provided comparison callback.
 *              Returns the matching data pointer, or NULL if no match exists.
 *****************************************************************************/
void *dlist_find(const DList *list, const void *criteria, dlist_match_fn match)
{
    DListNode *cur = list->head;
    while (cur)
    {
        if (match(cur->data, criteria))
            return cur->data;
        cur = cur->next;
    }
    return NULL;
}

/*****************************************************************************
 * Name         dlist_remove
 * Description  Remove the first node whose data matches the given criteria
 *              using the provided comparison callback.
 *              Returns true if a node was found and removed.
 *****************************************************************************/
bool dlist_remove(DList *list, const void *criteria, dlist_match_fn match, dlist_free_fn free_fn)
{
    DListNode *cur = list->head;
    while (cur)
    {
        if (match(cur->data, criteria))
        {
            if (cur->prev)
                cur->prev->next = cur->next;
            else
                list->head = cur->next;

            if (cur->next)
                cur->next->prev = cur->prev;
            else
                list->tail = cur->prev;

            if (free_fn)
                free_fn(cur->data);
            free_node(list, cur);
            list->size--;
            return true;
        }
        cur = cur->next;
    }
    return false;
}

/*****************************************************************************
 * Name         dlist_clear
 * Description  Remove all nodes from the list. If free_fn is provided, it is
 *              called for each node's data. List is reinitialized afterwards.
 *****************************************************************************/
void dlist_clear(DList *list, dlist_free_fn free_fn)
{
    DListNode *cur = list->head;
    while (cur)
    {
        DListNode *next = cur->next;
        if (free_fn)
            free_fn(cur->data);
        free_node(list, cur);
        cur = next;
    }
    list->head = list->tail = NULL;
    list->size = 0;
}

/*****************************************************************************
 * Name         dlist_insert_before
 * Description  Insert a new node before the given position node. If pos is NULL,
 *              the element is inserted at the head of the list.
 *****************************************************************************/
bool dlist_insert_before(DList *list, DListNode *pos, void *data)
{
    if (!pos)
    {
        return dlist_push_head(list, data);
    }
    DListNode *node = alloc_node(list);
    if (!node)
        return false;

    node->data = data;
    node->prev = pos->prev;
    node->next = pos;

    if (pos->prev)
        pos->prev->next = node;
    else
        list->head = node;

    pos->prev = node;
    list->size++;
    return true;
}

/*****************************************************************************
 * Name         dlist_insert_after
 * Description  Insert a new node after the given position node. If pos is NULL,
 *              the element is inserted at the tail of the list.
 *****************************************************************************/
bool dlist_insert_after(DList *list, DListNode *pos, void *data)
{
    if (!pos)
    {
        return dlist_push_tail(list, data);
    }
    DListNode *node = alloc_node(list);
    if (!node)
        return false;

    node->data = data;
    node->next = pos->next;
    node->prev = pos;

    if (pos->next)
        pos->next->prev = node;
    else
        list->tail = node;

    pos->next = node;
    list->size++;
    return true;
}