/*===========================================================================*/
/**
 * @file queue.c
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Circular Queue (or ring buffer) implementation
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
#include <stdio.h>
#include "queue.h"

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
 * Name         queue_init
 * Description  Initializes the circular queue.
 *****************************************************************************/
bool queue_init(Queue *q, void **buffer, size_t capacity)
{
    if (!q || capacity == 0)
        return false;
    q->buffer = buffer;
    q->capacity = capacity;
    q->head = 0;
    q->tail = 0;
    q->size = 0;
    return true;
}

/*****************************************************************************
 * Name         queue_free
 * Description  Frees the circular queue and its buffer.
 *****************************************************************************/
void queue_free(Queue *q)
{
    if (!q)
        return;

    q->buffer = NULL;
    q->capacity = 0;
    q->head = 0;
    q->tail = 0;
    q->size = 0;
}

/*****************************************************************************
 * Name         queue_is_empty
 * Description  Returns true if the queue is empty.
 *****************************************************************************/
bool queue_is_empty(const Queue *q)
{
    return q && q->size == 0;
}

/*****************************************************************************
 * Name         queue_is_full
 * Description  Returns true if the queue is full.
 *****************************************************************************/
bool queue_is_full(const Queue *q)
{
    return q && q->size == q->capacity;
}

/*****************************************************************************
 * Name         queue_push
 * Description  Adds an element to the queue (at tail).
 *****************************************************************************/
bool queue_push(Queue *q, void *data)
{
    if (!q || queue_is_full(q))
        return false;

    q->buffer[q->tail] = data;
    q->tail = (q->tail + 1) % q->capacity;
    q->size++;
    return true;
}

/*****************************************************************************
 * Name         queue_pop
 * Description  Removes and returns element from head.
 *****************************************************************************/
void *queue_pop(Queue *q)
{
    if (!q || queue_is_empty(q))
        return NULL;

    void *data = q->buffer[q->head];
    q->head = (q->head + 1) % q->capacity;
    q->size--;
    return data;
}

/*****************************************************************************
 * Name         queue_peek
 * Description  Returns element at head without removing.
 *****************************************************************************/
void *queue_peek(const Queue *q)
{
    if (!q || queue_is_empty(q))
        return NULL;
    return q->buffer[q->head];
}
