#ifndef QUEUE_H
#define QUEUE_H

/*===========================================================================*/
/**
 * @file queue.h
 *
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * API for the Circular Queue (or ring buffer) implementation
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
typedef struct {
    void **buffer;       /**< Pointer to array of void* */
    size_t capacity;     /**< Maximum number of elements */
    size_t head;         /**< Index of next element to pop */
    size_t tail;         /**< Index of next element to push */
    size_t size;         /**< Current number of elements */
} Queue;

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
 * @fn         cqueue_init
 * @brief      Initializes the circular queue.
 * @param[in]  q       Pointer to the Queue instance.
 * @param[in]  buffer  Pointer to the memory the Queue shall use.
 * @param[in]  capacity Maximum number of elements.
 * @return     true if initialization succeeded, false otherwise.
 ******************************************************************************/
bool queue_init(Queue *q, void **buffer, size_t capacity);

/*****************************************************************************
 * @fn         cqueue_free
 * @brief      Frees the circular queue memory.
 * @param[in]  q      Pointer to the Queue instance.
 * @return     None.
 ******************************************************************************/
void queue_free(Queue *q);

/*****************************************************************************
 * @fn         cqueue_is_empty
 * @brief      Checks if the queue is empty.
 * @param[in]  q      Pointer to the Queue instance.
 * @return     true if empty, false otherwise.
 ******************************************************************************/
bool queue_is_empty(const Queue *q);

/*****************************************************************************
 * @fn         cqueue_is_full
 * @brief      Checks if the queue is full.
 * @param[in]  q      Pointer to the Queue instance.
 * @return     true if full, false otherwise.
 ******************************************************************************/
bool queue_is_full(const Queue *q);

/*****************************************************************************
 * @fn         queue_push
 * @brief      Adds an element to the queue.
 * @param[in]  q      Pointer to the Queue instance.
 * @param[in]  data   Pointer to the data.
 * @return     true if successful, false if full.
 ******************************************************************************/
bool queue_push(Queue *q, void *data);

/*****************************************************************************
 * @fn         queue_pop
 * @brief      Removes an element from the queue.
 * @param[in]  q      Pointer to the Queue instance.
 * @return     Pointer to data, or NULL if empty.
 ******************************************************************************/
void *queue_pop(Queue *q);

/*****************************************************************************
 * @fn         queue_peek
 * @brief      Returns the element at head without removing it.
 * @param[in]  q      Pointer to the Queue instance.
 * @return     Pointer to data, or NULL if empty.
 ******************************************************************************/
void *queue_peek(const Queue *q);

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
