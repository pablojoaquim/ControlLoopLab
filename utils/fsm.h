#ifndef FSM_H
#define FSM_H

/*===========================================================================*/
/**
 * @file fsm.h
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * API for the Finite state machine (FSM)
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
#define FSM_EVENT_QUEUE_SIZE 32

/*===========================================================================*
 * Exported Preprocessor #define MACROS
 *===========================================================================*/

/*===========================================================================*
 * Exported Type Declarations
 *===========================================================================*/
typedef int fsm_state_id_t;
typedef int fsm_event_id_t;

typedef struct fsm_event
{
    fsm_event_id_t id;
    void *data; /* data pointer (NULL is NA) */
} fsm_event_t;

typedef struct fsm_event_queue
{
    fsm_event_t buf[FSM_EVENT_QUEUE_SIZE];
    int head;
    int tail;
    int count;
} fsm_event_queue_t;

/* ---- Forward declaration to break the circular dependency ---- */
typedef struct fsm_ctx fsm_ctx_t;

/* Function prototypes for state/transitions */
typedef void (*state_fn_t)(fsm_ctx_t *ctx);
typedef bool (*guard_fn_t)(fsm_ctx_t *ctx, void *event_data);
typedef void (*action_fn_t)(fsm_ctx_t *ctx, void *event_data);

/* State management structure */
typedef struct fsm_state
{
    fsm_state_id_t id; /* id */
    const char *name;  /* name (mainly for debug) */

    state_fn_t on_entry; /* on entry */
    state_fn_t on_exit;  /* on exit */
    state_fn_t on_do;    /* do it while keep in the state */
} fsm_state_t;

/* Transition management structure */
typedef struct fsm_transition
{
    fsm_state_id_t src; /* source state */
    fsm_event_id_t evt; /* event trigger */
    guard_fn_t guard;   /* guard (NULL if NA) */
    action_fn_t action; /* action to execute during transition (NULL if NA) */
    fsm_state_id_t dst; /* destination state (same as src for loops) */
} fsm_transition_t;

struct fsm_ctx
{
    const fsm_state_t *current;
    const fsm_state_t *states; /* states array */
    size_t state_count;

    const fsm_transition_t *transitions; /* transitions array */
    size_t transition_count;

    fsm_event_queue_t evq; /* ev queue */
};

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
extern "C" {
#endif
// @todo: Add pure C function prototypes here.
/*****************************************************************************
 * @fn         fsm_init
 * @brief      Initializes the Finite State Machine (FSM) context with
 *             a given initial state.
 * @param [in] ctx         Pointer to the FSM context instance.
 * @param [in] initial_id  Identifier of the initial state.
 * @return     None
 ******************************************************************************/
void fsm_init(fsm_ctx_t *ctx, fsm_state_id_t initial_id);

/*****************************************************************************
 * @fn         fsm_post_event
 * @brief      Posts an event into the FSM event queue.
 *             The event will later be consumed by fsm_process_events().
 * @param [in]  ctx   Pointer to the FSM context instance.
 * @param [in]  id    Event identifier.
 * @param [in]  data  Optional user data associated with the event.
 * @return      true if the event was successfully queued, false otherwise.
 ******************************************************************************/
bool fsm_post_event(fsm_ctx_t *ctx, fsm_event_id_t id, void *data);

/*****************************************************************************
 * @fn         fsm_process_events
 * @brief      Processes pending events in the FSM queue.
 *             For each event processed, transitions and actions are executed.
 * @param [in] ctx   Pointer to the FSM context instance.
 * @param [in] max   Maximum number of events to process in this call.
 * @return     None
 ******************************************************************************/
void fsm_process_events(fsm_ctx_t *ctx, int max);

/*****************************************************************************
 * @fn         fsm_run_do_action
 * @brief      Executes the “do” action of the current state if defined.
 *             Typically called periodically in the system main loop.
 * @param [in] ctx   Pointer to the FSM context instance.
 * @return     None
 ******************************************************************************/
void fsm_run_do_action(fsm_ctx_t *ctx);

/*****************************************************************************
 * @fn         fsm_setup
 * @brief      Configures the FSM with its state table and transition table.
 *             This function must be called before using any FSM operations.
 * @param [in] ctx               Pointer to the FSM context instance.
 * @param [in] states            Array of FSM state descriptors.
 * @param [in] state_count       Number of elements in the states array.
 * @param [in] transitions       Array of FSM transition descriptors.
 * @param [in] transition_count  Number of elements in the transitions array.
 * @return     None
 ******************************************************************************/
void fsm_setup(fsm_ctx_t *ctx, const fsm_state_t *states, size_t state_count, const fsm_transition_t *transitions, size_t transition_count);

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
#endif /* FSM_H */
