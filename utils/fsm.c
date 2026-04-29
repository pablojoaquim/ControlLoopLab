/*===========================================================================*/
/**
 * @file fsm.c
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 2025 - Pablo Joaquim
 * MIT License: https://opensource.org/licenses/MIT
 *------------------------------------------------------------------------------
 *
 * @section DESC DESCRIPTION:
 * Finite state machine in C (FSM)
 * - Support for entry/exit/do
 * - Support for guards, actions and transitions table
 * - Simple event queue added
 * - C99 style
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
#include <stdlib.h>
#include <string.h>
#include "fsm.h"

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
 * Event queue management functions
 *****************************************************************************/
static void eq_init(fsm_event_queue_t *q)
{
    q->head = q->tail = q->count = 0;
}

static bool eq_is_empty(fsm_event_queue_t *q)
{
    return (0 == q->count);
}

static bool eq_is_full(fsm_event_queue_t *q)
{
    return (FSM_EVENT_QUEUE_SIZE == q->count);
}

static bool eq_push(fsm_event_queue_t *q, fsm_event_t ev)
{
    if (eq_is_full(q))
        return false;
    q->buf[q->tail] = ev;
    q->tail = (q->tail + 1) % FSM_EVENT_QUEUE_SIZE;
    q->count++;
    return true;
}

static bool eq_pop(fsm_event_queue_t *q, fsm_event_t *out)
{
    if (eq_is_empty(q))
        return false;
    *out = q->buf[q->head];
    q->head = (q->head + 1) % FSM_EVENT_QUEUE_SIZE;
    q->count--;
    return true;
}

/*****************************************************************************
 * Helper functions
 *****************************************************************************/
static const fsm_state_t *fsm_find_state(const fsm_ctx_t *ctx, fsm_state_id_t id)
{
    for (size_t i = 0; i < ctx->state_count; ++i)
    {
        if (ctx->states[i].id == id)
            return &ctx->states[i];
    }
    return NULL;
}

static const fsm_transition_t *fsm_find_transition(const fsm_ctx_t *ctx, fsm_state_id_t src, fsm_event_id_t evt)
{
    for (size_t i = 0; i < ctx->transition_count; ++i)
    {
        const fsm_transition_t *t = &ctx->transitions[i];
        if (t->src == src && t->evt == evt)
            return t;
    }
    return NULL;
}

static void fsm_change_state(fsm_ctx_t *ctx, const fsm_state_t *dst)
{
    if (ctx->current == dst)
        return;

    if (ctx->current && ctx->current->on_exit)
        ctx->current->on_exit(ctx);

    ctx->current = dst;

    if (ctx->current && ctx->current->on_entry)
        ctx->current->on_entry(ctx);
}

/*****************************************************************************
 * Name         fsm_change_state
 * Description  Move to the destination state, executing on_exit from current
 *              state and on_entry in the destination state.
 *****************************************************************************/
void fsm_setup(fsm_ctx_t *ctx, const fsm_state_t *states, size_t state_count, const fsm_transition_t *transitions, size_t transition_count)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->states = states;
    ctx->state_count = state_count;
    ctx->transitions = transitions;
    ctx->transition_count = transition_count;
}

/*****************************************************************************
 * Name         fsm_init
 * Description  Move the current state to the initial state and executes its
 *              on_entry function if any.
 *****************************************************************************/
void fsm_init(fsm_ctx_t *ctx, fsm_state_id_t initial_id)
{
    eq_init(&ctx->evq);
    ctx->current = fsm_find_state(ctx, initial_id);
    if (ctx->current && ctx->current->on_entry)
        ctx->current->on_entry(ctx);
}

/*****************************************************************************
 * Name         fsm_post_event
 * Description  Execute the on_do() of the current state (if any)
 *****************************************************************************/
bool fsm_post_event(fsm_ctx_t *ctx, fsm_event_id_t id, void *data)
{
    fsm_event_t ev = {.id = id, .data = data};
    return eq_push(&ctx->evq, ev);
}

/*****************************************************************************
 * Name         fsm_process_events
 * Description  Process the received events (till 'max' events)
 *****************************************************************************/
void fsm_process_events(fsm_ctx_t *ctx, int max)
{
    fsm_event_t ev;
    int count = 0;

    while (count < max && eq_pop(&ctx->evq, &ev))
    {
        if (ctx->current)
        {
            // If the current state have a transition related to the received
            // event, the guard is checked, the action related is taken, and
            // then moved to the destination state
            const fsm_transition_t *t = fsm_find_transition(ctx, ctx->current->id, ev.id);
            if (t)
            {
                bool guard_ok = true;
                if (t->guard)
                    guard_ok = t->guard(ctx, ev.data);
                if (guard_ok)
                {
                    if (t->action)
                        t->action(ctx, ev.data);
                    const fsm_state_t *dst = fsm_find_state(ctx, t->dst);
                    if (dst)
                        fsm_change_state(ctx, dst);
                }
            }
        }
        count++;
    }
}

/*****************************************************************************
 * Name         fsm_run_do_action
 * Description  Execute the on_do() of the current state (if any)
 *****************************************************************************/
void fsm_run_do_action(fsm_ctx_t *ctx)
{
    if (ctx->current && ctx->current->on_do)
        ctx->current->on_do(ctx);
}
