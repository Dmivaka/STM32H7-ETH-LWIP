/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2018, Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * This file is part of the Simba project.
 */

#ifndef __ALLOC_CIRCULAR_HEAP_H__
#define __ALLOC_CIRCULAR_HEAP_H__

#include <stddef.h>

/* Circular_Heap. */
typedef struct circular_heap_t {
    void *begin_p;
    void *end_p;
    void *alloc_p;
    void *free_p;
}circular_heap_t;

/**
 * Initialize given circular heap. Buffers must be freed in the same
 * order as they were allocated.
 *
 * @param[in] self_p Circular heap to initialize.
 * @param[in] buf_p Memory buffer to use for the circular heap.
 * @param[in] size Size of the memory buffer.
 *
 * @return zero(0) or negative error code.
 */
int circular_heap_init(struct circular_heap_t *self_p,
                       void *buf_p,
                       size_t size);

/**
 * Allocate a buffer of given size from given circular heap.
 *
 * @param[in] self_p Circular heap to allocate from.
 * @param[in] size Number of bytes to allocate.
 *
 * @return Pointer to allocated buffer, or NULL if no memory could be
 *         allocated.
 */
void *circular_heap_alloc(struct circular_heap_t *self_p,
                          size_t size);

/**
 * Free the oldest allocated buffer, previously allocated with
 * ``circular_heap_alloc()``.
 *
 * @param[in] self_p Circular heap to free to.
 * @param[in] buf_p Buffer to free. Must be the oldest allocated
 *                  buffer.
 *
 * @return zero(0) or negative error code.
 */
int circular_heap_free(struct circular_heap_t *self_p,
                       void *buf_p);

/////////////////////// linked-list related stuff /////////////////////////////

typedef struct item
{
  struct item *nexta; // this is pointer to the next item - all the voltbro stuff + pointer
  void *payload; // this is my voltbro buffer! it contains all stuff like bus-num, length and data
}item;

typedef struct queue
{
  item *head;
  item *tail;
}queue;

void enqueue( queue *q, item *n );
void *dequeue( queue *q );
void *get_queue_head(queue *q);

#endif
