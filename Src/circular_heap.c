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

#include "main.h"
#include "circular_heap.h"

struct header_t {
#if CONFIG_ALIGNMENT != 8
    size_t size;
#else
    uint64_t size;
#endif
};

int circular_heap_init(struct circular_heap_t *self_p,
                       void *buf_p,
                       size_t size)
{
  if( !(self_p != NULL) ){ Error_Handler(); }
  if( !(buf_p != NULL) ){ Error_Handler(); }
  if( !(size > 0) ){ Error_Handler(); }

  self_p->begin_p = buf_p;
  self_p->end_p = ((char*)buf_p + size);
  self_p->alloc_p = buf_p;
  self_p->free_p = buf_p;

  return (0);
}

void *circular_heap_alloc(struct circular_heap_t *self_p,
                          size_t size)
{
    if( !(self_p != NULL) ){ Error_Handler(); }
    if( !(size > 0) ){ Error_Handler(); }

    struct header_t *header_p;

    header_p = NULL;
    size += sizeof(*header_p);

    /* Align the buffer to a 4 byte boundary. */
#if CONFIG_ALIGNMENT != 0
    size |= CONFIG_ALIGNMENT;
    size &= (-1 & ~(CONFIG_ALIGNMENT - 1));
#else
    size += 3;
    size &= 0xffffffc;
#endif

    /* Does it fit before end_p or free_p? */
    if (self_p->alloc_p >= self_p->free_p)  {
        /* begin_p <-> free_p <-> alloc_p <-> end_p */
        if (((char*)self_p->end_p - (char*)self_p->alloc_p) > size)  {
            header_p = self_p->alloc_p;
            self_p->alloc_p = (char*)self_p->alloc_p + size;
        } else if (((char*)self_p->free_p - (char*)self_p->begin_p) > size) {
            header_p = self_p->begin_p;
            self_p->alloc_p = (char*)self_p->begin_p + size;
        }
    } else {
        /* begin_p <-> alloc_p <-> free_p <-> end_p */
        if (((char*)self_p->free_p - (char*)self_p->alloc_p) > size)  {
            header_p = self_p->alloc_p;
            self_p->alloc_p = (char*)self_p->alloc_p + size;
        }
    }

    if (header_p != NULL) {
        header_p->size = size;

        return (&header_p[1]);
    } else {
        return (NULL);
    }
}

int circular_heap_free(struct circular_heap_t *self_p,
                       void *buf_p)
{
    if( !(self_p != NULL) ){ Error_Handler(); }
    if( !(buf_p != NULL) ){ Error_Handler(); }
    if( !(buf_p > self_p->begin_p) ){ Error_Handler(); }
    if( !(buf_p < self_p->end_p) ){ Error_Handler(); }

    struct header_t *header_p;

    header_p = buf_p;
    header_p--;

    if( !((header_p == self_p->free_p)
            || (header_p == self_p->begin_p)) ){ Error_Handler(); }

    if (header_p == self_p->begin_p) {
        self_p->free_p = self_p->begin_p;
    }

    self_p->free_p = (char*)self_p->free_p + header_p->size;

    return (0);
}

/////////////////////// linked-list related stuff /////////////////////////////

// a new element enters a queue from the backdoor (tail of the queue)
// removal of an element occurs from the front (head of the queue). 
void enqueue( queue *q, item *n ) 
{
  if( q->head == NULL ) 
  {
    q->head = q->tail = n;
    q->head->nexta = q->tail->nexta = NULL;
  }
  else
  {
    q->tail->nexta = n; // link new element to the current tail element
    q->tail = n; // move the tail to the new element
    q->tail->nexta = NULL; // new tail has not following element yet
  }
}

// detaches the head element from the queue and returns its pointer for de-allocation
void *dequeue( queue *q ) 
{
  if( q->head == NULL ) 
  {
    return NULL;
  }
  else
  {
    void *temp = q->head; // save current head for returning BEFORE updating the queue head position
    q->head = q->head->nexta;
    return temp;
  }
}

void *get_queue_head(queue *q) 
{
  if( q->head != NULL )
  {
    return &q->head->payload;
  }
  else
  {
    return NULL;
  }
}