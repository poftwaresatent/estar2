/* Minimal Estar implementation for 2D grid with LSM kernel.
 *
 * Copyright (C) 2013 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pqueue.h"
#include "cell.h"

#include <stdlib.h>
#include <err.h>
#include <stdio.h>
#include <math.h>


#define CALC_KEY(cell) ((cell)->rhs < (cell)->phi ? (cell)->rhs : (cell)->phi)


static void swap (cell_t ** aa, cell_t ** bb)
{
  size_t ti;
  cell_t *tc;
  ti = (*aa)->pqi;
  (*aa)->pqi = (*bb)->pqi;
  (*bb)->pqi = ti;
  tc = (*aa);
  (*aa) = (*bb);
  (*bb) = tc;
}


static void bubble_up (cell_t ** heap, size_t index)
{
  size_t parent;
  parent = index / 2;
  while ((parent > 0) && (heap[index]->key < heap[parent]->key)) {
    swap (&heap[index], &heap[parent]);
    index = parent;
    parent = index / 2;
  }
}


static void bubble_down (cell_t ** heap, size_t len, size_t index)
{
  size_t child, target;
  
  target = index;
  while (1) {
    child = 2 * index;
    if (child <= len && heap[child]->key < heap[target]->key) {
      target = child;
    }
    ++child;
    if (child <= len && heap[child]->key < heap[target]->key) {
      target = child;
    }
    if (index == target) {
      break;
    }
    swap (&heap[target], &heap[index]);
    index = target;
  }
}


void pqueue_init (pqueue_t * pq, size_t cap)
{
  pq->heap = malloc (sizeof(cell_t*) * (cap+1));
  if (NULL == pq->heap) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc", __func__);
  }
  pq->len = 0;
  pq->cap = cap;
}


void pqueue_fini (pqueue_t * pq)
{
  free (pq->heap);
  pq->len = 0;
  pq->cap = 0;
}


double pqueue_topkey (pqueue_t * pq)
{
  if (pq->len > 0) {
    return pq->heap[1]->key;
  }
  return INFINITY;
}


void pqueue_insert (pqueue_t * pq, cell_t * cell)
{
  size_t len;
  cell_t ** heap;
  
  // grow heap, realloc if necessary
  
  len = pq->len + 1;
  if (len <= pq->cap) {
    heap = pq->heap;
  }
  else {
    size_t cap;
    cap = 2 * pq->cap;
    heap = realloc (pq->heap, sizeof(cell_t*) * (cap+1));
    if (NULL == heap) {
      errx (EXIT_FAILURE, __FILE__": %s: realloc", __func__);
    }
    pq->heap = heap;
    pq->cap = cap;
  }
  pq->len = len;
  
  // append cell to heap and bubble up
  
  cell->key = CALC_KEY(cell);
  heap[len] = cell;
  cell->pqi = len;		/* initialize pqi */
  bubble_up (heap, len);
}


void pqueue_remove (pqueue_t * pq, cell_t * cell)
{
  pq->heap[cell->pqi] = pq->heap[pq->len];
  pq->heap[cell->pqi]->pqi = cell->pqi; /* keep pqi consistent! */
  --pq->len;
  bubble_down (pq->heap, pq->len, cell->pqi);
  cell->pqi = 0;		/* mark cell as not on queue */
}


void pqueue_update (pqueue_t * pq, cell_t * cell)
{
  cell->key = CALC_KEY(cell);
  
  // could probably make it more efficient by only bubbling down when
  // the bubble up did not change cell->pqi
  bubble_up (pq->heap, cell->pqi);
  bubble_down (pq->heap, pq->len, cell->pqi);
}


cell_t * pqueue_extract (pqueue_t * pq)
{
  cell_t * cell;
  
  if (0 == pq->len) {
    return NULL;
  }
  
  cell = pq->heap[1];
  cell->pqi = 0;		/* mark cell as not on queue */
  
  if (1 == pq->len) {
    pq->len = 0;
    return cell;
  }
  
  pq->heap[1] = pq->heap[pq->len];
  pq->heap[1]->pqi = 1;		/* keep pqi consistent */
  --pq->len;
  // here would be a good place to shrink the heap
  
  bubble_down (pq->heap, pq->len, 1);
  
  return cell;
}
