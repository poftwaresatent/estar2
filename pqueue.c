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

#include <stdlib.h>
#include <err.h>
#include <stdio.h>
#include <math.h>


////#define CALC_KEY(cell) ((cell)->rhs < (cell)->phi ? (cell)->rhs : (cell)->phi)


static void bubble_up (size_t * heap, double * key, size_t * pos, size_t index)
{
  size_t tmp;
  size_t parent;
  parent = index / 2;
  while ((parent > 0) && (key[heap[index]] < key[heap[parent]])) {
    
    // swap
    tmp = heap[index];
    heap[index] = heap[parent];
    heap[parent] = tmp;
    pos[heap[index]] = index;
    pos[heap[parent]] = parent;
    
    // go one level up
    index = parent;
    parent = index / 2;
  }
}


static void bubble_down (size_t * heap, size_t heaplen, double * key, size_t * pos, size_t index)
{
  size_t child, target, tmp;
  
  target = index;
  while (1) {
    
    // check for need to reorder
    child = 2 * index;
    if (child <= heaplen && key[heap[child]] < key[heap[target]]) {
      target = child;
    }
    ++child;
    if (child <= heaplen && key[heap[child]] < key[heap[target]]) {
      target = child;
    }
    if (index == target) {
      // we're done
      break;
    }

    // swap
    tmp = heap[index];
    heap[index] = heap[target];
    heap[target] = tmp;
    pos[heap[index]] = index;
    pos[heap[target]] = target;
    
    // go one level down
    index = target;
  }
}


void pqueue_init (pqueue_t * pq, size_t cap, size_t nelem)
{
  pq->heap = malloc ((sizeof *pq->heap) * (cap+1));
  if (NULL == pq->heap) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc heap", __func__);
  }
  pq->key = malloc ((sizeof *pq->key) * nelem);
  if (NULL == pq->key) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc key", __func__);
  }
  pq->pos = calloc (nelem, sizeof *pq->pos);
  if (NULL == pq->pos) {
    errx (EXIT_FAILURE, __FILE__": %s: calloc pos", __func__);
  }
  pq->len = 0;
  pq->cap = cap;
}


void pqueue_fini (pqueue_t * pq)
{
  free (pq->pos);
  free (pq->key);
  free (pq->heap);
  pq->len = 0;
  pq->cap = 0;
}


double pqueue_topkey (pqueue_t * pq)
{
  if (pq->len > 0) {
    return pq->key[pq->heap[1]];
  }
  return INFINITY;
}


void pqueue_insert_or_update (pqueue_t * pq, size_t elem, double key)
{
  size_t len;
  size_t * heap;
  
  if (0 != pq->pos[elem]) {
    // Could possibly make it more efficient by only bubbling down
    // when the bubble up did not change the position in the heap.
    pq->key[elem] = key;
    bubble_up (pq->heap, pq->key, pq->pos, pq->pos[elem]);
    // Note that pos[elem] may have been changed by bubble_up.
    bubble_down (pq->heap, pq->len, pq->key, pq->pos, pq->pos[elem]);
    return;
  }
  
  // grow heap, realloc if necessary
  
  len = pq->len + 1;
  if (len <= pq->cap) {
    heap = pq->heap;
  }
  else {
    size_t cap;
    cap = 2 * pq->cap;
    heap = realloc (pq->heap, sizeof(*heap) * (cap+1));
    if (NULL == heap) {
      errx (EXIT_FAILURE, __FILE__": %s: realloc", __func__);
    }
    pq->heap = heap;
    pq->cap = cap;
  }
  pq->len = len;
  
  // append elem to heap and bubble up
  
  pq->key[elem] = key;
  heap[len] = elem;
  pq->pos[elem] = len;		/* initialize pos */
  bubble_up (heap, pq->key, pq->pos, len);
}


void pqueue_remove_or_ignore (pqueue_t * pq, size_t elem)
{
  size_t pos;
  pos = pq->pos[elem];
  if (0 == pos) {
    // This could be done by the caller for efficiency, but it is much
    // more convenient to do it here.
    return;
  }
  
  pq->heap[pos] = pq->heap[pq->len];
  pq->pos[pos] = pos;		/* keep pos consistent! */
  --pq->len;
  bubble_down (pq->heap, pq->len, pq->key, pq->pos, pos);
  pq->pos[elem] = 0;		/* mark elem as not on queue */
}


size_t pqueue_extract_or_what (pqueue_t * pq)
{
  size_t elem;
  
  if (0 == pq->len) {
    return (size_t) -1;
  }
  
  elem = pq->heap[1];
  pq->pos[elem] = 0;		/* mark elem as not on queue */
  
  if (1 == pq->len) {
    pq->len = 0;
    // could free the heap here...
    return elem;
  }
  
  pq->heap[1] = pq->heap[pq->len];
  pq->pos[pq->heap[1]] = 1;	/* keep pos consistent */
  --pq->len;
  
  // here would be a good place to shrink the heap
  
  bubble_down (pq->heap, pq->len, pq->key, pq->pos, 1);
  
  return elem;
}


void pqueue_dump (pqueue_t * pq, char const * pfx)
{
  size_t ii;
  for (ii = 1; ii <= pq->len; ++ii) {
    printf ("%selem: %zu  pos:  %zu  key: %g\n",
	    pfx,
	    pq->heap[ii],
	    pq->pos[pq->heap[ii]],
	    pq->key[pq->heap[ii]]);
  }

}
