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

#include "estar.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <err.h>


static void estar_update (estar_t * estar, size_t elem);


/**
   \pre primary <= secondary
*/
static double interpolate (double cost, double primary, double secondary)
{
  double tmp;
  
  if (cost <= secondary - primary) {
    return primary + cost;
  }
  
  // pow(cost,2) could be cached inside estar_set_speed. And so could
  // the other squared terms. That might speed things up, but it would
  // certainly make hairier caching code.
  
  tmp = primary + secondary;
  return (tmp + sqrt(pow(tmp, 2.0)
		     - 2.0 * (pow(primary, 2.0)
			      + pow(secondary, 2.0)
			      - pow(cost, 2.0)))) / 2.0;
}


static void calc_rhs (estar_t * estar, size_t elem, double phimax)
{
  size_t * prop;
  size_t primary, secondary;
  double rr;
  
  estar->rhs[elem] = INFINITY;
  
  //////////////////////////////////////////////////
  // First try all the true interpolations. If all of those fail, a
  // fallback solution is computed after this big while loop.
  
  prop = estar->grid.cell[elem].prop;
  while ((size_t) -1 != *prop) {
    
    primary = *(prop++);
    secondary = *(prop++);
    
    //////////////////////////////////////////////////
    // Filter both the primary and the secondary: do not propagate
    // from obstacles, queued cells, cells above the wavefront, or
    // cells at infinity.
    
    if (estar->flags[primary] & ESTAR_FLAG_OBSTACLE
	|| estar->pq.pos[primary] != 0
	|| estar->phi[primary] > phimax
	|| isinf(estar->phi[primary])) {
      continue;
    }
    
    if (estar->flags[secondary] & ESTAR_FLAG_OBSTACLE
	|| estar->pq.pos[secondary] != 0
	|| estar->phi[secondary] > phimax
	|| isinf(estar->phi[secondary])) {
      continue;
    }
    
    //////////////////////////////////////////////////
    // Make sure that the primary lies below the secondary,
    // interpolate between them, and track the mimimum result.
    
    if (estar->phi[primary] < estar->phi[secondary]) {
      rr = interpolate (estar->cost[elem], estar->phi[primary], estar->phi[secondary]);
    }
    else {
      rr = interpolate (estar->cost[elem], estar->phi[secondary], estar->phi[primary]);
    }
    
    if (rr < estar->rhs[elem]) {
      estar->rhs[elem] = rr;
    }
  }

  //////////////////////////////////////////////////
  // If none of the above worked, try the fallback solution (without
  // interpolation).
  
  if (isinf (estar->rhs[elem])) {
    for (prop = estar->grid.cell[elem].prop; (size_t) -1 != *prop; ++prop) {
      if (estar->flags[*prop] & ESTAR_FLAG_OBSTACLE
	  || estar->pq.pos[*prop] != 0
	  || estar->phi[*prop] > phimax
	  || isinf(estar->phi[*prop])) {
	continue;
      }
      if (estar->phi[*prop] < estar->rhs[elem]) {
	estar->rhs[elem] = estar->phi[*prop];
      }
    }
    estar->rhs[elem] += estar->cost[elem];
  }
}


void estar_init (estar_t * estar, size_t dimx, size_t dimy, estar_hfunc_t hfunc)
{
  size_t ii;
  
  grid_init (&estar->grid, dimx, dimy);
  pqueue_init (&estar->pq, dimx + dimy, estar->grid.nelem);
  pqueue_init (&estar->pruned, dimx + dimy, estar->grid.nelem);
  
  estar->cost = malloc ((sizeof *estar->cost) * estar->grid.nelem);
  if (NULL == estar->cost) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc cost", __func__);
  }
  estar->phi = malloc ((sizeof *estar->phi) * estar->grid.nelem);
  if (NULL == estar->phi) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc phi", __func__);
  }
  estar->rhs = malloc ((sizeof *estar->rhs) * estar->grid.nelem);
  if (NULL == estar->rhs) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc rhs", __func__);
  }
  estar->flags = malloc ((sizeof *estar->flags) * estar->grid.nelem);
  if (NULL == estar->flags) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc flags", __func__);
  }
  
  for (ii = 0; ii < estar->grid.nelem; ++ii) {
    estar->cost[ii] = 1.0;
    estar->phi[ii] = INFINITY;
    estar->rhs[ii] = INFINITY;
    estar->flags[ii] = 0;
  }
  
  estar->hfunc = hfunc;
  estar->ubound = INFINITY;
}


void estar_fini (estar_t * estar)
{
  free(estar->flags);
  free(estar->rhs);
  free(estar->phi);
  free(estar->cost);
  grid_fini (&estar->grid);
  pqueue_fini (&estar->pq);
  pqueue_fini (&estar->pruned);
}


void estar_set_goal (estar_t * estar, size_t ix, size_t iy, double ubound)
{
  size_t ii, goal;
  
  /* re-initialize without touching cost and obstacle information */
  for (ii = 0; ii < estar->grid.nelem; ++ii) {
    estar->phi[ii] = INFINITY;
    estar->rhs[ii] = INFINITY;
    estar->flags[ii] &= ~ESTAR_FLAG_GOAL;
  }
  pqueue_reset (&estar->pq);
  pqueue_reset (&estar->pruned);
  
  /* set the goal and initialize the queue */
  goal = grid_elem (&estar->grid, ix, iy);
  estar->rhs[goal] = 0.0;
  estar->flags[goal] |= ESTAR_FLAG_GOAL;
  estar->flags[goal] &= ~ESTAR_FLAG_OBSTACLE;
  pqueue_insert (&estar->pq, goal, 0.0);
  estar->ubound = ubound;
}


void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed)
{
  double cost;
  size_t elem;
  size_t * nbor;
  
  elem = grid_elem (&estar->grid, ix, iy);
  
  if (speed <= 0.0) {
    cost = INFINITY;
  }
  else {
    cost = 1.0 / speed;
  }
  if (cost == estar->cost[elem]) {
    return;
  }
  
  estar->cost[elem] = cost;
  if (speed <= 0.0) {
    estar->phi[elem] = INFINITY;
    estar->rhs[elem] = INFINITY;
    estar->flags[elem] |= ESTAR_FLAG_OBSTACLE;
  }
  else {
    estar->flags[elem] &= ~ESTAR_FLAG_OBSTACLE;
  }
  
  estar_update (estar, elem);
  for (nbor = estar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
    estar_update (estar, *nbor);
  }
}


static void estar_update (estar_t * estar, size_t elem)
{
  if (estar->flags[elem] & ESTAR_FLAG_OBSTACLE) {
    pqueue_remove (&estar->pq, elem);
    return;
  }
  
  /* Make sure that goal cells remain at their rhs, which is supposed
     to be fixed and only serve as source for propagation, never as
     sink. */
  if ( ! (estar->flags[elem] & ESTAR_FLAG_GOAL)) {
    calc_rhs (estar, elem, pqueue_topkey (&estar->pq));
  }
  
  if (estar->phi[elem] != estar->rhs[elem]) {
    /* Use either rhs or phi as queue key, whichever is smaller. */
    if (estar->rhs[elem] < estar->phi[elem]) {
      pqueue_insert (&estar->pq, elem, estar->rhs[elem]);
    }
    else {
      pqueue_insert (&estar->pq, elem, estar->phi[elem]);
    }
  }
  else {
    pqueue_remove (&estar->pq, elem);
  }
}


void estar_propagate (estar_t * estar)
{
  size_t elem;
  size_t * nbor;
  double hval;
  
  //////////////////////////////////////////////////
  // are there any pruned elements that need to be resurrected?
  
  if (pqueue_topkey (&estar->pruned) < estar->ubound) {
    
    // Later, when the start element can change during operation,
    // we'll need to check whether the node can be re-pruned right
    // here,
    
    elem = pqueue_extract (&estar->pruned);
    estar->phi[elem] = INFINITY;
    pqueue_insert (&estar->pq, elem, estar->rhs[elem]);
    
    // Normally we could just run this whole block as a while loop,
    // but I like to see what's happening at each step with the
    // boundary elements, so return (for plotting).
    
    return;
  }
  
  //////////////////////////////////////////////////
  // the rest remains the same execept for pruning cells
  
  elem = pqueue_extract (&estar->pq);
  if ((size_t) -1 == elem) {
    return;
  }
  
  printf ("estar_propagate [%4zu %4zu]\n",
	  grid_ix (&estar->grid, elem), grid_iy (&estar->grid, elem));
  
  if (estar->phi[elem] > estar->rhs[elem]) {

    printf ("  lower %g to %g (delta: %g)\n",
	    estar->phi[elem], estar->rhs[elem], estar->phi[elem] - estar->rhs[elem]);

    estar->phi[elem] = estar->rhs[elem];
    
    hval = estar->hfunc (elem);
    if (estar->rhs[elem] + hval > estar->ubound) {
      
      printf ("  prune %g + %g = %g\n",
	      estar->rhs[elem], hval, estar->rhs[elem] + hval);
      
      pqueue_insert (&estar->pruned, elem, estar->rhs[elem] + hval);
    }
    else {
      pqueue_remove (&estar->pruned, elem);
      for (nbor = estar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
	estar_update (estar, *nbor);
      }
    }
  }
  else {

    printf ("  raise\n");
    
    estar->phi[elem] = INFINITY;
    for (nbor = estar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
      estar_update (estar, *nbor);
    }
    estar_update (estar, elem);
  }
}


int estar_check (estar_t * estar, char const * pfx)
{
  int status;
  size_t ii, jj, kk;
  size_t elem;
  
  status = 0;
  
  for (ii = 0; ii < estar->grid.dimx; ++ii) {
    for (jj = 0; jj < estar->grid.dimy; ++jj) {
      elem = grid_elem (&estar->grid, ii, jj);
      
      if (estar->rhs[elem] == estar->phi[elem]) {
	// consistent
	if (0 != estar->pq.pos[elem]) {
	  printf ("%sconsistent cell [%4zu %4zu] should not be on queue\n", pfx, ii, jj);
	  status |= 1;
	}
      }
      else {
	// inconsistent
	if (0 == estar->pq.pos[elem]) {
	  printf ("%sinconsistent cell [%4zu %4zu] should be on queue\n", pfx, ii, jj);
	  status |= 2;
	}
      }
      
      if (0 == estar->pq.pos[elem]) {
	// not on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (elem == estar->pq.heap[kk]) {
	    printf ("%scell [%4zu %4zu] with queue pos 0 should not be on queue\n", pfx, ii, jj);
	    status |= 4;
	    break;
	  }
	}
      }
      else {
	// on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (elem == estar->pq.heap[kk]) {
	    break;
	  }
	}
	if (kk > estar->pq.len) {
	  printf ("%scell [%4zu %4zu] with queue pos %4zu should be on queue\n", pfx,
		  ii, jj, estar->pq.pos[elem]);
	  status |= 8;
	}
      }
    }
  }
  
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    if (estar->pq.pos[estar->pq.heap[ii]] != ii) {
      printf ("%sinconsistent cell [%4zu %4zu] with queue pos %zu should have queue pos %zu\n",
	      pfx,
	      grid_ix (&estar->grid, estar->pq.heap[ii]),
	      grid_iy (&estar->grid, estar->pq.heap[ii]),
	      estar->pq.pos[estar->pq.heap[ii]],
	      ii);
      status |= 16;
      break;
    }
  }
  if (status & 16) {
    estar_dump_queue (estar, pfx);
  }
  
  return status;
}


void estar_dump_queue (estar_t * estar, char const * pfx)
{
  size_t ii;
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    printf ("%s[%3zu %3zu]  pos:  %3zu  key: %g  phi: %g  rhs: %g\n",
	    pfx,
	    grid_ix (&estar->grid, estar->pq.heap[ii]),
	    grid_iy (&estar->grid, estar->pq.heap[ii]),
	    estar->pq.pos[estar->pq.heap[ii]],
	    estar->pq.key[estar->pq.heap[ii]],
	    estar->phi[estar->pq.heap[ii]],
	    estar->rhs[estar->pq.heap[ii]]);
  }
}
