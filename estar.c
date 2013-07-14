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


static double interpolate (double cost, double primary, double secondary)
{
  double tmp;
  
  if (cost <= secondary - primary) {
    return primary + cost;
  }
  
  // pow(cost,2) could be cached inside estar_set_speed. And so could
  // the other squared terms. That might speed things up, but it would
  // certainly make hearier caching code.
  
  tmp = primary + secondary;
  return (tmp + sqrt(pow(tmp, 2.0)
		     - 2.0 * (pow(primary, 2.0)
			      + pow(secondary, 2.0)
			      - pow(cost, 2.0)))) / 2.0;
}


static void calc_rhs (estar_t * estar, size_t elem, double phimax)
{
  size_t * prop;
  size_t primary;
  size_t secondary;
  double rr;
  
  estar->rhs[elem] = INFINITY;
  prop = estar->grid.cell[elem].prop;
  while ((size_t) -1 != *prop) {
    
    // XXXX not sure why I coded this first check up using rhs. This
    // should come after the two filters below, at which moment rhs
    // and phi are known to be equal to each other anyway.
    
    primary = *(prop++);
    if (estar->rhs[primary] <= estar->rhs[*prop])  {
      secondary = *(prop++);
    }
    else {
      secondary = primary;
      primary = *(prop++);
    }
    
    // do not propagate from obstacles, queued cells, cells above the
    // wavefront, or cells at infinity
    //
    // XXXX how about DBOUND cells? The same goes for the secondary.
    //
    if (estar->flags[primary] & FLAG_OBSTACLE
	|| estar->pq.pos[primary] != 0
	|| estar->phi[primary] > phimax
	|| isinf(estar->phi[primary])) {
      continue;
    }
    
    // the same goes from the secondary, but if that fails at least we
    // can fall back to the non-interpolated update equation.
    if (estar->flags[secondary] & FLAG_OBSTACLE
	|| estar->pq.pos[secondary] != 0
	|| estar->phi[secondary] > phimax
	|| isinf(estar->phi[secondary])) {
      rr = estar->rhs[primary] + estar->cost[elem];
    }
    else {
      rr = interpolate (estar->cost[elem], estar->phi[primary], estar->phi[secondary]);
    }
    
    if (rr < estar->rhs[elem]) {
      estar->rhs[elem] = rr;
    }
  }
  
  if (isinf (estar->rhs[elem])) {
    // None of the above worked, we're probably done... but I have
    // lingering doubts about about the effects of in-place primary /
    // secondary sorting above, it could be imagined to create
    // situations where we overlook something. So, just to be on the
    // safe side, let's retry all non-interpolated options.

#warning "THE SAME FILTERS AS ABOVE SHOULD BE IN PLACE HERE"

#warning "and anyway this should not be necessary if the above is coded properly (do not swap before filtering, but after!)"

    for (prop = estar->grid.cell[elem].prop; (size_t) -1 != *prop; ++prop) {
      rr = estar->phi[*prop];
      if (rr < estar->rhs[elem]) {
	estar->rhs[elem] = rr;
      }
    }
    estar->rhs[elem] += estar->cost[elem];
  }
}


void estar_init (estar_t * estar, size_t dimx, size_t dimy, hfunc_t hfunc)
{
  size_t ii;
  
  grid_init (&estar->grid, dimx, dimy);
  pqueue_init (&estar->pq, dimx + dimy, estar->grid.nelem);
  
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
  estar->obound = INFINITY;
}


void estar_fini (estar_t * estar)
{
  free(estar->flags);
  free(estar->rhs);
  free(estar->phi);
  free(estar->cost);
  grid_fini (&estar->grid);
  pqueue_fini (&estar->pq);
}


// XXXX set_goal in the documentation does a re-initialization, unlike
// this here which currently only gets called once at the
// beginning... this version here is OK to /add/ goal cells to the
// goal set, but that currently does not work. The interface for
// setting the goal should be redesigned anyway.
void estar_set_goal (estar_t * estar, size_t ix, size_t iy, double obound)
{
  size_t goal = grid_elem (&estar->grid, ix, iy);
  estar->rhs[goal] = 0.0;
  estar->flags[goal] |= FLAG_GOAL;
  estar->flags[goal] &= ~FLAG_OBSTACLE;
  pqueue_insert_or_update (&estar->pq, goal, 0.0);
  estar->obound = obound;
}


void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed)
{
  double cost;
  size_t elem;
  size_t * nbor;
  
  elem = grid_elem (&estar->grid, ix, iy);
  
  // XXXX I'm undecided yet whether this check here makes the most
  // sense. The other option is to make sure that the caller doesn't
  // place obstacles into a goal cell. The latter somehow makes more
  // sense to me at the moment, so in gestar.c there is code to filter
  // goal cells from the obstacle setting routines.
  ////  if (cell->flags & FLAG_GOAL) {
  ////    return;
  ////  }
  
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
    estar->flags[elem] |= FLAG_OBSTACLE;
  }
  else {
    estar->flags[elem] &= ~FLAG_OBSTACLE;
  }
  
  estar_update (estar, elem);
  for (nbor = estar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
    estar_update (estar, *nbor);
  }
}


void estar_set_obound (estar_t * estar, double obound)
{
#warning "reimplement this with a separate priority queue"

  /* if (obound > estar->obound) { */
  /*   // XXXX brute force for now, should have a separate heap for */
  /*   // pruned cells to make this more efficient. */
  /*   size_t ii, nn; */
  /*   cell_t * cell; */
  /*   nn = estar->grid.dimx * estar->grid.dimy; */
  /*   cell = estar->grid.cell; */
  /*   for (ii = 0; ii < nn; ++ii) { */
  /*     if ((cell->flags & FLAG_DBOUND) && (cell->rhs + estar->hfunc(cell) < obound)) { */
  /* 	cell->flags &= ~FLAG_DBOUND; */
  /* 	cell->phi = INFINITY; */
  /* 	pqueue_insert_or_update (&estar->pq, cell); */
  /*     } */
  /*     ++cell; */
  /*   } */
  /* } */
  
  estar->obound = obound;
}


static void estar_update (estar_t * estar, size_t elem)
{
  if (estar->flags[elem] & FLAG_OBSTACLE) {
    pqueue_remove_or_ignore (&estar->pq, elem);
    return;
  }
  
  /* Make sure that goal cells remain at their rhs, which is supposed
     to be fixed and only serve as source for propagation, never as
     sink. */
  if ( ! (estar->flags[elem] & FLAG_GOAL)) {
    calc_rhs (estar, elem, pqueue_topkey (&estar->pq));
  }
  
  if (estar->phi[elem] != estar->rhs[elem]) {
    if (estar->rhs[elem] < estar->phi[elem]) {
      pqueue_insert_or_update (&estar->pq, elem, estar->rhs[elem]);
    }
    else {
      pqueue_insert_or_update (&estar->pq, elem, estar->phi[elem]);
    }
  }
  else {
    pqueue_remove_or_ignore (&estar->pq, elem);
  }
}


void estar_propagate (estar_t * estar)
{
  size_t elem;
  size_t * nbor;
  
  //////////////////////////////////////////////////
  // determine next element to expand
  
  do {
    elem = pqueue_extract_or_what (&estar->pq);
    if ((size_t) -1 == elem) {
      return;
    }
    if (estar->rhs[elem] + estar->hfunc (elem) >= estar->obound) {
      estar->flags[elem] |= FLAG_DBOUND;
      estar->phi[elem] = INFINITY;	/* not sure if this is needed */
    }
    else {
      estar->flags[elem] &= ~FLAG_DBOUND;
      break;
    }
  } while (1);
  
  //////////////////////////////////////////////////
  // expand it
  
  if (estar->phi[elem] > estar->rhs[elem]) {
    estar->phi[elem] = estar->rhs[elem];
    for (nbor = estar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
      estar_update (estar, *nbor);
    }
  }
  else {
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
	  printf ("%sconsistent cell should not be on queue\n", pfx);
	  status |= 1;
	}
      }
      else {
	// inconsistent
	if ( (! (estar->flags[elem] | FLAG_DBOUND)) && 0 == estar->pq.pos[elem]) {
	  printf ("%sinconsistent cell should be on queue\n", pfx);
	  status |= 2;
	}
      }
      
      if (0 == estar->pq.pos[elem]) {
	// not on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (elem == estar->pq.heap[kk]) {
	    printf ("%scell with queue pos 0 should not be on queue\n", pfx);
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
	  printf ("%scell with queue pos != 0 should be on queue\n", pfx);
	  status |= 8;
	}
      }
    }
  }
  
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    if (estar->pq.pos[estar->pq.heap[ii]] != ii) {
      printf ("%sinconsistent queue pos %zu should be %zu\n", pfx,
	      estar->pq.pos[estar->pq.heap[ii]], ii);
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
    printf ("%s[%zu %zu]  pqi:  %zu  key: %g  phi: %g  rhs: %g\n",
	    pfx,
	    grid_ix (&estar->grid, estar->pq.heap[ii]),
	    grid_iy (&estar->grid, estar->pq.heap[ii]),
	    estar->pq.pos[estar->pq.heap[ii]],
	    estar->pq.key[estar->pq.heap[ii]],
	    estar->phi[estar->pq.heap[ii]],
	    estar->rhs[estar->pq.heap[ii]]);
  }
}
