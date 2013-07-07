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


static void calc_rhs (cell_t * cell, double phimax)
{
  cell_t ** prop;
  cell_t * primary;
  cell_t * secondary;
  double rr;
  
  cell->rhs = INFINITY;
  prop = cell->prop;
  while (NULL != *prop) {
    
    primary = *(prop++);
    if (primary->rhs <= (*prop)->rhs)  {
      secondary = *(prop++);
    }
    else {
      secondary = primary;
      primary = *(prop++);
    }
    
    // do not propagate from obstacles, queued cells, cells above the
    // wavefront, or cells at infinity
    if (primary->flags & FLAG_OBSTACLE
	|| primary->pqi != 0
	|| primary->phi > phimax
	|| isinf(primary->phi)) {
      continue;
    }
    
    // the same goes from the secondary, but if that fails at least we
    // can fall back to the non-interpolated update equation.
    if (secondary->flags & FLAG_OBSTACLE
	|| secondary->pqi != 0
	|| secondary->phi > phimax
	|| isinf(secondary->phi)) {
      rr = primary->rhs + cell->cost;
    }
    else {
      rr = interpolate (cell->cost, primary->rhs, secondary->rhs);
    }
    
    if (rr < cell->rhs) {
      cell->rhs = rr;
    }
  }
  
  // If none of the above worked, we're probably done anyway, but I'm
  // not sure about the in-place sorting between primary and secondary
  // above and whether that might not, in combination with infinite
  // values maybe, create situations where we overlook something. So,
  // just to be on the safe side, retry all non-interpolated options.
  if (isinf (cell->rhs)) {
    for (prop = cell->nbor; *prop != 0; ++prop) {
      rr = (*prop)->phi;
      if (rr < cell->rhs) {
	cell->rhs = rr;
      }
    }
    cell->rhs += cell->cost;
  }
}


void estar_init (estar_t * estar, size_t dimx, size_t dimy)
{
  grid_init (&estar->grid, dimx, dimy);
  pqueue_init (&estar->pq, dimx + dimy);
}


void estar_fini (estar_t * estar)
{
  grid_fini (&estar->grid);
  pqueue_fini (&estar->pq);
}


void estar_set_goal (estar_t * estar, size_t ix, size_t iy)
{
  cell_t * goal = grid_at (&estar->grid, ix, iy);
  goal->rhs = 0.0;
  goal->flags |= FLAG_GOAL;
  pqueue_insert (&estar->pq, goal);
}


void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed)
{
  cell_t * cell;
  cell_t ** nbor;

  cell = grid_at (&estar->grid, ix, iy);
  if (speed <= 0.0) {
    cell->cost = INFINITY;
    cell->phi = INFINITY;
    cell->rhs = INFINITY;
    cell->flags |= FLAG_OBSTACLE;
  }
  else {
    cell->cost = 1.0 / speed;
    cell->flags &= ~FLAG_OBSTACLE;
  }
  
  estar_update_cell (estar, cell);
  for (nbor = cell->nbor; *nbor != 0; ++nbor) {
    estar_update_cell (estar, *nbor);
  }
}


void estar_update_cell (estar_t * estar, cell_t * cell)
{
  if (cell->flags & FLAG_OBSTACLE || cell->flags & FLAG_GOAL) {
    if (cell->pqi != 0) {
      pqueue_remove (&estar->pq, cell);
    }
    return;
  }
  
  calc_rhs (cell, pqueue_topkey (&estar->pq));
  
  if (cell->phi != cell->rhs) {
    if (cell->pqi == 0) {
      pqueue_insert (&estar->pq, cell);
    }
    else {
      pqueue_update (&estar->pq, cell);
    }
  }
  else if (cell->pqi != 0) {
    pqueue_remove (&estar->pq, cell);
  }
}


void estar_step (estar_t * estar)
{
  cell_t * cell;
  cell_t ** nbor;
  
  cell = pqueue_extract (&estar->pq);
  if (NULL == cell) {
    return;
  }
  
  if (cell->phi > cell->rhs) {
    cell->phi = cell->rhs;
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update_cell (estar, *nbor);
    }
  }
  else {
    cell->phi = INFINITY;
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update_cell (estar, *nbor);
    }
    estar_update_cell (estar, cell);
  }
}


int estar_check (estar_t * estar, char const * pfx)
{
  int status;
  size_t ii, jj, kk;
  
  status = 0;
  
  for (ii = 0; ii < estar->grid.dimx; ++ii) {
    for (jj = 0; jj < estar->grid.dimy; ++jj) {
      cell_t * cell;
      cell = grid_at (&estar->grid, ii, jj);
      
      if (cell->rhs == cell->phi) {
	// consistent
	if (0 != cell->pqi) {
	  printf ("%sconsistent cell should not be on queue\n", pfx);
	  status |= 1;
	}
      }
      else {
	// inconsistent
	if (0 == cell->pqi) {
	  printf ("%sinconsistent cell should be on queue\n", pfx);
	  status |= 2;
	}
      }
      
      if (0 == cell->pqi) {
	// not on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (cell == estar->pq.heap[kk]) {
	    printf ("%scell with pqi == 0 should not be on queue\n", pfx);
	    status |= 4;
	    break;
	  }
	}
      }
      else {
	// on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (cell == estar->pq.heap[kk]) {
	    break;
	  }
	}
	if (kk > estar->pq.len) {
	  printf ("%scell with pqi != 0 should be on queue\n", pfx);
	  status |= 8;
	}
      }
    }
  }
  
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    if (estar->pq.heap[ii]->pqi != ii) {
      printf ("%sinconsistent pqi\n", pfx);
      estar_dump_queue (estar, pfx);
      status |= 16;
      break;
    }
  }
  
  return status;
}


void estar_dump_queue (estar_t * estar, char const * pfx)
{
  size_t ii;
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    printf ("%s[%zu %zu]  pqi:  %zu  key: %g  phi: %g  rhs: %g\n",
	    pfx,
	    (estar->pq.heap[ii] - estar->grid.cell) % estar->grid.dimx,
	    (estar->pq.heap[ii] - estar->grid.cell) / estar->grid.dimx,
	    estar->pq.heap[ii]->pqi, estar->pq.heap[ii]->key,
	    estar->pq.heap[ii]->phi, estar->pq.heap[ii]->rhs);
  }
}
