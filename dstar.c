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

#include "dstar.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <err.h>


static void dstar_update (dstar_t * dstar, size_t elem);


static void calc_rhs (dstar_t * dstar, size_t elem, double phimax)
{
  size_t * nbor;
  dstar->rhs[elem] = INFINITY;
  for (nbor = dstar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
    if (dstar->flags[*nbor] & DSTAR_FLAG_OBSTACLE
	|| dstar->pq.pos[*nbor] != 0
	|| dstar->phi[*nbor] > phimax
	|| isinf(dstar->phi[*nbor])) {
      continue;
    }
    if (dstar->phi[*nbor] < dstar->rhs[elem]) {
      dstar->rhs[elem] = dstar->phi[*nbor];
    }
  }
  dstar->rhs[elem] += dstar->cost[elem];
}


void dstar_init (dstar_t * dstar, size_t dimx, size_t dimy, dstar_hfunc_t hfunc)
{
  size_t ii;
  
  grid_init (&dstar->grid, dimx, dimy);
  pqueue_init (&dstar->pq, dimx + dimy, dstar->grid.nelem);
  
  dstar->cost = malloc ((sizeof *dstar->cost) * dstar->grid.nelem);
  if (NULL == dstar->cost) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc cost", __func__);
  }
  dstar->phi = malloc ((sizeof *dstar->phi) * dstar->grid.nelem);
  if (NULL == dstar->phi) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc phi", __func__);
  }
  dstar->rhs = malloc ((sizeof *dstar->rhs) * dstar->grid.nelem);
  if (NULL == dstar->rhs) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc rhs", __func__);
  }
  dstar->flags = malloc ((sizeof *dstar->flags) * dstar->grid.nelem);
  if (NULL == dstar->flags) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc flags", __func__);
  }
  
  for (ii = 0; ii < dstar->grid.nelem; ++ii) {
    dstar->cost[ii] = 1.0;
    dstar->phi[ii] = INFINITY;
    dstar->rhs[ii] = INFINITY;
    dstar->flags[ii] = 0;
  }
  
  dstar->hfunc = hfunc;
}


void dstar_fini (dstar_t * dstar)
{
  free(dstar->flags);
  free(dstar->rhs);
  free(dstar->phi);
  free(dstar->cost);
  grid_fini (&dstar->grid);
  pqueue_fini (&dstar->pq);
}


// XXXX set_goal in the documentation does a re-initialization, unlike
// this here which currently only gets called once at the
// beginning... this version here is OK to /add/ goal cells to the
// goal set, but that currently does not work. The interface for
// setting the goal should be redesigned anyway.
void dstar_set_goal (dstar_t * dstar, size_t ix, size_t iy)
{
  size_t goal = grid_elem (&dstar->grid, ix, iy);
  dstar->rhs[goal] = 0.0;
  dstar->flags[goal] |= DSTAR_FLAG_GOAL;
  dstar->flags[goal] &= ~DSTAR_FLAG_OBSTACLE;
  pqueue_insert_or_update (&dstar->pq, goal, 0.0);
}


void dstar_set_speed (dstar_t * dstar, size_t ix, size_t iy, double speed)
{
  double cost;
  size_t elem;
  size_t * nbor;
  
  elem = grid_elem (&dstar->grid, ix, iy);
  
  if (speed <= 0.0) {
    cost = INFINITY;
  }
  else {
    cost = 1.0 / speed;
  }
  if (cost == dstar->cost[elem]) {
    return;
  }
  
  dstar->cost[elem] = cost;
  if (speed <= 0.0) {
    dstar->phi[elem] = INFINITY;
    dstar->rhs[elem] = INFINITY;
    dstar->flags[elem] |= DSTAR_FLAG_OBSTACLE;
  }
  else {
    dstar->flags[elem] &= ~DSTAR_FLAG_OBSTACLE;
  }
  
  dstar_update (dstar, elem);
  for (nbor = dstar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
    dstar_update (dstar, *nbor);
  }
}


static void dstar_update (dstar_t * dstar, size_t elem)
{
  if (dstar->flags[elem] & DSTAR_FLAG_OBSTACLE) {
    pqueue_remove_or_ignore (&dstar->pq, elem);
    return;
  }
  
  /* Make sure that goal cells remain at their rhs, which is supposed
     to be fixed and only serve as source for propagation, never as
     sink. */
  if ( ! (dstar->flags[elem] & DSTAR_FLAG_GOAL)) {
    calc_rhs (dstar, elem, pqueue_topkey (&dstar->pq));
  }
  
  if (dstar->phi[elem] != dstar->rhs[elem]) {
    if (dstar->rhs[elem] < dstar->phi[elem]) {
      pqueue_insert_or_update (&dstar->pq, elem, dstar->rhs[elem]);
    }
    else {
      pqueue_insert_or_update (&dstar->pq, elem, dstar->phi[elem]);
    }
  }
  else {
    pqueue_remove_or_ignore (&dstar->pq, elem);
  }
}


void dstar_propagate (dstar_t * dstar)
{
  size_t elem;
  size_t * nbor;
  
  elem = pqueue_extract_or_what (&dstar->pq);
  if ((size_t) -1 == elem) {
    return;
  }
  
  if (dstar->phi[elem] > dstar->rhs[elem]) {
    dstar->phi[elem] = dstar->rhs[elem];
    for (nbor = dstar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
      dstar_update (dstar, *nbor);
    }
  }
  else {
    dstar->phi[elem] = INFINITY;
    for (nbor = dstar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
      dstar_update (dstar, *nbor);
    }
    dstar_update (dstar, elem);
  }
}


int dstar_compute_path (dstar_t * dstar, size_t sx, size_t sy)
{
  size_t start, elem;
  size_t * nbor;
  int nsteps;
  double skey;
  
  start = grid_elem (&dstar->grid, sx, sy);
  skey = dstar->hfunc (start);
  
  printf ("dstar_compute_path\n  queue size %zu\n  propagating ", dstar->pq.len);
  
  while (0 != dstar->pq.len) { ////pqueue_topkey (&dstar->pq) < skey || dstar->rhs[start] != dstar->phi[start]) {
    
    printf (".");
    fflush (stdout);
    
    dstar_propagate (dstar);
    if (pqueue_empty (&dstar->pq)) {
      
      printf ("X");
      break;
    }
  }
  
  printf ("\n");
  
  for (elem = 0; elem < dstar->grid.nelem; ++elem) {
    dstar->flags[elem] &= ~DSTAR_FLAG_PATH;
  }
  
  if (isinf (dstar->phi[start])) {
    printf ("  unreachable\n");
    /* unreachable */
    return -1;
  }
  
  nsteps = 0;
  elem = start;
  do {
    printf ("  %g\n", dstar->phi[elem]);
    ++nsteps;
    dstar->flags[elem] |= DSTAR_FLAG_PATH;
    for (nbor = dstar->grid.cell[elem].nbor; (size_t) -1 != *nbor; ++nbor) {
      if (dstar->phi[*nbor] < dstar->phi[elem]) {
	elem = *nbor;
      }
    }
  } while ( ! dstar->flags[elem] & DSTAR_FLAG_GOAL);
  
  return nsteps;
}
