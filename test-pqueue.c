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

#include "grid.h"
#include "pqueue.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>


static int check (pqueue_t * pq, double * key, size_t len)
{
  size_t ii;
  cell_t * cell;
  
  for (ii = 0; ii < len; ++ii) {
    cell = pqueue_extract (pq);
    if (NULL == cell) {
      printf ("  ERROR queue empty at ii = %zu\n", ii);
      return 1;
    }
    if (cell->key != key[ii]) {
      printf ("  ERROR key at ii = %zu is %g but should be %g\n", ii, cell->key, key[ii]);
      return 2;
    }
    if (0 != cell->pqi) {
      printf ("  ERROR pqi should be zero after pqueue_extract\n");
      return 3;
    }
  }
  
  cell = pqueue_extract (pq);
  if (NULL != cell) {
    printf ("  ERROR queue should be empty after %zu extractions\n", len);
    return 4;
  }
  
  return 0;
}


int main (int argc, char ** argv)
{
  pqueue_t pq;
  double key[] = { 1.1, 2.2, 2.2, 3.3 };
  grid_t grid;
  
  pqueue_init (&pq, 5);
  grid_init (&grid, 10, 1);
  
  grid.cell[0].rhs = 2.2;
  grid.cell[1].rhs = 3.3;
  grid.cell[2].rhs = 1.9;
  grid.cell[3].rhs = 1.1;
  grid.cell[4].rhs = 3.3;
  
  pqueue_insert (&pq, &grid.cell[0]);
  printf ("after insertion of grid[0]  %p\n", &grid.cell[0]);
  pqueue_dump (&pq, &grid, "  ");

  pqueue_insert (&pq, &grid.cell[1]);
  printf ("after insertion of grid[1]  %p\n", &grid.cell[1]);
  pqueue_dump (&pq, &grid, "  ");
  
  pqueue_insert (&pq, &grid.cell[2]);
  printf ("after insertion of grid[2]  %p\n", &grid.cell[2]);
  pqueue_dump (&pq, &grid, "  ");
  
  pqueue_insert (&pq, &grid.cell[3]);
  printf ("after insertion of grid[3]  %p\n", &grid.cell[3]);
  pqueue_dump (&pq, &grid, "  ");
  
  pqueue_insert (&pq, &grid.cell[4]);
  printf ("after insertion of grid[4]  %p\n", &grid.cell[4]);
  pqueue_dump (&pq, &grid, "  ");
  
  grid.cell[1].rhs = 2.2;
  pqueue_update (&pq, &grid.cell[1]);
  printf ("after update of grid[1] %p to 2.2\n", &grid.cell[1]);
  pqueue_dump (&pq, &grid, "  ");
  
  pqueue_remove (&pq, &grid.cell[2]);
  printf ("after removal of %p\n", &grid.cell[2]);
  pqueue_dump (&pq, &grid, "  ");
  
  if (0 == check (&pq, key, sizeof(key) / sizeof(double))) {
    printf ("OK\n");
  }
  
  return 0;
}
