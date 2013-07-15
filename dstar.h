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

#ifndef ESTAR_DSTAR_H
#define ESTAR_DSTAR_H

#include "grid.h"
#include "pqueue.h"


enum {
  DSTAR_FLAG_GOAL      = 1,
  DSTAR_FLAG_OBSTACLE  = 2,
  DSTAR_FLAG_PATH      = 4
};


typedef double (*hfunc_t)(size_t);

typedef struct {
  double * cost;
  double * phi;
  double * rhs;
  int * flags;
  grid_t grid;
  pqueue_t pq;
  hfunc_t hfunc;
} dstar_t;


void dstar_init (dstar_t * dstar, size_t dimx, size_t dimy, hfunc_t hfunc);
void dstar_fini (dstar_t * dstar);
void dstar_set_goal (dstar_t * dstar, size_t ix, size_t iy);
void dstar_set_speed (dstar_t * dstar, size_t ix, size_t iy, double speed);
void dstar_propagate (dstar_t * dstar);
void dstar_propagate_to (dstar_t * dstar, size_t sx, size_t sy);

#endif
