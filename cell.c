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

#include "cell.h"


int cell_calc_gradient (cell_t * cell, double * gx, double * gy)
{
  cell_t ** nn;
  cell_t * n1;
  cell_t * n2;
  int direction;
  
  n1 = NULL;
  for (nn = cell->nbor; *nn != NULL; ++nn) {
    if ((*nn)->rhs < cell->rhs
	&& (n1 == NULL || (*nn)->rhs < n1->rhs)) {
      n1 = *nn;
    }
  }
  if (NULL == n1) {
    return 0;
  }
  
  direction = n1 - cell;
  n2 = NULL;
  for (nn = cell->nbor; *nn != NULL; ++nn) {
    if ((*nn) != n1
	&& direction != cell - *nn
	&& (n2 == NULL || (*nn)->rhs < n2->rhs)) {
      n2 = *nn;
    }
  }
  
  if (NULL == n2) {
    if (direction == -1) {
      *gx = n1->rhs - cell->rhs;
      *gy = 0.0;
    }
    else if (direction == 1) {
      *gx = cell->rhs - n1->rhs;
      *gy = 0.0;
    }
    else if (direction < 0) {
      *gx = 0.0;
      *gy = n1->rhs - cell->rhs;
    }
    else {
      *gx = 0.0;
      *gy = cell->rhs - n1->rhs;
    }
    return 1;
  }
  
  if (direction == -1) {
    *gx = n1->rhs - cell->rhs;
    if (cell - n2 > 0) {
      *gy = n2->rhs - cell->rhs;
    }
    else {
      *gy = cell->rhs - n2->rhs;
    }
  }
  else if (direction == 1) {
    *gx = cell->rhs - n1->rhs;
    if (cell - n2 > 0) {
      *gy = n2->rhs - cell->rhs;
    }
    else {
      *gy = cell->rhs - n2->rhs;
    }
  }
  else if (direction < 0) {
    if (cell - n2 > 0) {
      *gx = n2->rhs - cell->rhs;
    }
    else {
      *gx = cell->rhs - n2->rhs;
    }
    *gy = n1->rhs - cell->rhs;
  }
  else {
    if (cell - n2 > 0) {
      *gx = n2->rhs - cell->rhs;
    }
    else {
      *gx = cell->rhs - n2->rhs;
    }
    *gy = cell->rhs - n1->rhs;
  }
  
  return 2;
}
