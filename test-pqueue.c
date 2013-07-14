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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>


static int check (pqueue_t * pq, double * key, size_t len)
{
  size_t ii;
  size_t elem;
  
  for (ii = 0; ii < len; ++ii) {
    elem = pqueue_extract_or_what (pq);
    if ((size_t) -1 == elem) {
      printf ("  ERROR queue empty at ii = %zu\n", ii);
      return 1;
    }
    if (pq->key[elem] != key[ii]) {
      printf ("  ERROR key at ii = %zu is %g but should be %g\n", ii, pq->key[elem], key[ii]);
      return 2;
    }
    if (0 != pq->pos[elem]) {
      printf ("  ERROR pq->pos[elem] should be zero after pqueue_extract\n");
      return 3;
    }
  }
  
  elem = pqueue_extract_or_what (pq);
  if ((size_t) -1 != elem) {
    printf ("  ERROR queue should be empty after %zu extractions\n", len);
    return 4;
  }
  
  return 0;
}


int main (int argc, char ** argv)
{
  pqueue_t pq;
  double key[] = { 1.1, 2.1, 2.2, 3.3 };
  
  pqueue_init (&pq, 2, 10);
  
  pqueue_insert_or_update (&pq, 0, 2.2);
  printf ("after insertion of (0, 2.2)\n");
  pqueue_dump (&pq, "  ");

  pqueue_insert_or_update (&pq, 1, 3.3);
  printf ("after insertion of (1, 3.3)\n");
  pqueue_dump (&pq, "  ");
  
  pqueue_insert_or_update (&pq, 2, 1.9);
  printf ("after insertion of (2, 1.9)\n");
  pqueue_dump (&pq, "  ");
  
  pqueue_insert_or_update (&pq, 3, 1.1);
  printf ("after insertion of (3, 1.1)\n");
  pqueue_dump (&pq, "  ");
  
  pqueue_insert_or_update (&pq, 4, 3.3);
  printf ("after insertion of (4, 3.3)\n");
  pqueue_dump (&pq, "  ");
  
  pqueue_insert_or_update (&pq, 1, 2.1);
  printf ("after update of (1, 2.1)\n");
  pqueue_dump (&pq, "  ");
  
  pqueue_remove_or_ignore (&pq, 2);
  printf ("after removal of 2\n");
  pqueue_dump (&pq, "  ");
  
  if (0 == check (&pq, key, sizeof(key) / sizeof(double))) {
    printf ("OK\n");
  }
  
  return 0;
}
