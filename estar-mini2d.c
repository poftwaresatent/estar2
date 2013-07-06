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
#include "pqueue.h"

#include <gtk/gtk.h>
#include <err.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define DIMX 100
#define DIMY 100


static cell_t grid[DIMX * DIMY];
static pqueue_t pq;

static GtkWidget * w_phi;
static gint w_phi_width, w_phi_height;
static gint w_phi_sx, w_phi_sy, w_phi_x0, w_phi_y0;
static int play;


#define cidx(ii,jj) ((ii)+(jj)*DIMX)


static void dump_cell (char const * pfx, cell_t const * cell)
{
  size_t ix, iy;
  ix = (cell - grid) % DIMX;
  iy = (cell - grid) / DIMX;
  printf ("%s[%3zu  %3zu]  k: %4g  r: %4g  p: %4g\n",
	  pfx, ix, iy, cell->key, cell->rhs, cell->phi);
}


static void dump_queue ()
{
  size_t ii;
  printf ("  queue length %zu\n", pq.len);
  for (ii = 1; ii <= pq.len; ++ii) {
    dump_cell ("    ", pq.heap[ii]);
  }
}


static void fini ()
{
  pqueue_fini (&pq);
}


static void init ()
{
  size_t ii, jj, idx;
  cell_t ** succ;
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      idx = cidx(ii, jj);
      grid[idx].phi = NAN;
      grid[idx].rhs = NAN;
      grid[idx].key = NAN;
      grid[idx].pqi = 0;
      succ = grid[idx].succ;
      if (ii > 0) {
	*(succ++) = &grid[idx - 1];
      }
      if (ii < DIMX - 1) {
	*(succ++) = &grid[idx + 1];
      }
      if (jj > 0) {
	*(succ++) = &grid[idx - DIMX];
      }
      if (jj < DIMY - 1) {
	*(succ++) = &grid[idx + DIMX];
      }
      *succ = 0;
    }
  }
  
  idx = cidx(2, 2);		/* goal */
  grid[idx].rhs = 0.0;
  
  pqueue_init (&pq, DIMX + DIMY);
  pqueue_insert (&pq, &grid[idx]);
  
  play = 0;
  
  printf ("  initialized\n");
  dump_queue ();
}


static void update_cell (cell_t * cell)
{
  cell_t ** succ;
  
  if (cell->rhs == 0) {		/* hacked goal detection */
    return;
  }
  
  printf ("  update_cell: min of {");
  cell->rhs = NAN;
  for (succ = cell->succ; *succ != 0; ++succ) {
    double rr = 1.0 + (*succ)->phi;
    printf ("  %4g", rr);
    if (isnan(cell->rhs) || rr < cell->rhs) {
      cell->rhs = rr;
    }
  }
  printf ("  } is %4g\n", cell->rhs);
  
  if (cell->phi != cell->rhs) {
    if (cell->pqi == 0) {
      pqueue_insert (&pq, cell);
      dump_cell ("  update_cell: inserted ", cell);
    }
    else {
      pqueue_update (&pq, cell);
      dump_cell ("  update_cell: updated  ", cell);
    }
  }
  else if (cell->pqi != 0) {
    pqueue_remove (&pq, cell);
    dump_cell (  "  update_cell: removed  ", cell);
  }
}


static void update ()
{
  cell_t * cell;
  cell_t ** succ;
  
  if (pq.len == 0) {
    return;
  }
  
  cell = pqueue_extract (&pq);
  if (isnan(cell->phi) || cell->phi > cell->rhs) {
    dump_cell ("update: lower\n  before: ", cell);
    cell->phi = cell->rhs;
    for (succ = cell->succ; *succ != 0; ++succ) {
      update_cell (*succ);
    }
  }
  else {
    dump_cell ("update: raise\n  before", cell);
    cell->phi = NAN;
    for (succ = cell->succ; *succ != 0; ++succ) {
      update_cell (*succ);
    }
    update_cell (cell);
  }
  dump_cell ("  after ", cell);
  
  dump_queue ();
  
  gtk_widget_queue_draw (w_phi);
}


void cb_play (GtkWidget * ww, gpointer data)
{
  if (play) {
    play = 0;
    g_print("PAUSE\n");
  }
  else {
    play = 1;
    g_print("PLAY\n");
  }
}


void cb_next (GtkWidget * ww, gpointer data)
{
  if (play) {
    play = 0;
    g_print("PAUSE\n");    
  }
  else {
    update ();
  }    
}


void cb_quit (GtkWidget * ww, gpointer data)
{
  g_print("quit\n");
  gtk_main_quit();
}


gint cb_phi_expose (GtkWidget * ww,
		    GdkEventExpose * ee,
		    gpointer data)
{
  size_t ii, jj;
  cairo_t * cr = gdk_cairo_create (ee->window);
  double phimax;
  
  phimax = 0.0;
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      size_t const idx = cidx(ii, jj);
      if (phimax < grid[idx].phi) {
	phimax = grid[idx].phi;
      }
    }
  }
  
  cr = gdk_cairo_create (ee->window);
  
  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0);
  cairo_rectangle (cr, 0, 0, w_phi_width, w_phi_height);
  cairo_fill (cr);
  
  cairo_set_source_rgb (cr, 0.5, 0.5, 0.5);
  cairo_set_line_width (cr, 2.0);
  cairo_rectangle (cr, w_phi_x0 - 2, w_phi_y0 + 2, DIMX * w_phi_sx + 4, DIMY * w_phi_sy - 4);
  cairo_stroke (cr);
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      cell_t * cell = &grid[cidx(ii, jj)];
      if (cell->pqi == 0) {
	// not on queue
	if (isnan(cell->phi)) {
	  // unvisited
	  cairo_set_source_rgb (cr, 0.0, 0.0, 0.5);
	}
	else {
	  if (cell->phi == cell->rhs) {
	    // consistent
	    //// cairo_set_source_rgb (cr, 0.0, 1.0 - cell->phi / phimax, 0.0);
	    cairo_set_source_rgb (cr, 0.0, 1.0, 0.0);
	  }
	  else {
	    // inconsistent
	    cairo_set_source_rgb (cr, 0.0, 1.0 - cell->phi / phimax, 0.5);
	  }
	}
      }
      else {
	// queued
	//// cairo_set_source_rgb (cr, 1.0 - cell->key / phimax, 0.0, 0.0);
	cairo_set_source_rgb (cr, 1.0, 0.0, 0.0);
      }
      cairo_rectangle (cr,
		       w_phi_x0 + (ii-1) * w_phi_sx,
		       w_phi_y0 + jj * w_phi_sy,
		       w_phi_sx,
		       - w_phi_sy);
      cairo_fill (cr);
    }
  }
  
  cairo_destroy (cr);
  
  return TRUE;			// TRUE to stop event propagation
}


gint cb_phi_size_allocate (GtkWidget * ww,
			   GtkAllocation * aa,
			   gpointer data)
{
  w_phi_width = aa->width;
  w_phi_height = aa->height;
  
  w_phi_sx = w_phi_width / DIMX;
  if (w_phi_sx < 1) {
    w_phi_sx = 1;
  }
  w_phi_sy = - w_phi_height / DIMY;
  if ( - w_phi_sy < 1) {
    w_phi_sy = -1;
  }
  if (w_phi_sx > - w_phi_sy) {
    w_phi_sx = - w_phi_sy;
  }
  else {
    w_phi_sy = - w_phi_sx;
  }
  w_phi_x0 = (w_phi_width - DIMX * w_phi_sx) / 2;
  w_phi_y0 = w_phi_height - (w_phi_height + DIMY * w_phi_sy) / 2;
  
  return TRUE;			// TRUE to stop event propagation
}


gint cb_phi_click (GtkWidget * ww,
		   GdkEventButton * bb,
		   gpointer data)
{
  size_t ii, jj;
  gdouble const cx = (bb->x - w_phi_x0) / w_phi_sx + 0.5;
  gdouble const cy = (bb->y - w_phi_y0) / w_phi_sy + 0.5;
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      double const dd = sqrt (pow (cx - ii, 2.0) + pow (cy - jj, 2.0));
      size_t const idx = cidx(ii, jj);
      grid[idx].phi  = dd;
    }
  }
  
  gtk_widget_queue_draw (w_phi);
  
  return TRUE;			// TRUE to stop event propagation
}


gint idle (gpointer data)
{
  if (play) {
    update ();
  }
  return TRUE;
}


int main (int argc, char ** argv)
{
  GtkWidget *window, *vbox, *hbox, *btn;
  
  if (0 != atexit(fini)) {
    err (EXIT_FAILURE, "atexit");
  }
  
  gtk_init (&argc, &argv);
  init ();
  
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  
  vbox = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox);
  gtk_widget_show (vbox);
  
  w_phi = gtk_drawing_area_new ();
  g_signal_connect (w_phi, "expose_event", G_CALLBACK (cb_phi_expose), NULL);
  g_signal_connect (w_phi, "size_allocate", G_CALLBACK (cb_phi_size_allocate), NULL);
  g_signal_connect (w_phi, "button_press_event", G_CALLBACK (cb_phi_click), NULL);
  gtk_widget_set_events (w_phi, GDK_BUTTON_PRESS_MASK);
  
  gtk_widget_show (w_phi);
  
  gtk_widget_set_size_request (w_phi, 400, 500);
  gtk_box_pack_start (GTK_BOX (vbox), w_phi, TRUE, TRUE, 0);
  
  hbox = gtk_hbox_new (TRUE, 3);
  gtk_box_pack_start (GTK_BOX (vbox), hbox, FALSE, TRUE, 0);
  gtk_widget_show (hbox);
  
  btn = gtk_button_new_with_label ("play");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_play), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
  btn = gtk_button_new_with_label ("next");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_next), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
  btn = gtk_button_new_with_label ("quit");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_quit), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
  gtk_idle_add (idle, 0);
  
  gtk_widget_show (window);
  gtk_main ();
  
  return 0;
}
