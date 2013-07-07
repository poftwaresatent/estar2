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

#include <gtk/gtk.h>
#include <err.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define DIMX 50
#define DIMY 50


static estar_t estar;

static GtkWidget * w_phi;
static gint w_phi_width, w_phi_height;
static gint w_phi_sx, w_phi_sy, w_phi_x0, w_phi_y0;
static int play, dbg;


static void fini ()
{
  estar_fini (&estar);
}


static void init ()
{
  estar_init (&estar, DIMX, DIMY);
  estar_set_goal (&estar, 2, 2);
  
  play = 0;
  dbg = 0;
  
  if (dbg) {
    printf ("  initialized\n");
    estar_dump_queue (&estar, "  ");
  }
}


static void update ()
{
  int status;
  
  if (estar.pq.len == 0) {
    /* if (play) { */
    /*   play = 0; */
    /*   printf("PAUSE\n"); */
    /* } */
    return;
  }
  
  estar_step (&estar);
  
  status = estar_check (&estar, "*** ");
  if (0 != status) {
    play = 0;
    printf ("ERROR %d (see above)\n", status);
  }
  
  gtk_widget_queue_draw (w_phi);
}


void cb_flush (GtkWidget * ww, gpointer data)
{
  printf ("FLUSH\n");
  while (estar.pq.len != 0) {
    estar_step (&estar);
  }
  gtk_widget_queue_draw (w_phi);
}


void cb_play (GtkWidget * ww, gpointer data)
{
  if (play) {
    play = 0;
    printf ("PAUSE\n");
  }
  else {
    play = 1;
    printf ("PLAY\n");
  }
}


void cb_next (GtkWidget * ww, gpointer data)
{
  if (play) {
    play = 0;
  }
  else {
    update ();
  }    
}


void cb_quit (GtkWidget * ww, gpointer data)
{
  gtk_main_quit();
}


gint cb_phi_expose (GtkWidget * ww,
		    GdkEventExpose * ee,
		    gpointer data)
{
  size_t ii, jj;
  cairo_t * cr = gdk_cairo_create (ee->window);
  double topkey, maxkey, maxrhs;
  cell_t * cell;
  
  if (estar.pq.len > 0) {
    topkey = estar.pq.heap[1]->key;
  }
  else {
    topkey = INFINITY;
  }
  maxkey = topkey;
  for (ii = 2; ii <= estar.pq.len; ++ii) {
    if (estar.pq.heap[ii]->key > maxkey) {
      maxkey = estar.pq.heap[ii]->key;
    }
  }
  
  maxrhs = 0.0;
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      cell = grid_at (&estar.grid, ii, jj);
      if (cell->rhs == cell->phi && cell->rhs <= topkey && maxrhs < cell->rhs) {
	maxrhs = cell->rhs;
      }
    }
  }
  if (maxrhs == 0.0) {
    maxrhs = 0.0001;		/* avoid potential div by zero */
  }
  
  if (dbg) {
    printf ("cb_phi_expose:\n"
	    "  topkey %4g\n"
	    "  maxkey %4g\n"
	    "  maxrhs %4g\n",
	    topkey, maxkey, maxrhs);
  }
  
  cr = gdk_cairo_create (ee->window);
  
  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0);
  cairo_rectangle (cr, 0, 0, w_phi_width, w_phi_height);
  cairo_fill (cr);
  
  cairo_set_source_rgb (cr, 0.5, 0.5, 0.5);
  cairo_set_line_width (cr, 2.0);
  cairo_rectangle (cr, w_phi_x0 - 2, w_phi_y0 + 2, DIMX * w_phi_sx + 4, DIMY * w_phi_sy - 4);
  cairo_stroke (cr);
  
  //////////////////////////////////////////////////
  // filled squares to indicate phi or key
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      cell_t * cell = grid_at (&estar.grid, ii, jj);
      
      if (isinf(cell->rhs)) {
	// at infinity: indicate with blue
	cairo_set_source_rgb (cr, 0.0, 0.0, 0.5);
      }
      else if (cell->rhs == cell->phi) {
	// consistent
	if (isinf(topkey)) {
	  // empty queue
	  double const vv = 1.0 - cell->rhs / maxrhs;
	  cairo_set_source_rgb (cr, vv, vv, vv);
	}
	else {
	  if (cell->rhs <= maxrhs) {
	    // valid: green
	    double const vv = 1.0 - cell->rhs / maxrhs;
	    cairo_set_source_rgb (cr, 0.0, vv, 0.0);
	  }
	  else {
	    // pending: yellow
	    cairo_set_source_rgb (cr, 0.8, 0.8, 0.0);
	  }
	}
      }
      else {
	// inconsistent
	if (cell->pqi == 0) {
	  // but not on queue: ERROR!
	  // (this should by now be caught by checks elsewhere though)
	  printf ("*** ERROR there is an inconsistent cell which is not on the queue.\n");
	  cairo_set_source_rgb (cr, 0.0, 1.0, 1.0);
	  play = 0;
	}
	else {
	  // on queue: red
	  if (maxkey == topkey) {
	    cairo_set_source_rgb (cr, 1.0, 0.5, 0.0);
	  }
	  else {
	    double const vv = 1.0 - (cell->key - topkey) / (maxkey - topkey);
	    cairo_set_source_rgb (cr, 0.2 + 0.8 * vv, 0.0, 0.0);
	  }
	}
      }
      cairo_rectangle (cr,
		       w_phi_x0 + ii * w_phi_sx,
		       w_phi_y0 + (jj+1) * w_phi_sy,
		       w_phi_sx,
		       - w_phi_sy);
      cairo_fill (cr);
    }
  }
  
  //////////////////////////////////////////////////
  // frame to indicate flags
  
  cairo_set_line_width (cr, 1.0);
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      cell_t * cell = grid_at (&estar.grid, ii, jj);
      
      if (cell->flags & FLAG_GOAL) {
	// goal: green (unless overridden below)
	cairo_set_source_rgb (cr, 0.0, 1.0, 0.0);
      }
      else if (cell->cost > 1.0) {
	// obstacle: red
	cairo_set_source_rgb (cr, 1.0, 0.0, 0.0);
      }
      else {
	continue;
      }
      
      cairo_rectangle (cr,
		       w_phi_x0 + (ii+0.1) * w_phi_sx,
		       w_phi_y0 + (jj+0.9) * w_phi_sy,
		       0.8 * w_phi_sx,
		       - 0.8 * w_phi_sy);
      cairo_stroke (cr);
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
  gdouble const cx = (bb->x - w_phi_x0) / w_phi_sx - 0.5;
  gdouble const cy = (bb->y - w_phi_y0) / w_phi_sy - 0.5;
  int const ix = (int) rint (cx);
  int const iy = (int) rint (cy);
  cell_t * cell;
  cell_t ** nbor;
  
  if (ix >= 0 && ix < DIMX && iy >= 0 && iy < DIMY) {
    if (dbg) { printf ("click [%4d  %4d]\n", ix, iy); }
    cell = grid_at (&estar.grid, ix, iy);
    if (cell->cost  < 2.0 ) {
      cell->cost = 100.0;
    }
    else {
      cell->cost = 1.0;
    }
    estar_update_cell (&estar, cell);
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update_cell (&estar, *nbor);
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
  
  btn = gtk_button_new_with_label ("flush");
  g_signal_connect (btn, "clicked", G_CALLBACK (cb_flush), NULL);
  gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show (btn);
  
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