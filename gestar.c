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
#include "dstar.h"

#include <gtk/gtk.h>
#include <err.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#define DIMX 50
#define DIMY 50
#define ODIST 3
#define GOALX 15
#define GOALY 15
#define STARTX 30
#define STARTY 30


static estar_t estar;
static dstar_t dstar;

static GtkWidget * w_phi;
static gint w_phi_width, w_phi_height;
static gint w_phi_sx, w_phi_sy, w_phi_x0, w_phi_y0;
static int play, dbg, mousex, mousey, drag;


static void fini ()
{
  dstar_fini (&dstar);
  estar_fini (&estar);
}


// Strictly speaking, there should be two of these, one for estar and one for dstar.
static double hfunc (size_t elem)
{
  double dx = grid_ix (&estar.grid, elem) - (double) STARTX;
  double dy = grid_iy (&estar.grid, elem) - (double) STARTY;
  return sqrt (dx * dx + dy * dy);
}


static double compute_obound ()
{
  if (-1 == dstar_compute_path (&dstar, STARTX, STARTY)) {
    if (dbg) { printf ("compute_obound found no path\n"); }
    return INFINITY;
  }
  if (dbg) { printf ("compute_obound %g\n", dstar.phi[grid_elem (&dstar.grid, STARTX, STARTY)]); }
  return dstar.phi[grid_elem (&dstar.grid, STARTX, STARTY)];
}


static void init ()
{
  estar_init (&estar, DIMX, DIMY, hfunc);
  dstar_init (&dstar, DIMX, DIMY, hfunc);
  
  // It is important to first let D* know about the goal, because
  // compute_bound uses that information.
  
  dstar_set_goal (&dstar, GOALX, GOALY);
  estar_set_goal (&estar, GOALX, GOALY, compute_obound ());
  
  play = 0;
  dbg = 1;
  mousex = -1;
  mousey = -1;
  drag = 0;
  
  if (dbg) {
    printf ("  initialized\n");
    estar_dump_queue (&estar, "  ");
  }
}


static void update ()
{
  int status;
  
  if (drag > 0 || estar.pq.len == 0) {
    return;
  }
  
  estar_propagate (&estar);
  
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
    estar_propagate (&estar);
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
    printf ("PAUSE\n");
  }
  else {
    update ();
  }    
}


void cb_quit (GtkWidget * ww, gpointer data)
{
  play = 0;
  printf ("QUIT\n");
  gtk_main_quit();
}


gint cb_phi_expose (GtkWidget * ww,
		    GdkEventExpose * ee,
		    gpointer data)
{
  size_t ii, jj;
  cairo_t * cr;
  double topkey, maxknown, maxoverall;
  size_t elem;
  
  topkey = pqueue_topkey (&estar.pq);
  maxknown = 0.0;
  maxoverall = 0.0;
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      elem = grid_elem (&estar.grid, ii, jj);
      if (estar.rhs[elem] == estar.phi[elem] && isfinite(estar.rhs[elem])) {
	if (0 == pqueue_pos(&estar.pq, elem)
	    && estar.rhs[elem] <= topkey
	    && maxknown < estar.rhs[elem]) {
	  maxknown = estar.rhs[elem];
	}
	if (maxoverall < estar.rhs[elem]) {
	  maxoverall = estar.rhs[elem];
	}
      }
    }
  }
  if (maxknown == 0.0) {
    maxknown = 0.0001;
  }
  if (maxoverall == 0.0) {
    maxoverall = 0.0001;
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
  // filled squares
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      elem = grid_elem (&estar.grid, ii, jj);
      
      if (estar.flags[elem] & ESTAR_FLAG_OBSTACLE) {
	cairo_set_source_rgb (cr, 0.0, 0.0, 0.0);
      }
      else {
	double red, green, blue;
	red = 1.0 - 1.0 / estar.cost[elem];
	if (isinf(estar.rhs[elem])) { /* unreached / unreachable */
	  green = 0.0;
	  blue = 0.5;
	}
	else if (estar.rhs[elem] <= maxknown) { /* known */
	  blue = 1.0 - estar.rhs[elem] / maxknown;
	  if (0 != estar.pruned.pos[elem]) {
	    green = 0.5 * blue;
	  }
	  else {
	    green = blue;
	  }
	}
	else {			/* to be rediscovered */
	  green = 1.0 - estar.rhs[elem] / maxoverall;
	  blue = 0.0;
	}
	cairo_set_source_rgb (cr, red, green, blue);
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
  // frames
  
  cairo_set_line_width (cr, 1.0);
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      elem = grid_elem (&estar.grid, ii, jj);
      
      if (ii == STARTX && jj == STARTY) { /* start */
	cairo_set_source_rgb (cr, 0.0, 1.0, 1.0);
      }
      else if (estar.flags[elem] & ESTAR_FLAG_GOAL) { /* goal */
	cairo_set_source_rgb (cr, 0.0, 1.0, 1.0);
      }
      else if (0 != pqueue_pos (&estar.pq, elem)) { /* on queue */
	cairo_set_source_rgb (cr, 1.0, 0.5, 0.0);
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

  //////////////////////////////////////////////////
  // focussing annotations
  
  cairo_set_line_width (cr, 1.0);
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      elem = grid_elem (&estar.grid, ii, jj);
      
      if (dstar.flags[elem] & DSTAR_FLAG_PATH) {
	cairo_set_source_rgb (cr, 0.0, 1.0, 0.5);
	cairo_arc (cr,
		   w_phi_x0 + (ii+0.5) * w_phi_sx,
		   w_phi_y0 + (jj+0.5) * w_phi_sy,
		   0.35 * w_phi_sx,
		   0.0,
		   2.0 * M_PI);
	cairo_stroke (cr);
      }
      
      if (0 != estar.pruned.pos[elem]) {
	cairo_set_source_rgb (cr, 1.0, 0.0, 0.5);
	cairo_move_to (cr,
		       w_phi_x0 + (ii+0.2) * w_phi_sx,
		       w_phi_y0 + (jj+0.2) * w_phi_sy);
	cairo_line_to (cr,
		       w_phi_x0 + (ii+0.8) * w_phi_sx,
		       w_phi_y0 + (jj+0.8) * w_phi_sy);
	cairo_move_to (cr,
		       w_phi_x0 + (ii+0.8) * w_phi_sx,
		       w_phi_y0 + (jj+0.2) * w_phi_sy);
	cairo_line_to (cr,
		       w_phi_x0 + (ii+0.2) * w_phi_sx,
		       w_phi_y0 + (jj+0.8) * w_phi_sy);
	cairo_stroke (cr);
      }
      
      if (estar.flags[elem] & ESTAR_FLAG_OBSTACLE) {
	cairo_set_source_rgb (cr, 1.0, 0.5, 1.0);
	cairo_arc (cr,
		   w_phi_x0 + (ii+0.5) * w_phi_sx,
		   w_phi_y0 + (jj+0.5) * w_phi_sy,
		   0.25 * w_phi_sx,
		   0.0,
		   2.0 * M_PI);
	cairo_fill (cr);
      }

    }
  }
  
  //////////////////////////////////////////////////
  // if available, trace the path from start to goal
  
  elem = grid_elem (&estar.grid, STARTX, STARTY);
  if (0 == pqueue_pos (&estar.pq, elem)
      && estar.rhs[elem] <= maxknown) {
    double px, py, dd, dmax, ds;
    px = STARTX;
    py = STARTY;
    dmax = 1.3 * estar.rhs[elem];
    
    ds = 0.1;
    for (dd = 0.0; dd <= dmax; dd += ds) {
      double gx, gy, gg;
      int ix, iy;
      if (0 == grid_calc_gradient (&estar.grid, estar.phi, elem, &gx, &gy)) {
	break;
      }
      gg = sqrt(pow(gx, 2.0) + pow(gy, 2.0));
      gx *= ds / gg;
      gy *= ds / gg;
      
      cairo_set_line_width (cr, 2.0);
      if (fmod(dd, 2.0) < 1.0) {
	cairo_set_source_rgb (cr, 1.0, 1.0, 1.0);
      }
      else {
	cairo_set_source_rgb (cr, 0.0, 0.0, 0.0);
      }
      cairo_move_to(cr, w_phi_x0 + (px + 0.5) * w_phi_sx, w_phi_y0 + (py + 0.5) * w_phi_sy);
      px += gx;
      py += gy;
      cairo_line_to(cr, w_phi_x0 + (px + 0.5) * w_phi_sx, w_phi_y0 + (py + 0.5) * w_phi_sy);
      cairo_stroke (cr);
      
      ix = (int) rint(px);
      iy = (int) rint(py);
      if (ix < 0 || ix >= DIMX || iy < 0 || iy >= DIMY) {
	break;
      }
      elem = grid_elem (&estar.grid, ix, iy);
      if (estar.flags[elem] & ESTAR_FLAG_GOAL) {
	break;
      }

      /* cairo_set_line_width (cr, 1.0); */
      /* if (fmod(dd, 2.0) < 1.0) { */
      /* 	cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); */
      /* } */
      /* else { */
      /* 	cairo_set_source_rgb (cr, 0.0, 0.0, 0.0); */
      /* } */
      /* cairo_rectangle (cr, */
      /* 		       w_phi_x0 + (ix + 0.1) * w_phi_sx, */
      /* 		       w_phi_y0 + (iy + 0.9) * w_phi_sy, */
      /* 		       0.8 * w_phi_sx, */
      /* 		       - 0.8 * w_phi_sy); */
      /* cairo_stroke (cr); */
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


static void change_obstacle (int cx, int cy, int dist, int add)
{
  int const dim = 2 * dist + 1;
  double md2[dim * dim];
  double * ptr;
  double d2;
  int x0, y0, x1, y1, x2, y2, x3, y3, ix, iy, jx, jy, nobst;
  
  x0 = cx - dist;
  if (0 > x0) {
    x0 = 0;
  }
  y0 = cy - dist;
  if (0 > y0) {
    y0 = 0;
  }
  x1 = cx + dist + 1;
  if (x1 > DIMX) {
    x1 = DIMX;
  }
  y1 = cy + dist + 1;
  if (y1 > DIMY) {
    y1 = DIMY;
  }
  
  x2 = cx - 2 * dist;
  if (0 > x2) {
    x2 = 0;
  }
  y2 = cy - 2 * dist;
  if (0 > y2) {
    y2 = 0;
  }
  x3 = cx + 2 * dist + 1;
  if (x3 > DIMX) {
    x3 = DIMX;
  }
  y3 = cy + 2 * dist + 1;
  if (y3 > DIMY) {
    y3 = DIMY;
  }
  
  nobst = 0;
  for (ix = x2; ix < x3; ++ix) {
    for (iy = y2; iy < y3; ++iy) {
      if (0 == add && ix == cx && iy == cy) {
	continue;
      }
      if ((0 != add && ix == cx && iy == cy)
	  || (estar.flags[grid_elem (&estar.grid, ix, iy)] & ESTAR_FLAG_OBSTACLE))
	{
	  ptr = md2;
	  for (jx = x0; jx < x1; ++jx) {
	    for (jy = y0; jy < y1; ++jy) {
	      d2 = pow (ix-jx, 2.0) + pow (iy-jy, 2.0);
	      if (0 == nobst || d2 < *ptr) {
		*ptr = d2;
	      }
	      ++ptr;
	    }
	  }
	  ++nobst;
	}
      else {
      }
    }
  }
  
  if (0 == nobst) {
    for (ix = x0; ix < x1; ++ix) {
      for (iy = y0; iy < y1; ++iy) {
	estar_set_speed (&estar, ix, iy, 1.0);
	dstar_set_speed (&dstar, ix, iy, 1.0);
      }
    }
  }
  else {
    ptr = md2;
    for (ix = x0; ix < x1; ++ix) {
      for (iy = y0; iy < y1; ++iy) {
	d2 = sqrt(*(ptr++)) - 0.5;
	if (d2 < 0) {
	  estar_set_speed (&estar, ix, iy, 0.0);
	  dstar_set_speed (&dstar, ix, iy, 0.0);
	}
	else if (d2 >= dist) {
	  estar_set_speed (&estar, ix, iy, 1.0);
	  dstar_set_speed (&dstar, ix, iy, 1.0);
	}
	else {
	  estar_set_speed (&estar, ix, iy, d2 / dist);
	  dstar_set_speed (&dstar, ix, iy, d2 / dist);
	}
      }
    }
  }
}


gint cb_phi_click (GtkWidget * ww,
		   GdkEventButton * bb,
		   gpointer data)
{
  gdouble const cx = (bb->x - w_phi_x0) / w_phi_sx - 0.5;
  gdouble const cy = (bb->y - w_phi_y0) / w_phi_sy - 0.5;
  mousex = (int) rint (cx);
  mousey = (int) rint (cy);
  size_t elem;
  
  if (bb->type == GDK_BUTTON_RELEASE) {
    drag = 0;
    return TRUE;
  }
  
  if (mousex >= 0 && mousex < DIMX && mousey >= 0 && mousey < DIMY) {
    elem = grid_elem (&estar.grid, mousex, mousey);
    if (estar.flags[elem] & ESTAR_FLAG_GOAL) {
      return TRUE;
    }
    if (estar.flags[elem] & ESTAR_FLAG_OBSTACLE) {
      drag = -1;
      change_obstacle (mousex, mousey, ODIST, 0);
      if (dbg) {
	printf ("click: removed obstacle at %d %d\n", mousex, mousey);
	estar_dump_queue (&estar, "  ");
      }
    }
    else {
      drag = -2;
      change_obstacle (mousex, mousey, ODIST, 1);
      if (dbg) {
	printf ("click: added obstacle at %d %d\n", mousex, mousey);
	estar_dump_queue (&estar, "  ");
      }
    }
    estar.ubound = compute_obound ();
    gtk_widget_queue_draw (w_phi);
    
    if (dbg) {
      int status = estar_check (&estar, "+++ ");
      if (0 != status) {
	play = 0;
	printf ("ERROR %d (see above)\n", status);
      }
    }
  }
  
  return TRUE;			// TRUE to stop event propagation
}


static gint cb_phi_motion(GtkWidget * ww,
			  GdkEventMotion * ee)
{
  int mx, my;
  GdkModifierType modifier;
  size_t elem;
  
  gdk_window_get_pointer(ww->window, &mx, &my, &modifier);
  mx = (int) rint (((double)mx - w_phi_x0) / w_phi_sx - 0.5);
  my = (int) rint (((double)my - w_phi_y0) / w_phi_sy - 0.5);
  
  if (mx == mousex && my == mousey) {
    return TRUE;
  }
  mousex = mx;
  mousey = my;
  
  if (drag < 0) {
    drag = -drag;
  }
  
  if (mousex >= 0 && mousex < DIMX && mousey >= 0 && mousey < DIMY) {
    elem = grid_elem (&estar.grid, mousex, mousey);
    if (estar.flags[elem] & ESTAR_FLAG_GOAL) {
      return TRUE;
    }
    if (drag == 1) {
      change_obstacle (mousex, mousey, ODIST, 0);
    }
    else {
      change_obstacle (mousex, mousey, ODIST, 1);
    }
    estar.ubound = compute_obound ();
    gtk_widget_queue_draw (w_phi);
  }
  
  return TRUE;
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
  g_signal_connect (w_phi, "button_release_event", G_CALLBACK (cb_phi_click), NULL);
  g_signal_connect (w_phi, "motion_notify_event", G_CALLBACK (cb_phi_motion), NULL);
  gtk_widget_set_events (w_phi,
			 GDK_BUTTON_PRESS_MASK |
			 GDK_BUTTON_RELEASE_MASK |
			 GDK_BUTTON_MOTION_MASK);
  
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
