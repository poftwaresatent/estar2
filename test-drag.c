#include <gtk/gtk.h>
#include <err.h>
#include <math.h>

#define DIMX 5
#define DIMY 6

static GtkWidget * w_phi;
static gint w_phi_width = 500;
static gint w_phi_height = 320;
static gint w_phi_sx, w_phi_sy, w_phi_x0, w_phi_y0;

static int lastx = -1;
static int lasty = -1;


static void cb_quit(GtkWidget * ww, gpointer data)
{
  gtk_main_quit();
}


static gint cb_expose(GtkWidget * ww,
		      GdkEventExpose * ee,
		      gpointer data)
{
  int ii, jj;
  cairo_t * cr;
  
  cr = gdk_cairo_create(ee->window);
  
  cairo_set_source_rgb (cr, 1.0, 1.0, 1.0);
  cairo_rectangle (cr, 0, 0, w_phi_width, w_phi_height);
  cairo_fill (cr);
  
  cairo_set_source_rgb (cr, 0.5, 0.5, 0.5);
  cairo_set_line_width (cr, 2.0);
  cairo_rectangle (cr, w_phi_x0 - 2, w_phi_y0 + 2, DIMX * w_phi_sx + 4, DIMY * w_phi_sy - 4);
  cairo_stroke (cr);
  
  for (ii = 0; ii < DIMX; ++ii) {
    for (jj = 0; jj < DIMY; ++jj) {
      if (ii == lastx && jj == lasty) {
	cairo_set_source_rgb (cr, 0.5, 0.5, 0.0);
      }
      else if (0 == (ii + jj) % 2) {
	cairo_set_source_rgb (cr, 0.5, 0.0, 0.0);
      }
      else {
	cairo_set_source_rgb (cr, 0.0, 0.5, 0.0);
      }
      cairo_rectangle (cr,
		       w_phi_x0 + ii * w_phi_sx,
		       w_phi_y0 + (jj+1) * w_phi_sy,
		       w_phi_sx,
		       - w_phi_sy);
      cairo_fill (cr);
    }
  }
  
  cairo_destroy(cr);
  
  return TRUE;
}


static gint cb_size_allocate(GtkWidget * ww,
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
  
  return TRUE;
}


static gint cb_click(GtkWidget * ww,
		     GdkEventButton * bb,
		     gpointer data)
{
  lastx = (int) rint ((bb->x - w_phi_x0) / w_phi_sx - 0.5);
  lasty = (int) rint ((bb->y - w_phi_y0) / w_phi_sy - 0.5);
  
  printf ("[%2d %2d] cb_click:  t: %d  x: %f  y: %f", lastx, lasty, bb->type, bb->x, bb->y);
  if (bb->type == GDK_BUTTON_PRESS) {
    printf ("  press\n");
  }
  else if (bb->type == GDK_BUTTON_RELEASE) {
    printf ("  release\n");
  }
  else {
    printf ("  other\n");
  }
  return TRUE;
}


static gint cb_motion(GtkWidget * ww,
		      GdkEventMotion * ee)
{
  int mx, my;
  GdkModifierType modifier;
  gdk_window_get_pointer(ww->window, &mx, &my, &modifier);
  
  lastx = (int) rint (((double)mx - w_phi_x0) / w_phi_sx - 0.5);
  lasty = (int) rint (((double)my - w_phi_y0) / w_phi_sy - 0.5);
  
  printf ("[%2d %2d] cb_motion:  m: %d  x: %d  y: %d\n", lastx, lasty, modifier, mx, my);
  
  return TRUE;
}


static gint idle(gpointer data)
{
  gtk_widget_queue_draw(w_phi);
  return TRUE;
}


static void init_gui(int * argc, char *** argv)
{
  GtkWidget *window, *vbox, *hbox, *btn;
  
  gtk_init(argc, argv);
  
  window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  
  vbox = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER (window), vbox);
  gtk_widget_show(vbox);
  
  w_phi = gtk_drawing_area_new();
  g_signal_connect(w_phi, "expose_event", G_CALLBACK (cb_expose), NULL);
  g_signal_connect(w_phi, "size_allocate", G_CALLBACK (cb_size_allocate), NULL);
  g_signal_connect(w_phi, "button_press_event", G_CALLBACK (cb_click), NULL);
  g_signal_connect(w_phi, "button_release_event", G_CALLBACK (cb_click), NULL);
  g_signal_connect(w_phi, "motion_notify_event", G_CALLBACK (cb_motion), NULL);
  gtk_widget_set_events(w_phi,
			GDK_BUTTON_PRESS_MASK |
			GDK_BUTTON_RELEASE_MASK |
			GDK_BUTTON_MOTION_MASK);
  
  gtk_widget_show(w_phi);
  
  gtk_widget_set_size_request(w_phi, w_phi_width, w_phi_height);
  gtk_box_pack_start(GTK_BOX (vbox), w_phi, TRUE, TRUE, 0);
  
  hbox = gtk_hbox_new(TRUE, 3);
  gtk_box_pack_start(GTK_BOX (vbox), hbox, FALSE, TRUE, 0);
  gtk_widget_show(hbox);
  
  btn = gtk_button_new_with_label("quit");
  g_signal_connect(btn, "clicked", G_CALLBACK (cb_quit), NULL);
  gtk_box_pack_start(GTK_BOX (hbox), btn, TRUE, TRUE, 0);
  gtk_widget_show(btn);
  
  gtk_idle_add(idle, 0);
  
  gtk_widget_show(window);
}


int main(int argc, char ** argv)
{
  init_gui(&argc, &argv);
  gtk_main();
  return 0;
}
