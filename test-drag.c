#include <gtk/gtk.h>
#include <err.h>

#define DIMX 50
#define DIMY 50

static GtkWidget * gw;
static gint gw_width = 500;
static gint gw_height = 320;
static gint gw_sx, gw_sy, gw_x0, gw_y0;


static void cb_quit(GtkWidget * ww, gpointer data)
{
  gtk_main_quit();
}


static gint cb_expose(GtkWidget * ww,
		      GdkEventExpose * ee,
		      gpointer data)
{
  cairo_t * cr = gdk_cairo_create(ee->window);
  
  cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
  cairo_set_source_rgb(cr, 1.0, 1.0, 1.0);
  cairo_rectangle(cr, 0, 0, gw_width, gw_height);
  cairo_fill(cr);
  
  cairo_destroy(cr);
  
  return TRUE;
}


static gint cb_size_allocate(GtkWidget * ww,
			     GtkAllocation * aa,
			     gpointer data)
{
  gw_width = aa->width;
  gw_height = aa->height;
  
  gw_sx = gw_width / DIMX;
  if (gw_sx < 1) {
    gw_sx = 1;
  }
  gw_sy = - gw_height / DIMY;
  if ( - gw_sy < 1) {
    gw_sy = -1;
  }
  if (gw_sx > - gw_sy) {
    gw_sx = - gw_sy;
  }
  else {
    gw_sy = - gw_sx;
  }
  gw_x0 = (gw_width - DIMX * gw_sx) / 2;
  gw_y0 = gw_height - (gw_height + DIMY * gw_sy) / 2;
  
  return TRUE;
}


static gint cb_click(GtkWidget * ww,
		     GdkEventButton * bb,
		     gpointer data)
{
  printf ("cb_click:  t: %d  x: %f  y: %f", bb->type, bb->x, bb->y);
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
  printf ("cb_motion:  m: %d  x: %d  y: %d\n", modifier, mx, my);
  return TRUE;
}


static gint idle(gpointer data)
{
  gtk_widget_queue_draw(gw);
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
  
  gw = gtk_drawing_area_new();
  g_signal_connect(gw, "expose_event", G_CALLBACK (cb_expose), NULL);
  g_signal_connect(gw, "size_allocate", G_CALLBACK (cb_size_allocate), NULL);
  g_signal_connect(gw, "button_press_event", G_CALLBACK (cb_click), NULL);
  g_signal_connect(gw, "button_release_event", G_CALLBACK (cb_click), NULL);
  g_signal_connect(gw, "motion_notify_event", G_CALLBACK (cb_motion), NULL);
  gtk_widget_set_events(gw,
			GDK_BUTTON_PRESS_MASK |
			GDK_BUTTON_RELEASE_MASK |
			GDK_BUTTON_MOTION_MASK);
  
  gtk_widget_show(gw);
  
  gtk_widget_set_size_request(gw, gw_width, gw_height);
  gtk_box_pack_start(GTK_BOX (vbox), gw, TRUE, TRUE, 0);
  
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
