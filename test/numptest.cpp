#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include "../src/nump.h"

using namespace std;

int main() {
    cairo_surface_t *surface = cairo_pdf_surface_create("output.pdf", 500, 500);
    cairo_t *cr = cairo_create(surface);

//    cairo_select_font_face(cr, "serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
//    cairo_set_font_size(cr, 32.0);
//    cairo_set_source_rgb(cr, 0.0, 0.0, 1.0);
//    cairo_move_to(cr, 10.0, 50.0);
//    cairo_show_text(cr, "Hello, world");

    cairo_show_page(cr);
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    return 0;
}
