#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include <armadillo>
#include "nump.h"
#include "shared/utility/drawing/SearchTreeDrawing.h"

using shared::utility::drawing::drawSearchTree;

int main() {

    arma::vec2 start = {0, 0};
    arma::vec2 goal  = {1, 1};
    auto tree = nump::SearchTree::fromRRT(start, goal, 100);

    arma::ivec2 surfaceDimensions = {100, 100};
    cairo_surface_t *surface = cairo_pdf_surface_create("output.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);

    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    drawSearchTree(cr, tree);


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
