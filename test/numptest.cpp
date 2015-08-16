#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include <armadillo>
#include "nump.h"
#include "shared/utility/drawing/SearchTreeDrawing.h"

using shared::utility::drawing::drawSearchTree;
using shared::utility::drawing::drawCircle;

int main() {
    arma::ivec2 surfaceDimensions = {100, 100};
    cairo_surface_t *surface = cairo_pdf_surface_create("output.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);

    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    // Get random seed:
    arma::arma_rng::set_seed_random();
    arma::vec randvec = arma::randu(1);
    int seed = floor(randvec(0) * 10000);
    std::cout << "SEED: " << seed << " (vec: " << randvec(0) << ")" << std::endl;

    // Setup problem:
    arma::vec2 start = {0, 0};
    arma::vec2 goal  = {1, 1};

    int numPoints = 100;
    std::vector<nump::math::Circle> obstacles;
    obstacles.push_back({{0.6, -0.9}, 1.0});
//    obstacles.push_back({{0.2, 0.2}, 0.1});
    obstacles.push_back({{0.5, 0.5}, 0.2});
//    obstacles.push_back({{0.8, 0.5}, 0.25});
    obstacles.push_back({{0.4, 1.5}, 0.7});

    // Run RRT:
    arma::arma_rng::set_seed(seed);
    auto rrtTree = nump::SearchTree::fromRRT(cr, start, goal, numPoints, obstacles);
    drawSearchTree(cr, rrtTree);
    cairo_show_page(cr);

    // Run RRT*:
    arma::arma_rng::set_seed(seed);
    auto rrtsTree = nump::SearchTree::fromRRTs(cr, start, goal, numPoints, obstacles, [cr](const nump::SearchTree& tree, const arma::vec2 newState, bool extended){
        if (!extended) {
            return;
        }
        drawCircle(cr, newState, 0.025, {1, 0, 0});
        drawSearchTree(cr, tree);
        cairo_show_page(cr);
    });
    drawSearchTree(cr, rrtsTree);
    cairo_show_page(cr);

//    cairo_select_font_face(cr, "serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
//    cairo_set_font_size(cr, 32.0);
//    cairo_set_source_rgb(cr, 0.0, 0.0, 1.0);
//    cairo_move_to(cr, 10.0, 50.0);
//    cairo_show_text(cr, "Hello, world");

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return 0;
}
