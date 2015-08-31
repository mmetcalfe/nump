#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include <armadillo>
#include "nump.h"
#include "shared/utility/drawing/SearchTreeDrawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "tests.h"

using utility::math::geometry::Ellipse;
using shared::utility::drawing::drawSearchTree;
using shared::utility::drawing::drawRRBT;
using shared::utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;

arma::arma_rng::seed_type randomSeed() {
    arma::arma_rng::set_seed_random();
    arma::vec randvec = arma::randu(1);
    arma::arma_rng::seed_type seed = floor(randvec(0) * 123456789);
    arma::arma_rng::set_seed(seed);
    return seed;
}

int main() {
    // Run the tests:
    trajectoryTests();
    intersectionTests();
    propogateTests();

    arma::ivec2 surfaceDimensions = {500, 500};
    cairo_surface_t *surface = cairo_pdf_surface_create("output.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);

    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    // Get random seed:
    int seed = randomSeed();
    arma::arma_rng::set_seed(seed);
    std::cout << "SEED: " << seed << std::endl;

    // Setup problem:
//    Transform2D start = {0, 0, 0};
//    Transform2D goal  = {1, 1, 0};
    arma::vec2 start = {0, 0};
    arma::vec2 goal  = {1, 1};

    int numPoints = 500;
    std::vector<nump::math::Circle> obstacles;
    obstacles.push_back({{0.6, -0.9}, 1.0});
//    obstacles.push_back({{0.2, 0.2}, 0.1});
    obstacles.push_back({{0.5, 0.5}, 0.2});
//    obstacles.push_back({{0.8, 0.5}, 0.25});
    obstacles.push_back({{0.4, 1.5}, 0.7});
//
//    // Run RRT:
//    arma::arma_rng::set_seed(seed);
//    auto rrtTree = nump::SearchTree::fromRRT(cr, start, goal, numPoints, obstacles);
//
//    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//    drawSearchTree(cr, rrtTree);
//    cairo_show_page(cr);
//    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
////    cairo_push_group(cr);
//
//    // Run RRT*:
//    arma::arma_rng::set_seed(seed);
//    auto rrtsTree = nump::SearchTree::fromRRTs(cr, start, goal, numPoints, obstacles, [cr](const nump::SearchTree& tree, const nump::SearchTree::StateT newState, bool extended){
////        if (!extended) {
////            return;
////        }
////        cairo_pattern_t *group = cairo_pop_group(cr);
////
//////        fillCircle(cr, newState.rows(0,1), 0.025, {1, 0, 0});
////        drawSearchTree(cr, tree);
////
////        cairo_set_source(cr, group);
////        cairo_paint(cr);
////        cairo_pattern_destroy(group);
////
////        cairo_show_page(cr);
////        cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
////        cairo_push_group(cr);
//    });
//
////    cairo_pop_group_to_source(cr);
//
//    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//    drawSearchTree(cr, rrtsTree);
//    cairo_show_page(cr);


    // Run RRBT:
    arma::arma_rng::set_seed(seed);
    nump::RRBT::StateCovT initCov = arma::diagmat(arma::vec({0.001, 0.001}));
//    nump::RRBT::StateCovT initCov = arma::mat({{0.05, 0.01},{0.01,0.05}});
    auto rrbtTree = nump::RRBT::fromRRBT(cr, start, initCov, goal, numPoints, obstacles, [cr](const nump::RRBT& tree, const nump::RRBT::StateT newState, bool extended){
    });
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
    drawRRBT(cr, rrbtTree);
    cairo_show_page(cr);

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return 0;
}
