#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include <armadillo>
#include "nump.h"
#include "shared/utility/drawing/SearchTreeDrawing.h"

using shared::utility::drawing::drawSearchTree;
using shared::utility::drawing::fillCircle;
using nump::Transform2D;

arma::arma_rng::seed_type randomSeed() {
    arma::arma_rng::set_seed_random();
    arma::vec randvec = arma::randu(1);
    arma::arma_rng::seed_type seed = floor(randvec(0) * 123456789);
    arma::arma_rng::set_seed(seed);
    return seed;
}

void trajectoryTests() {
    std::cout << "TrajectoryTests: BEGIN" << std::endl;

    // Create surface:
    arma::ivec2 surfaceDimensions = {500, 500};
    cairo_surface_t *surface = cairo_pdf_surface_create("trajectoryTests.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);
    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    // White background:
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);


    // Get random seed:
    int seed = randomSeed();
    arma::arma_rng::set_seed(seed);
    std::cout << "  SEED: " << seed << std::endl;

    double size = 0.2;

    Transform2D x1 = {0.5, 0.5, 1.5*M_PI}; //nump::SearchTree::TrajT::sample();

    shared::utility::drawing::cairoSetSourceRGB(cr, {0.5, 0.5, 0.5});
    shared::utility::drawing::drawRobot(cr, x1, size);

    int numTrials = 10;
    int numSteps = 200;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

//        Transform2D x1 = nump::SearchTree::TrajT::sample();
        nump::SearchTree::StateT x2 = nump::SearchTree::TrajT::sample();
        nump::SearchTree::TrajT x = nump::SearchTree::steer(x1, x2);


//        shared::utility::drawing::cairoSetSourceRGB(cr, col * 0.5);
//        shared::utility::drawing::drawRobot(cr, x1, size);
        shared::utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.5);
        shared::utility::drawing::drawRobot(cr, x2, size);

        if (!x.reachesTarget) {
            cairo_set_source_rgb (cr, 1, 0.0, 0.0);
            shared::utility::drawing::drawRobot(cr, x1, size);
            cairo_set_source_rgb (cr, 1, 0.5, 0.5);
            shared::utility::drawing::drawRobot(cr, x2, size);
//            continue;
        }

        // Path colour:
        arma::vec3 pathCol = col * 0.75;
        if (x.reachesTarget) {
            pathCol = col * 0.75;
        } else {
            pathCol = { 0.5, 0.2, 0.2 };
        }

        double timeStep = size*0.25;
        for (double t = 0; t < x.t; t += timeStep) {
            Transform2D pos = x(t);

            shared::utility::drawing::cairoSetSourceRGBAlpha(cr, pathCol, 0.5);
            shared::utility::drawing::drawRobot(cr, pos, size * 0.2);
            shared::utility::drawing::cairoSetSourceRGBAlpha(cr, pathCol * 0.5, 0.5);
            shared::utility::drawing::showText(cr, pos.rows(0,1), size * 0.1, t);
        }

//        shared::utility::drawing::cairoSetSourceRGB(cr, pathCol);
        shared::utility::drawing::cairoSetSourceRGB(cr, {0,0,0});
        shared::utility::drawing::drawRobot(cr, x(x.t), size * 0.2);
        shared::utility::drawing::cairoSetSourceRGB(cr, {0.8,0.8,0.8});
        shared::utility::drawing::showText(cr, x(x.t).rows(0,1), size * 0.1, x.t, ", ", x.reachesTarget);
    }

    cairo_show_page(cr);

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "TrajectoryTests: END" << std::endl << std::endl;
}


int main() {

//    trajectoryTests();

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

    // Run RRT:
    arma::arma_rng::set_seed(seed);
    auto rrtTree = nump::SearchTree::fromRRT(cr, start, goal, numPoints, obstacles);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
    drawSearchTree(cr, rrtTree);
    cairo_show_page(cr);
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
    cairo_push_group(cr);

    // Run RRT*:
    arma::arma_rng::set_seed(seed);
    auto rrtsTree = nump::SearchTree::fromRRTs(cr, start, goal, numPoints, obstacles, [cr](const nump::SearchTree& tree, const nump::SearchTree::StateT newState, bool extended){
//        if (!extended) {
//            return;
//        }
//        cairo_pattern_t *group = cairo_pop_group(cr);
//
////        fillCircle(cr, newState.rows(0,1), 0.025, {1, 0, 0});
//        drawSearchTree(cr, tree);
//
//        cairo_set_source(cr, group);
//        cairo_paint(cr);
//        cairo_pattern_destroy(group);
//
//        cairo_show_page(cr);
//        cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);
//        cairo_push_group(cr);
    });

    cairo_pop_group_to_source(cr);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint_with_alpha (cr, 1);

    drawSearchTree(cr, rrtsTree);
    cairo_show_page(cr);


    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
    return 0;
}
