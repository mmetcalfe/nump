//
// Created by Mitchell Metcalfe on 31/08/2015.
//

#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include <armadillo>
#include "nump.h"
#include "shared/utility/drawing/cairo_drawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "tests.h"

using utility::math::geometry::Ellipse;
using utility::drawing::drawSearchTree;
using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;

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
    std::cout << "  SEED: " << seed << std::endl;

    double size = 0.2;

    nump::SearchTree::StateT x1 = {{0.5, 0.5, 1.5*M_PI}}; //nump::SearchTree::TrajT::sample();
//    arma::vec2 x1 = {0.5, 0.5}; //nump::SearchTree::TrajT::sample();

    utility::drawing::cairoSetSourceRGB(cr, {0.5, 0.5, 0.5});
    utility::drawing::drawRobot(cr, x1.position, size);

    int numTrials = 10;
    int numSteps = 200;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

//        Transform2D x1 = nump::SearchTree::TrajT::sample();
        nump::SearchTree::StateT x2 = nump::SearchTree::TrajT::sample({1, 1});
        nump::SearchTree::TrajT x = nump::SearchTree::steer(x1, x2);


//        utility::drawing::cairoSetSourceRGB(cr, col * 0.5);
//        utility::drawing::drawRobot(cr, x1, size);
        utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.5);
        utility::drawing::drawRobot(cr, x2.position, size);

        if (!x.reachesTarget) {
            cairo_set_source_rgb (cr, 1, 0.0, 0.0);
            utility::drawing::drawRobot(cr, x1.position, size);
            cairo_set_source_rgb (cr, 1, 0.5, 0.5);
            utility::drawing::drawRobot(cr, x2.position, size);
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
            nump::SearchTree::StateT pos = x(t);

            utility::drawing::cairoSetSourceRGBAlpha(cr, pathCol, 0.5);
            utility::drawing::drawRobot(cr, pos.position, size * 0.2);
            utility::drawing::cairoSetSourceRGBAlpha(cr, pathCol * 0.5, 0.5);
            utility::drawing::showText(cr, pos.position.rows(0,1), size * 0.1, t);
        }

//        utility::drawing::cairoSetSourceRGB(cr, pathCol);
        utility::drawing::cairoSetSourceRGB(cr, {0,0,0});
        utility::drawing::drawRobot(cr, x(x.t).position, size * 0.2);
        utility::drawing::cairoSetSourceRGB(cr, {0.8,0.8,0.8});
        utility::drawing::showText(cr, x(x.t).position.rows(0,1), size * 0.1, x.t, ", ", x.reachesTarget);
    }

    cairo_show_page(cr);

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "TrajectoryTests: END" << std::endl << std::endl;
}
