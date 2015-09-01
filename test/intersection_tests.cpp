//
// Created by Mitchell Metcalfe on 31/08/2015.
//

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

void intersectionTests() {
    std::cout << "IntersectionTests: BEGIN" << std::endl;

    // Get random seed:
    int seed = randomSeed();
    std::cout << "  SEED: " << seed << std::endl;

    // Create surface:
    arma::ivec2 surfaceDimensions = {500, 500};
    cairo_surface_t *surface = cairo_pdf_surface_create("intersectionTests.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);
    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    // White background:
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);

    // Set the robot's state and uncertainty:
    arma::vec2 state = {0.5, 0.5};
    arma::mat22 stateCov = {
            { 0.025,  0.01},
            { 0.01,  0.01}
    };
    Ellipse confEllipse = Ellipse::forConfidenceRegion(state, stateCov * 0.5);

    int numTrials = 5000;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

        arma::vec randCircle = arma::randu(3);

//        Circle circle = {randCircle.rows(0, 1), 0.01};
        Circle circle = {randCircle.rows(0, 1), 0.05 + randCircle(2)*0.2};

        cairo_set_line_width(cr, 0.002);
        shared::utility::drawing::drawCircle(cr, circle);

        // TODO: Write a circle-ellipse intersection test.
        bool intersects = utility::math::geometry::intersection::test(circle, confEllipse);
        if (intersects) {
            shared::utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.1);
            cairo_stroke(cr);
        } else {
            shared::utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.7);
            cairo_fill(cr);
        }
    }

    // Draw confidence region of the robot:
    cairo_set_line_width(cr, 0.02);
    shared::utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    shared::utility::drawing::drawEllipse(cr, confEllipse);

    shared::utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    shared::utility::drawing::drawRotatedRectangle(cr, confEllipse);

    // Add page to PDF:
    cairo_show_page(cr);

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "IntersectionTests: END" << std::endl << std::endl;
}
