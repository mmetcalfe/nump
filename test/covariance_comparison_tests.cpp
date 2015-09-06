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
#include "RRBT.h"

using utility::math::geometry::Ellipse;
using shared::utility::drawing::drawSearchTree;
using shared::utility::drawing::drawRRBT;
using shared::utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;

void covarianceComparisonTests() {
    std::cout << "covarianceComparisonTests: BEGIN" << std::endl;

    // Get random seed:
    int seed = randomSeed();
    std::cout << "  SEED: " << seed << std::endl;

    // Create surface:
    arma::ivec2 surfaceDimensions = {500, 500};
    cairo_surface_t *surface = cairo_pdf_surface_create("covarianceComparisonTests.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);
    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    // White background:
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    cairo_set_line_width(cr, 0.01);

    int numTrials = 20;
    for (int i = 0; i < numTrials; i++) {
        // Set the robot's state and uncertainty:
        arma::vec2 stateA = {0.33, 0.5};
        arma::vec2 stateB = {0.66, 0.5};
        arma::mat22 normMatA = arma::randn(2, 2);
        arma::mat22 normMatB = arma::randn(2, 2);
        arma::mat22 stateCovA = normMatA.t()*normMatA * 0.01;
        arma::mat22 stateCovB = normMatB.t()*normMatB * 0.01;

        bool aLessThanB = nump::RRBT::compareCovariancesLT(stateCovA, stateCovB);

        Ellipse ellipseA = Ellipse::forConfidenceRegion(stateA, stateCovA);
        Ellipse ellipseB = Ellipse::forConfidenceRegion(stateB, stateCovB);

        cairo_set_line_width(cr, 0.02);
        if (aLessThanB) {
            shared::utility::drawing::cairoSetSourceRGB(cr, {0.0, 0.8, 0.0});
            shared::utility::drawing::drawEllipse(cr, ellipseA);
            shared::utility::drawing::cairoSetSourceRGB(cr, {0.8, 0.0, 0.0});
            shared::utility::drawing::drawEllipse(cr, ellipseB);
        } else {
            shared::utility::drawing::cairoSetSourceRGB(cr, {0.0, 0.8, 0.0});
            shared::utility::drawing::drawEllipse(cr, ellipseB);
            shared::utility::drawing::cairoSetSourceRGB(cr, {0.8, 0.0, 0.0});
            shared::utility::drawing::drawEllipse(cr, ellipseA);
        }

        // Add page to PDF:
        cairo_show_page(cr);
    }

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "covarianceComparisonTests: END" << std::endl << std::endl;
}
