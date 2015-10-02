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
#include "shared/utility/math/distributions.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "tests.h"
#include "RRBT.h"

using utility::math::geometry::Ellipse;
using utility::drawing::drawSearchTree;
using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
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
        nump::RRBT::StateT stateA, stateB;
        stateA.head(2) = arma::vec({0.33, 0.5});
        stateB.head(2) = arma::vec({0.66, 0.5});

        int matsize = 3;
        nump::RRBT::StateCovT normMatA = arma::randn(matsize, matsize);
        nump::RRBT::StateCovT normMatB = arma::randn(matsize, matsize);
        nump::RRBT::StateCovT stateCovA = normMatA.t()*normMatA * 0.01;
        nump::RRBT::StateCovT stateCovB = normMatB.t()*normMatB * 0.01;

        bool aLessThanB = nump::RRBT::compareCovariancesLT(stateCovA, stateCovB);

        Ellipse ellipseA = utility::math::distributions::confidenceRegion(stateA.head(2), stateCovA.submat(0,0,1,1), 0.95);
        Ellipse ellipseB = utility::math::distributions::confidenceRegion(stateB.head(2), stateCovB.submat(0,0,1,1), 0.95);

        cairo_set_line_width(cr, 0.02);
        if (aLessThanB) {
            utility::drawing::cairoSetSourceRGB(cr, {0.0, 0.8, 0.0});
            utility::drawing::drawEllipse(cr, ellipseA);
            utility::drawing::cairoSetSourceRGB(cr, {0.8, 0.0, 0.0});
            utility::drawing::drawEllipse(cr, ellipseB);
        } else {
            utility::drawing::cairoSetSourceRGB(cr, {0.0, 0.8, 0.0});
            utility::drawing::drawEllipse(cr, ellipseB);
            utility::drawing::cairoSetSourceRGB(cr, {0.8, 0.0, 0.0});
            utility::drawing::drawEllipse(cr, ellipseA);
        }

        // Add page to PDF:
        cairo_show_page(cr);
    }

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
    std::cout << "covarianceComparisonTests: END" << std::endl << std::endl;
}
