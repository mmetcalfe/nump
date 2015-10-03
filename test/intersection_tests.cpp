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

using utility::math::geometry::Ellipse;
using utility::drawing::drawSearchTree;
using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::RotatedRectangle;
using nump::math::Circle;

void circleEllipseIntersectionTests(cairo_t *cr) {
    // Set the robot's state and uncertainty:
    arma::vec2 state = {0.5, 0.5};
    arma::mat22 stateCov = {
            { 0.025,  0.01},
            { 0.01,  0.01}
    };
    Ellipse confEllipse = utility::math::distributions::confidenceRegion(state, stateCov * 0.5, 0.95);

    int numTrials = 2000;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

        arma::vec randCircle = arma::randu(3);

//        Circle circle = {randCircle.rows(0, 1), 0.01};
        Circle circle = {randCircle.rows(0, 1), 0.01 + randCircle(2)*randCircle(2)*0.2};

        cairo_set_line_width(cr, 0.002);
        utility::drawing::drawCircle(cr, circle);

        bool intersects = utility::math::geometry::intersection::test(circle, confEllipse);
        if (intersects) {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.1);
            cairo_stroke(cr);
        } else {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.7);
            cairo_fill(cr);
        }
    }

    std::cout << "trans " << confEllipse.getTransform() << std::endl;
    std::cout << "size " << confEllipse.getSize() << std::endl;


    // Draw confidence region of the robot:
    cairo_set_line_width(cr, 0.005);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    utility::drawing::drawEllipse(cr, confEllipse);

    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    utility::drawing::drawRotatedRectangle(cr, confEllipse);
    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void confidenceEllipseTests(cairo_t *cr) {
    cairo_set_line_width(cr, 0.005);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});

    arma::vec2 stateL = {0.333, 0.5};
    arma::mat22 stateCovL = {
            { 0.030, 0.000},
            { 0.000, 0.015},
    };
    Ellipse confEllipseL = utility::math::distributions::confidenceRegion(stateL.head(2), stateCovL.submat(0,0,1,1), 0.95, 2);
    utility::drawing::cairoSetSourceRGB(cr, {1.0,0.0,0.0});
    utility::drawing::drawRotatedRectangle(cr, confEllipseL);
    utility::drawing::drawEllipse(cr, confEllipseL);

    arma::vec2 stateR = {0.666, 0.5};
    arma::mat22 stateCovR = {
            { 0.015, 0.000},
            { 0.000, 0.03},
    };
    Ellipse confEllipseR = utility::math::distributions::confidenceRegion(stateR.head(2), stateCovR.submat(0,0,1,1), 0.95, 2);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,1.0});
    utility::drawing::drawRotatedRectangle(cr, confEllipseR);
    utility::drawing::drawEllipse(cr, confEllipseR);


    // arma::vec2 confInterval = utility::math::distributions::confidenceRegion(state(0), stateCov(0, 0), 0.95, 3);
    // utility::drawing::cairoSetSourceRGB(cr, {1.0,0.0,0.0});
    // utility::drawing::drawLine(cr, {confInterval(0), state(1)}, {confInterval(1), state(1)});
    // utility::drawing::drawLine(cr, {confInterval(0), 0}, {confInterval(0), 1});
    // utility::drawing::drawLine(cr, {confInterval(1), 0}, {confInterval(1), 1});
    // cairo_stroke(cr);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void circleRobotConfidenceRegionIntersectionTests(cairo_t *cr) {
    Transform2D state = {0.5, 0.5, 0.5};
    arma::mat33 stateCov = {
            { 0.015, 0.01, 0.005},
            { 0.01, 0.03, 0.01},
            { 0.005, 0.01, 0.02}
    };


    arma::vec2 footprintSize = {0.12, 0.17};

    cairo_set_line_width(cr, 0.005);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});

    RotatedRectangle robotFootprint = {state, footprintSize};
    utility::drawing::drawRotatedRectangle(cr, robotFootprint);

    Ellipse confEllipseXY = utility::math::distributions::confidenceRegion(state.head(2), stateCov.submat(0,0,1,1), 0.95, 3);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    utility::drawing::drawRotatedRectangle(cr, confEllipseXY);
    utility::drawing::drawEllipse(cr, confEllipseXY);

    // Draw a point-based representation of the true projection of the
    // confidence ellipsoid onto the XY plane.
    {
        arma::vec3 eigval;  // eigenvalues are stored in ascending order.
        arma::mat33 eigvec;
        arma::eig_sym(eigval, eigvec, stateCov);

        // std::cout << __FILE__ << ", " << __LINE__ << ": eigvals: " << eigval.t() << std::endl;
        // std::cout << __FILE__ << ", " << __LINE__ << ": eigvec: " << eigvec.t() << std::endl;

        // arma::mat33 ellipsoidRot = eigvec.t(); // Transforms to local rotation.
        // double step = 0.02;
        // double smin = -1;
        // double smax = 1;
        // for (double sx = smin; sx < smax; sx += step) {
        //     for (double sy = smin; sy < smax; sy += step) {
        //         for (double st = smin; st < smax; st += step) {
        //             // double chiSquareVal = 5.991;
        //             double chiSquareVal = 7.815;
        //             arma::vec3 sp = {sx, sy, st};
        //             arma::vec3 spDiff = sp - state;
        //             arma::vec3 spLocal = ellipsoidRot * spDiff;
        //             arma::vec3 halfAxisLengths = arma::sqrt(chiSquareVal*eigval);
        //             double rx = spLocal(0) / halfAxisLengths(0);
        //             double ry = spLocal(1) / halfAxisLengths(1);
        //             double rt = spLocal(2) / halfAxisLengths(2);
        //             if (rx*rx + ry*ry + rt*rt <= 1) {
        //                 // utility::drawing::cairoSetSourceRGBAlpha(cr, {0.0,1.0,1.0}, 0.1);
        //                 utility::drawing::cairoSetSourceRGB(cr, {0.0,1.0,0.0});
        //                 utility::drawing::drawCircle(cr, {{sx, sy}, 0.001});
        //                 cairo_stroke(cr);
        //             }
        //         }
        //     }
        // }

        arma::mat33 ellipsoidRot = eigvec.t(); // Transforms to local rotation.
        double step = 0.02;
        double smin = -1;
        double smax = 1;
        for (double sx = smin; sx < smax; sx += step) {

            auto yRange = confEllipseXY.yRangeForX(sx);

            if (yRange.size() == 2) {
                utility::drawing::cairoSetSourceRGB(cr, {0.0,1.0,0.0});
                utility::drawing::drawLine(cr, {sx, yRange[0]}, {sx, yRange[1]});
                cairo_stroke(cr);
            }
            // for (double sy = smin; sy < smax; sy += step) {
            //         // double chiSquareVal = 5.991;
            //         double chiSquareVal = 7.815;
            //         arma::vec3 sp = {sx, sy, sx}; // Note: Dummy t value.
            //         arma::vec3 spDiff = sp - state;
            //         arma::vec3 spLocal = ellipsoidRot * spDiff;
            //         arma::vec3 halfAxisLengths = arma::sqrt(chiSquareVal*eigval);
            //         double rx = spLocal(0) / halfAxisLengths(0);
            //         double ry = spLocal(1) / halfAxisLengths(1);
            //         // double rt = spLocal(2) / halfAxisLengths(2);
            //
            //         double s = rx*rx + ry*ry;
            //         if (1 - s < 0) {
            //             continue;
            //         }
            //
            //         double rtMax = std::sqrt(1 - s);
            //         double rtMin = -rtMax;
            //
            //         arma::vec3 rMin = {rx, ry, rtMin};
            //         arma::vec3 rMax = {rx, ry, rtMax};
            //
            //         // if (rx*rx + ry*ry + rt*rt <= 1) {
            //         //     // utility::drawing::cairoSetSourceRGBAlpha(cr, {0.0,1.0,1.0}, 0.1);
            //         //     utility::drawing::cairoSetSourceRGB(cr, {0.0,1.0,0.0});
            //         //     utility::drawing::drawCircle(cr, {{sx, sy}, 0.001});
            //         //     cairo_stroke(cr);
            //         // }
            // }
        }

        // arma::vec2 eigval;  // eigenvalues are stored in ascending order.
        // arma::mat22 eigvec;
        // arma::eig_sym(eigval, eigvec, stateCov.submat(0,0,1,1));
        //
        // std::cout << __FILE__ << ", " << __LINE__ << ": eigvals: " << eigval.t() << std::endl;
        // std::cout << __FILE__ << ", " << __LINE__ << ": eigvec: " << eigvec.t() << std::endl;
        //
        // arma::mat22 ellipsoidRot = eigvec.t(); // Transforms to local rotation.
        // double step = 0.005;
        // for (double sx = 0; sx < 1; sx += step) {
        //     for (double sy = 0; sy < 1; sy += step) {
        //         double chiSquareVal = 5.991;
        //         arma::vec2 sp = {sx, sy};
        //         arma::vec2 spDiff = sp - state.xy();
        //         arma::vec2 spLocal = ellipsoidRot * spDiff;
        //         arma::vec2 halfAxisLengths = arma::sqrt(chiSquareVal*eigval);
        //         double rx = spLocal(0) / halfAxisLengths(0);
        //         double ry = spLocal(1) / halfAxisLengths(1);
        //         if (rx*rx + ry*ry <= 1) {
        //             utility::drawing::cairoSetSourceRGB(cr, {0.0,1.0,1.0});
        //             utility::drawing::drawCircle(cr, {{sx, sy}, 0.001});
        //             cairo_stroke(cr);
        //         }
        //     }
        // }

        // arma::vec2 primaryAxis = arma::vec(eigvec.col(1));
        // double angle = std::atan2(primaryAxis(1), primaryAxis(0));
        //
        // double chiSquareVal = cChiSquare(conf, dof);
        // // double chiSquareVal = 5.991; // for 95% confidence interval
        // arma::vec axisLengths = 2*arma::sqrt(chiSquareVal*eigval);
        // arma::vec2 size = {axisLengths(1), axisLengths(0)};
        //
        // Transform2D trans = {mean, angle};
        // return {trans, size};
    }

    arma::mat22 stateCovXT = {
            { stateCov(0,0), stateCov(0,2)},
            { stateCov(2,0), stateCov(2,2)}
    };
    Ellipse confEllipseXT = utility::math::distributions::confidenceRegion({state(0), state(2)}, stateCovXT, 0.95, 3);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,1.0});
    utility::drawing::drawEllipse(cr, confEllipseXT);

    arma::vec2 confInterval = utility::math::distributions::confidenceRegion(state(0), stateCov(0, 0), 0.95, 3);
    utility::drawing::cairoSetSourceRGB(cr, {1.0,0.0,0.0});
    utility::drawing::drawLine(cr, {confInterval(0), state(1)}, {confInterval(1), state(1)});
    // utility::drawing::drawLine(cr, {state(0), confInterval(0)}, {state(0), confInterval(1)});

    utility::drawing::drawLine(cr, {confInterval(0), 0}, {confInterval(0), 1});

    utility::drawing::drawLine(cr, {confInterval(1), 0}, {confInterval(1), 1});

    cairo_stroke(cr);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

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

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    circleRobotConfidenceRegionIntersectionTests(cr);
    cairo_show_page(cr);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    confidenceEllipseTests(cr);
    cairo_show_page(cr);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    circleEllipseIntersectionTests(cr);
    cairo_show_page(cr);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "IntersectionTests: END" << std::endl << std::endl;
}
