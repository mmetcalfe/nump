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
using shared::utility::drawing::cairoSetSourceRGB;
using shared::utility::drawing::cairoSetSourceRGBAlpha;
using shared::utility::drawing::drawRobot;
using shared::utility::drawing::drawErrorEllipse;

using nump::math::Transform2D;
using nump::math::Circle;

void propogateTests() {
    std::cout << "PropogateTests: BEGIN" << std::endl;

    // Create surface:
    arma::ivec2 surfaceDimensions = {500, 500};
    cairo_surface_t *surface = cairo_pdf_surface_create("propogateTests.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);
    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));

    // White background:
    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);

    // Get random seed:
    int seed = randomSeed();
    std::cout << "  SEED: " << seed << std::endl;

    double size = 0.1;

    // Define colours:
    arma::vec3 colObstacle = {0.0, 0.0, 0.0};
    arma::vec3 colInitial = {0.5, 0.5, 0.5};
    arma::vec3 colProgress = {0.7, 0.6, 0.3};
    arma::vec3 colSuccess = {0.5, 0.9, 0.5};
    arma::vec3 colFailure = {0.9, 0.5, 0.5};

    // Create the obstacles:
    std::vector<nump::math::Circle> obstacles;
//    obstacles.push_back({{0.6, -0.9}, 1.0});
//    obstacles.push_back({{0.5, 0.5}, 0.2});
//    obstacles.push_back({{0.4, 1.5}, 0.7});

    // Create initial state:
    arma::vec2 x1 = {0.5, 0.5}; //nump::SearchTree::TrajT::sample();

    // Create initial belief node:
    auto n1 = std::make_shared<nump::RRBT::BeliefNode>(std::weak_ptr<nump::RRBT::GraphT::Node>()); // null weak pointer
    n1->parent = nullptr;
    n1->stateCov = arma::diagmat(arma::vec({0.001, 0.001}));
    n1->stateCov(0,1) = n1->stateCov(1,0) = 0.0;
    n1->stateDistribCov = arma::diagmat(arma::vec({0.0, 0.0}));
    n1->cost = 0;

    int numTrials = 10;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

        // White background:
        cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);

        // Draw obstacles:
        for (auto& obs : obstacles) {
            fillCircle(cr, obs, colObstacle, 0.3);
        }

        // Draw initial distribution:
        cairoSetSourceRGB(cr, colInitial);
        drawRobot(cr, x1, size);
        drawErrorEllipse(cr, x1, n1->stateCov + n1->stateDistribCov, 0.95);

        // Sample target state:
        nump::SearchTree::StateT x2 = nump::SearchTree::TrajT::sample();
//        x2(0) = 0.5 + 0.5 * x2(0);

        // Generate the trajectory:
        nump::SearchTree::TrajT traj = nump::RRBT::connect(x1, x2);

        // Perform belief propogation:
        auto n2 = nump::RRBT::propagate(traj, n1, obstacles,
                                        [cr, size, colProgress, colSuccess, traj](auto t, auto xt, auto nt) {
            double frac = t / traj.t;

            arma::vec3 col = (1-frac)*colProgress + frac*colSuccess;

            cairoSetSourceRGBAlpha(cr, col, 0.2);
            drawRobot(cr, xt, size * 0.2);
            drawErrorEllipse(cr, xt, nt->stateCov + nt->stateDistribCov, 0.95);
        });

        if (n2 == nullptr) {
            // Handle propogation failure:
            cairoSetSourceRGB(cr, colFailure);
            drawRobot(cr, x2, size);
        } else {
            // Draw resultant distribution:
            cairoSetSourceRGB(cr, colSuccess);
            drawRobot(cr, x2, size);

            drawErrorEllipse(cr, x2, n2->stateCov + n2->stateDistribCov, 0.95);
        }

        cairo_show_page(cr);
    }

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "PropogateTests: END" << std::endl << std::endl;
}
