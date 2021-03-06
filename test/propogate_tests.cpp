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
using utility::drawing::cairoSetSourceRGB;
using utility::drawing::cairoSetSourceRGBAlpha;
using utility::drawing::drawRobot;
using utility::drawing::drawErrorEllipse;

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
//    int seed = 36957015;
    arma::arma_rng::set_seed(seed);

    std::cout << "  SEED: " << seed << std::endl;

    double size = 0.1;

    // Define colours:
    arma::vec3 colObstacle = {0.0, 0.0, 0.0};
    arma::vec3 colRegion = {0.0, 0.0, 1.0};
    arma::vec3 colInitial = {0.5, 0.5, 0.5};
    arma::vec3 colProgress = {0.7, 0.6, 0.3};
    arma::vec3 colSuccess = {0.5, 0.9, 0.5};
    arma::vec3 colFailure = {0.9, 0.5, 0.5};
    double lwHighlight = 0.01;
    double lwNormal = 0.003;

    // Create the obstacles:
    std::vector<nump::math::Circle> obstacles;
    std::vector<nump::math::Circle> measurementRegions;
    obstacles.push_back({{0.6, -0.9}, 1.0});
//    obstacles.push_back({{0.5, 0.5}, 0.2});
    obstacles.push_back({{0.4, 1.5}, 0.7});

    for (auto& obs : obstacles) {
        // nump::math::Circle reg = {obs.centre, obs.radius + 0.2};
        nump::math::Circle reg = {obs.centre, obs.radius + 2};
        measurementRegions.push_back(reg);
    }

    // Create initial state:
//    arma::vec2 x1 = {0.5, 0.5}; //nump::RRBT::TrajT::sample();
    // Transform2D x1 = {0.5, 0.5, 0}; //nump::RRBT::TrajT::sample();
    nump::RRBT::StateT x1;
     x1.position = {0.5, 0.5, 0}; //nump::RRBT::TrajT::sample();
//    x1 = nump::RRBT::TrajT::sample({1, 1});

    arma::vec2 footprintSize = {0.1, 0.15};

    // Create initial belief node:
    auto n1 = std::make_shared<nump::RRBT::BeliefNode>(std::weak_ptr<nump::RRBT::GraphT::Node>()); // null weak pointer
    n1->parent = nullptr;
    // n1->stateCov = arma::diagmat(arma::vec({0.001, 0.001, 0.001}));
    n1->stateCov = arma::diagmat(arma::vec({0.0001, 0.0001, 0.0001}));
//    n1->stateCov = arma::diagmat(arma::vec({0.001, 0.001}));
    n1->stateCov(0,1) = n1->stateCov(1,0) = 0.0;
    n1->stateDistribCov = arma::diagmat(arma::vec({0.0, 0.0, 0.0}));
//    n1->stateDistribCov = arma::diagmat(arma::vec({0.0, 0.0}));
    n1->cost = 0;

    int numTrials = 20;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

        // White background:
        cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);

        // Draw obstacles:
        for (auto& reg : measurementRegions) {
            fillCircle(cr, reg, colRegion, 0.3);
        }
        for (auto& obs : obstacles) {
            fillCircle(cr, obs, colObstacle, 0.3);
        }

        x1 = nump::RRBT::TrajT::sample({1, 1});

        // Draw initial distribution:
        cairoSetSourceRGB(cr, colInitial);
        drawRobot(cr, x1.position, size);
        cairo_set_line_width(cr, lwHighlight);
        drawErrorEllipse(cr, x1.position.head(2), arma::mat(n1->stateCov + n1->stateDistribCov).submat(0,0,1,1), 0.95);
        cairo_stroke(cr);

        // Sample target state:
        nump::RRBT::StateT x2 = nump::RRBT::TrajT::sample({1, 1});
//        x2(0) = 0.5 + 0.5 * x2(0);

        // Generate the trajectory:
        nump::RRBT::TrajT traj = nump::RRBT::connect(x1, x2);

        numptest::SearchScenario::Config::RRBT rrbtConfig;
        rrbtConfig.propagateTimeStep = 0.1;
        rrbtConfig.chanceConstraint = 0.7;
        
        // Perform belief propagation:
        auto n2 = nump::RRBT::propagate(traj, n1, footprintSize, rrbtConfig, obstacles, measurementRegions, [&](auto t, auto xt, auto nt) {
            double frac = t / traj.t;

            nump::RRBT::StateCovT fullCov = nt->stateCov + nt->stateDistribCov;

            arma::vec3 colt = (1-frac)*colProgress + frac*colSuccess;
            double alpha = 0.3;
            cairo_set_line_width(cr, lwNormal);

            if (!nump::RRBT::satisfiesChanceConstraint(xt, fullCov, footprintSize, obstacles, rrbtConfig.chanceConstraint)) {
                colt = colFailure;
                alpha = 1;
                cairo_set_line_width(cr, lwHighlight);
            }

            cairoSetSourceRGBAlpha(cr, colt,alpha);
            drawRobot(cr, xt.position, size * 0.2);
            drawErrorEllipse(cr, xt.position.head(2), fullCov.submat(0,0,1,1), 0.95);
            cairo_stroke(cr);

            cairoSetSourceRGBAlpha(cr, colt * 0.5, alpha);
            drawErrorEllipse(cr, xt.position.head(2), nt->stateCov.submat(0,0,1,1), 0.95);
            cairo_stroke(cr);
        });

        cairo_set_line_width(cr, lwHighlight);

        if (n2 == nullptr) {
            // Handle propogation failure:
            cairoSetSourceRGB(cr, colFailure);
            drawRobot(cr, x2.position, size);
        } else {
            // Draw resultant distribution:
            cairoSetSourceRGB(cr, colSuccess);
            drawRobot(cr, x2.position, size);

            drawErrorEllipse(cr, x2.position.head(2), arma::mat(n2->stateCov + n2->stateDistribCov).submat(0,0,1,1), 0.95);
            cairo_stroke(cr);
        }

        cairo_show_page(cr);
    }

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;

    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "PropogateTests: END" << std::endl << std::endl;
}
