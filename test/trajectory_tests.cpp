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
#include "shared/utility/math/angle.h"
#include "tests.h"

using utility::math::geometry::Ellipse;
// using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::Circle;

void drawTrajectory(cairo_t *cr, const nump::RRBT::TrajT& traj, double timeStep, double size) {
    for (double t = 0; t < traj.t; t += timeStep) {
        nump::RRBT::StateT pos = traj(t);
        utility::drawing::drawRobot(cr, pos.position, size * 0.2, true);
    }
    utility::drawing::drawRobot(cr, traj(traj.t).position, size * 0.2, true);
}

void drawSampleTrajectories(cairo_t *cr) {
    double size = 0.2;

    nump::RRBT::StateT x1 = {{0.5, 0.5, 1.5*M_PI}}; //nump::RRBT::TrajT::sample();
    //    arma::vec2 x1 = {0.5, 0.5}; //nump::RRBT::TrajT::sample();

    utility::drawing::cairoSetSourceRGB(cr, {0.5, 0.5, 0.5});
    utility::drawing::drawRobot(cr, x1.position, size);

    int numTrials = 12;
    int numSteps = 200;
    for (int i = 0; i < numTrials; i++) {
        double t = i / double(numTrials);
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

    //        Transform2D x1 = nump::RRBT::TrajT::sample();
        // nump::RRBT::StateT x2 = nump::RRBT::TrajT::sample({1, 1});
        nump::RRBT::StateT x2;
        arma::vec2 dirVec = utility::math::angle::bearingToUnitVector(2*arma::datum::pi*t);
        double targetHeading = 0;
        double targetDist = 0.4;
        switch (i % 3) {
            case 1:
                targetDist *= 1;
                targetHeading = 0;
                break;
            case 2:
                targetDist *= 0.9;
                targetHeading = arma::datum::pi*0.25;
                break;
            default:
                targetDist *= 0.8;
                targetHeading = arma::datum::pi*0.5;
                break;
        }
        x2.position = x1.position + Transform2D({targetDist*dirVec, targetHeading});
        nump::RRBT::TrajT traj = nump::RRBT::steer(x1, x2);


    //        utility::drawing::cairoSetSourceRGB(cr, col * 0.5);
    //        utility::drawing::drawRobot(cr, x1, size);
        utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.5);
        utility::drawing::drawRobot(cr, x2.position, size);

        if (!traj.reachesTarget) {
            cairo_set_source_rgb (cr, 1, 0.0, 0.0);
            utility::drawing::drawRobot(cr, x1.position, size);
            cairo_set_source_rgb (cr, 1, 0.5, 0.5);
            utility::drawing::drawRobot(cr, x2.position, size);
    //            continue;
        }

        // Path colour:
        arma::vec3 pathCol = col * 0.75;
        if (traj.reachesTarget) {
            pathCol = col * 0.75;
        } else {
            pathCol = { 0.5, 0.2, 0.2 };
        }

        utility::drawing::cairoSetSourceRGBAlpha(cr, pathCol, 0.5);
        double timeStep = size*0.25;
        drawTrajectory(cr, traj, timeStep, size);
        cairo_fill(cr);

    //        utility::drawing::cairoSetSourceRGB(cr, pathCol);
        utility::drawing::cairoSetSourceRGB(cr, {1,1,1});
        utility::drawing::drawRobot(cr, traj(traj.t).position, size * 0.2);
        utility::drawing::cairoSetSourceRGB(cr, {0,0,0});
        // utility::drawing::showText(cr, x(x.t).position.rows(0,1) + arma::vec2({0, 0.05}), size * 0.1, x.t, ", ", x.reachesTarget);
        utility::drawing::showText(cr, traj(traj.t).position.rows(0,1) + arma::vec2({0, 0.02}), size * 0.1, traj.t);
    }
}

void drawDistanceField(cairo_t *cr) {
    double size = 0.2;

    nump::RRBT::StateT x1 = {{0.5, 0.5, 1.5*arma::datum::pi}}; //nump::RRBT::TrajT::sample();
    //    arma::vec2 x1 = {0.5, 0.5}; //nump::RRBT::TrajT::sample();

    int gridSize = 50;
    double cellSize = 1 / double(gridSize - 1);
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            double x = 1 - i / double(gridSize - 1);
            double y = j / double(gridSize - 1);
            nump::RRBT::StateT x2;
            x2.position = Transform2D({x, y, 0});
            nump::RRBT::TrajT traj = nump::RRBT::steer(x1, x2);

            double c = traj.t * 0.8;
            arma::vec3 distCol = {c, c, c};

            if (!traj.reachesTarget) {
                distCol = {1, 0.5, 0.5};
            } else {
                double c = traj.t * 0.8;
                // arma::vec3 col = {c,1-c,1-c};
                // utility::drawing::cairoSetSourceRGB(cr, col);
                distCol = {c, c, c};
            }

            utility::drawing::cairoSetSourceRGB(cr, distCol);
            Transform2D resultPos = traj(traj.t).position;
            // utility::drawing::drawRobot(cr, resultPos, size * 0.2);
            // utility::drawing::drawCircle(cr, {x2.position.xy(), 1.05*arma::datum::sqrt2/(2*gridSize)});
            utility::drawing::drawRotatedRectangle(cr, {{x, y, 0}, {cellSize*1.05, cellSize*1.05}});
            cairo_fill(cr);

            utility::drawing::cairoSetSourceRGB(cr, distCol * 0.5);
            cairo_set_line_width(cr, 0.002);
            utility::drawing::drawRobot(cr, x2.position, 1.2*arma::datum::sqrt2/(2*gridSize), true);
            cairo_stroke(cr);
        }
    }

    utility::drawing::cairoSetSourceRGB(cr, {0,0,0});
    utility::drawing::drawRobot(cr, x1.position, 0.1);
}

void trajectoryTests() {
    std::cout << "TrajectoryTests: BEGIN" << std::endl;

    // Create surface:
    arma::ivec2 surfaceDimensions = {500, 500};
    cairo_surface_t *surface = cairo_pdf_surface_create("trajectoryTests.pdf", surfaceDimensions(0), surfaceDimensions(1));
    cairo_t *cr = cairo_create(surface);
    cairo_scale(cr, surfaceDimensions(0), surfaceDimensions(1));


    // Get random seed:
    int seed = randomSeed();
    std::cout << "  SEED: " << seed << std::endl;

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    drawSampleTrajectories(cr);
    cairo_show_page(cr);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    drawDistanceField(cr);

    double size = 0.2;
    nump::RRBT::StateT x1 = {{0.5, 0.5, 1.5*arma::datum::pi}}; //nump::RRBT::TrajT::sample();
    // utility::drawing::cairoSetSourceRGB(cr, {0.5, 0.5, 0.5});
    // utility::drawing::drawRobot(cr, x1.position, size);
    int gridSize = 4;
    double cellSize = 1 / double(gridSize - 1);
    double gridScale = 0.5;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double x = 1 - i / double(gridSize - 1);
            double y = j / double(gridSize - 1);
            arma::vec2 xyPos = arma::vec2({x, y}) - arma::vec2({0.5, 0.5});
            nump::RRBT::StateT x2;
            x2.position = x1.position + Transform2D({xyPos * gridScale, arma::datum::pi*0.5});
            nump::RRBT::TrajT traj = nump::RRBT::steer(x1, x2);

            arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));
            arma::vec3 pathCol = col * 0.75;
            if (traj.reachesTarget) {
                pathCol = col * 0.75;
            } else {
                pathCol = { 0.5, 0.2, 0.2 };
            }
            // utility::drawing::cairoSetSourceRGBAlpha(cr, pathCol, 0.5);
            utility::drawing::cairoSetSourceRGB(cr, pathCol);
            double timeStep = size*0.25;
            drawTrajectory(cr, traj, timeStep, size);
            cairo_fill(cr);
        }
    }
    utility::drawing::cairoSetSourceRGB(cr, {0,0,0});
    utility::drawing::drawRobot(cr, x1.position, 0.1);
    cairo_show_page(cr);


    // Clean up:
    cairo_destroy(cr);
    cairo_surface_destroy(surface);

    std::cout << "TrajectoryTests: END" << std::endl << std::endl;
}
