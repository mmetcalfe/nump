//
// Created by Mitchell Metcalfe on 31/08/2015.
//

#include <iostream>
#include <cairo/cairo.h>
#include <cairo/cairo-pdf.h>
#include <math.h>
#include <armadillo>
#include "nump.h"
#include "BipedRobotModel.h"
#include "shared/utility/drawing/cairo_drawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/distributions.h"
#include "shared/utility/math/angle.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "tests.h"

using utility::math::geometry::Ellipse;
using utility::drawing::drawSearchTree;
using utility::drawing::drawRRBT;
using utility::drawing::fillCircle;
using nump::math::Transform2D;
using nump::math::RotatedRectangle;
using nump::math::Circle;

std::vector<std::vector<double>> quadratureRoots = {
    {}, {}, // Make indexing easy.
    {0.5773502692, -0.5773502692},
    {0.7745966692, 0, -0.7745966692},
    {0.8611363116, 0.3399810436, -0.3399810436, -0.8611363116}
};

std::vector<std::vector<double>> quadratureCoefficients = {
    {}, {}, // Make indexing easy.
    {1, 1},
    {5.0/9.0, 8.0/9.0, 5.0/9.0},
    {0.3478548451, 0.6521451549, 0.6521451549, 0.3478548451}
};

double gaussianQuadrature(cairo_t* cr, std::function<double(arma::vec3)> func, RotatedRectangle rect, double zSpan, arma::ivec3 order) {
    // Integrates func over the rect, with a z-range of [-zSpan/2, zSpan/2].

    double sum = 0;

    for (int i = 0; i < order(0); i++) {
        double ci = quadratureCoefficients[order(0)][i];
        double u = quadratureRoots[order(0)][i];
        double lx = u*rect.size(0)*0.5;
        for (int j = 0; j < order(1); j++) {
            double cj = quadratureCoefficients[order(1)][j];
            double v = quadratureRoots[order(1)][j];
            double ly = v*rect.size(1)*0.5;
            auto worldXY = rect.transform.localToWorld({lx, ly, 0});
            double x = worldXY.x();
            double y = worldXY.y();
            for (int k = 0; k < order(2); k++) {
                double ck = quadratureCoefficients[order(2)][k];
                double w = quadratureRoots[order(2)][k];
                double z = w*zSpan*0.5;

                double f = func({x, y, z});

                sum += ci*ck*cj*f;

                utility::drawing::drawCircle(cr, {{x, y}, 0.01});
                cairo_stroke(cr);
            }
        }
    }

    double rangeFactor = (rect.size(0)/2) * (rect.size(1)/2) * (zSpan/2);

    return sum*rangeFactor;
}

void drawKickBoxes(cairo_t *cr, Transform2D robot, const numptest::SearchScenario::Config::KickBox& kbConfig, double ballRadius) {
    cairo_save(cr);
        utility::drawing::cairoTransformToLocal(cr, robot);

        double kbX = kbConfig.footFrontX+ballRadius+kbConfig.kickExtent*0.5;
        double kbY = kbConfig.footSep*0.5 + kbConfig.footWidth*0.5;
        utility::drawing::drawRotatedRectangle(cr, {{kbX, kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}});
        utility::drawing::drawRotatedRectangle(cr, {{kbX, -kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}});
    cairo_restore(cr);
}

std::vector<RotatedRectangle> getLocalKickBoxes(Transform2D robot, const numptest::SearchScenario::Config::KickBox& kbConfig, double ballRadius) {
    double kbX = kbConfig.footFrontX+ballRadius+kbConfig.kickExtent*0.5;
    double kbY = kbConfig.footSep*0.5 + kbConfig.footWidth*0.5;
    RotatedRectangle left = {{kbX, -kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}};
    RotatedRectangle right = {{kbX, kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}};
    return {left, right};
}

arma::mat33 transformToLocalDistribution(Transform2D trans, arma::mat33 transCov, Transform2D pos) {
    Transform2D diff = pos - trans;

    double sinTheta = std::sin(trans.angle());
    double cosTheta = std::cos(trans.angle());

    arma::mat33 J; // Jacobian of trans.worldToLocal(pos) with respect to trans.
    J(0,0) = -cosTheta;
    J(0,1) = -sinTheta;
    J(0,2) = -diff.x()*sinTheta + diff.y()*cosTheta;
    J(1,0) =  sinTheta;
    J(1,1) = -cosTheta;
    J(1,2) = -diff.x()*cosTheta - diff.y()*sinTheta;
    J(2,0) = 0;
    J(2,1) = 0;
    J(2,2) = -1;

    return J*transCov*J.t();
}

void kickProbabilityTests(cairo_t *cr) {
    // Transform2D robot = {0.333, 0.5, 0.5};
    // arma::mat33 stateCov = {
    //         { 0.001, 0, 0},
    //         { 0, 0.005, 0},
    //         { 0, 0, 0.05}
    // };
    // Circle ball = {{0.5, 0.666}, 0.065};


    Transform2D robot = {0.333, 0.5, 0.5};
    double vxy = 0.002;
    double vxt = 0.004;
    double vyt = 0.01;
    arma::mat33 stateCov = {
            { 0.003, vxy, vxt},
            { vxy, 0.007, vyt},
            { vxt, vyt, 0.02}
    };
    // arma::mat33 stateCov = {
    //         { 0.001, 0, 0},
    //         { 0, 0.005, 0},
    //         { 0, 0, 0.05}
    // };

    Circle ball = {{0.5, 0.666}, 0.065};
    // double targetAngle = -1;
    double targetAngle = 0;
    double targetAngleRange = arma::datum::pi/3;

    arma::vec2 footprintSize = {0.12, 0.17};
    RotatedRectangle robotFootprint = {robot, footprintSize};
    Ellipse confEllipseXY = utility::math::distributions::confidenceRegion(robot.head(2), stateCov.submat(0,0,1,1), 0.95, 3);

    double footWidth = 0.07;
    numptest::SearchScenario::Config::KickBox kbConfig = {
        0.1, // kickExtent
        footWidth, // footWidth
        footprintSize(1) - 2*footWidth, // footSep
        0.06, // footFrontX
    };

    arma::vec3 globalCol = arma::normalise(arma::vec(arma::randu(3)));
    arma::vec3 localCol = arma::normalise(arma::vec(arma::randu(3)));
    arma::vec3 quadCol = arma::normalise(arma::vec(arma::randu(3)));

    // Draw footprint and confidence ellipse:
    utility::drawing::cairoSetSourceRGB(cr, globalCol);
    cairo_set_line_width(cr, 0.01);
    drawKickBoxes(cr, robot, kbConfig, ball.radius);
    utility::drawing::drawRotatedRectangle(cr, robotFootprint);
    utility::drawing::drawEllipse(cr, confEllipseXY);
    cairo_stroke(cr);

    // Draw global ball:
    cairo_set_line_width(cr, 0.01);
    utility::drawing::drawCircle(cr, ball);

    arma::vec2 target = {std::cos(targetAngle), std::sin(targetAngle)};
    arma::vec2 minTarget = {std::cos(targetAngle-0.5*targetAngleRange), std::sin(targetAngle-0.5*targetAngleRange)};
    arma::vec2 maxTarget = {std::cos(targetAngle+0.5*targetAngleRange), std::sin(targetAngle+0.5*targetAngleRange)};
    utility::drawing::drawLine(cr, ball.centre, ball.centre+target);
    utility::drawing::drawLine(cr, ball.centre, ball.centre+minTarget);
    utility::drawing::drawLine(cr, ball.centre, ball.centre+maxTarget);
    cairo_stroke(cr);

    // Draw local ball and confidence:
    utility::drawing::cairoSetSourceRGB(cr, localCol);
    cairo_set_line_width(cr, 0.005);
    cairo_save(cr);
        utility::drawing::cairoTransformToLocal(cr, robot);

        Transform2D globalBallTarget = {ball.centre, targetAngle};
        Transform2D localBallTarget = robot.worldToLocal(globalBallTarget);
        arma::mat33 localBallTargetCov = transformToLocalDistribution(robot, stateCov, globalBallTarget);
        Ellipse localConfEllipse = utility::math::distributions::confidenceRegion(localBallTarget.xy(), localBallTargetCov.submat(0,0,1,1), 0.95, 3);

        auto kickBoxes = getLocalKickBoxes(robot, kbConfig, ball.radius);

        {
            int numSamples = 1000000;
            cairo_save(cr);
            int numLocalSamples = numSamples;
            int numHitsLeftLocal = 0;
            int numHitsRightLocal = 0;
            arma::mat localSamples = utility::math::distributions::randn(numLocalSamples, localBallTarget, localBallTargetCov);
            for (int i = 0; i < arma::size(localSamples)(1); i++) {
                utility::drawing::cairoSetSourceRGB(cr, {0, 0, 0});
                Transform2D pt = localSamples.col(i);
                if (std::abs(pt.angle()) < 0.5*targetAngleRange) {
                    if (kickBoxes[0].contains(pt.xy())) {
                        utility::drawing::cairoSetSourceRGB(cr, {1, 0, 0});
                        numHitsLeftLocal += 1;
                    } else if (kickBoxes[1].contains(pt.xy())) {
                        utility::drawing::cairoSetSourceRGB(cr, {1, 0, 0});
                        numHitsRightLocal += 1;
                    }
                }
                // utility::drawing::drawRobot(cr, pt, 0.01);
                // // cairo_fill(cr);
            }
            std::cout << "approxLeftKickBoxProbLocal: " << numHitsLeftLocal/double(numLocalSamples) << std::endl;
            std::cout << "approxRightKickBoxProbLocal: " << numHitsRightLocal/double(numLocalSamples) << std::endl;
            std::cout << "approxKickBoxProbLocal: " << (numHitsRightLocal + numHitsLeftLocal)/double(numLocalSamples) << std::endl;

            int numGlobalSamples = numSamples;
            int numHitsLeftGlobal = 0;
            int numHitsRightGlobal = 0;
            arma::mat globalSamples = utility::math::distributions::randn(numGlobalSamples, robot, stateCov);
            for (int i = 0; i < arma::size(globalSamples)(1); i++) {
                utility::drawing::cairoSetSourceRGB(cr, {0, 0, 0});
                Transform2D sampleRobot = globalSamples.col(i);
                // Transform2D localSampleRobot = robot.worldToLocal(sampleRobot);
                Transform2D pt = sampleRobot.worldToLocal(globalBallTarget); // sampleRobotLocalBallTarget
                if (std::abs(pt.angle()) < 0.5*targetAngleRange) {
                    if (kickBoxes[0].contains(pt.xy())) {
                        utility::drawing::cairoSetSourceRGB(cr, {1, 0, 0});
                        numHitsLeftGlobal += 1;
                    } else if (kickBoxes[1].contains(pt.xy())) {
                        utility::drawing::cairoSetSourceRGB(cr, {1, 0, 0});
                        numHitsRightGlobal += 1;
                    }
                }

                if (i < 10000) {
                    utility::drawing::drawRobot(cr, pt, 0.01);
                    // utility::drawing::drawRobot(cr, localSampleRobot, 0.01);
                    // // cairo_fill(cr);
                }
            }
            std::cout << "approxLeftKickBoxProbGlobal: " << numHitsLeftGlobal/double(numGlobalSamples) << std::endl;
            std::cout << "approxRightKickBoxProbGlobal: " << numHitsRightGlobal/double(numGlobalSamples) << std::endl;
            std::cout << "approxKickBoxProbGlobal: " << (numHitsRightGlobal + numHitsLeftGlobal)/double(numGlobalSamples) << std::endl;
            cairo_restore(cr);
        }

        auto densityFunc = [=](auto x){ return utility::math::distributions::dnorm(localBallTarget, localBallTargetCov, x); };
        // auto densityFunc = [=](auto x){ return 1; }; //utility::math::distributions::dnorm(localBallTarget, localBallTargetCov, x); };
        utility::drawing::cairoSetSourceRGB(cr, quadCol);
        int order = 4;
        double leftKickBoxProb = gaussianQuadrature(cr, densityFunc, kickBoxes[0], targetAngleRange, {order, order, order});
        double rightKickBoxProb = gaussianQuadrature(cr, densityFunc, kickBoxes[1], targetAngleRange, {order, order, order});
        double kickProb = leftKickBoxProb + rightKickBoxProb;
        std::cout << "leftKickBoxProb: " << leftKickBoxProb << std::endl;
        std::cout << "rightKickBoxProb: " << rightKickBoxProb << std::endl;
        std::cout << "kickProb: " << kickProb << std::endl;
        // std::cout << "targetAngleRange: " << targetAngleRange << std::endl;
        // std::cout << "kbWidth: " << kickBoxes[0].size(0) << std::endl;
        // std::cout << "kbHeight: " << kickBoxes[0].size(1) << std::endl;
        // std::cout << "volume: " << targetAngleRange * kickBoxes[0].size(0) * kickBoxes[0].size(1) << std::endl;

        utility::drawing::cairoSetSourceRGB(cr, localCol);
        utility::drawing::drawCircle(cr, {localBallTarget.xy(), ball.radius});
        utility::drawing::drawEllipse(cr, localConfEllipse);
        utility::drawing::drawRotatedRectangle(cr, kickBoxes[0]);
        utility::drawing::drawRotatedRectangle(cr, kickBoxes[1]);
        cairo_stroke(cr);

        arma::vec2 localTarget = {std::cos(localBallTarget.angle()), std::sin(localBallTarget.angle())};
        arma::vec2 localMinTarget = {std::cos(localBallTarget.angle()-0.5*targetAngleRange), std::sin(localBallTarget.angle()-0.5*targetAngleRange)};
        arma::vec2 localMaxTarget = {std::cos(localBallTarget.angle()+0.5*targetAngleRange), std::sin(localBallTarget.angle()+0.5*targetAngleRange)};
        utility::drawing::drawLine(cr, localBallTarget.xy(), localBallTarget.xy()+localTarget);
        utility::drawing::drawLine(cr, localBallTarget.xy(), localBallTarget.xy()+localMinTarget);
        utility::drawing::drawLine(cr, localBallTarget.xy(), localBallTarget.xy()+localMaxTarget);
        cairo_stroke(cr);
    cairo_restore(cr);



    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void kickBoxTests(cairo_t *cr) {

    Circle ball = {{0.5, 0.5}, 0.065};
    double targetAngle = 2;
    double targetAngleRange = arma::datum::pi/3;
    arma::vec2 footprintSize = {0.12, 0.17};

    double footWidth = 0.07;
    numptest::SearchScenario::Config::KickBox kbConfig = {
        0.1, // kickExtent
        footWidth, // footWidth
        footprintSize(1) - 2*footWidth, // footSep
        0.06, // footFrontX
    };

    int numTrials = 500;
    for (int i = 0; i < numTrials; i++) {
        cairo_set_line_width(cr, 0.002);
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

        arma::vec rvec = arma::randu(3);
        Transform2D robot = { rvec(0), rvec(1), (rvec(2) * 2 - 1) * M_PI };
        RotatedRectangle robotFootprint = {robot, footprintSize};

        if (!nump::BipedRobotModel::canKickBallAtTarget(robotFootprint, ball, kbConfig, targetAngle, targetAngleRange)) {
            utility::drawing::drawRotatedRectangle(cr, robotFootprint);
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.05);
            cairo_stroke(cr);
        } else {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.7);

            // Draw kickboxes and robot footprints:
            utility::drawing::drawRotatedRectangle(cr, robotFootprint);
            cairo_fill(cr);

            cairo_set_line_width(cr, 0.01);
            drawKickBoxes(cr, robot, kbConfig, ball.radius);
            // cairo_save(cr);
            //     cairo_set_line_width(cr, 0.01);
            //     utility::drawing::cairoTransformToLocal(cr, robot);
            //
            //     double kbX = kbConfig.footFrontX+ball.radius+kbConfig.kickExtent*0.5;
            //     double kbY = kbConfig.footSep*0.5 + kbConfig.footWidth*0.5;
            //     utility::drawing::drawRotatedRectangle(cr, {{kbX, kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}});
            //     utility::drawing::drawRotatedRectangle(cr, {{kbX, -kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}});
            // cairo_restore(cr);
            cairo_stroke(cr);

            // utility::drawing::drawCircle(cr, {robot.xy(), 0.005});
            // cairo_fill(cr);
        }
    }

    // Draw footprint and confidence ellipse:
    utility::drawing::cairoSetSourceRGB(cr, {0,0,0});
    cairo_set_line_width(cr, 0.005);
    utility::drawing::drawCircle(cr, ball);
    cairo_stroke(cr);

    arma::vec2 target = {std::cos(targetAngle), std::sin(targetAngle)};
    arma::vec2 minTarget = {std::cos(targetAngle-0.5*targetAngleRange), std::sin(targetAngle-0.5*targetAngleRange)};
    arma::vec2 maxTarget = {std::cos(targetAngle+0.5*targetAngleRange), std::sin(targetAngle+0.5*targetAngleRange)};
    utility::drawing::drawLine(cr, ball.centre, ball.centre+target);
    utility::drawing::drawLine(cr, ball.centre, ball.centre+minTarget);
    utility::drawing::drawLine(cr, ball.centre, ball.centre+maxTarget);
    cairo_stroke(cr);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void robotCircleConfidenceRegionIntersectionTests(cairo_t *cr) {

    // Transform2D robot = {0.5, 0.5, 0.5};
    // arma::vec2 footprintSize = {0.12, 0.17};
    // RotatedRectangle robotFootprint = {robot, footprintSize};

    Circle circle = {{0.5, 0.5}, 0.07};
    double vxy = -0.00125;
    arma::mat22 circleCov = {
            { 0.0025, vxy },
            { vxy, 0.001 },
    };

    arma::vec eigvals;
    arma::mat22 eigvecs;
    arma::eig_sym(eigvals, eigvecs, circleCov);
    if (eigvals(0) <= 0 || eigvals(1) <= 0) {
        std::cerr << "Matrix is not a valid covariance matrix." << std::endl << eigvals << std::endl;
        return;
    }

    int numTrials = 0;
    for (int i = 0; i < numTrials; i++) {
        cairo_set_line_width(cr, 0.002);
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));
        // arma::vec randCircle = arma::randu(3);
        // double rad = 0.01 + randCircle(2)*randCircle(2)*0.2;
        // double rad = 0.01 + randCircle(2)*0.15;
        // double rad = 0.1;
        // Circle circle = {randCircle.rows(0, 1) - arma::vec2({0.5, 0.5}), rad};

        // cairo_save(cr);
        // utility::drawing::cairoTransformToLocal(cr, robot);

        // utility::drawing::drawCircle(cr, circle);
        // Ellipse confEllipse = utility::math::distributions::confidenceRegion(circle.centre, circleCov, 0.95, 2);
        // utility::drawing::drawEllipse(cr, confEllipse);

        arma::vec rvec = arma::randu(3);
        Transform2D robot = { rvec(0), rvec(1), (rvec(2) * 2 - 1) * M_PI };

        arma::vec2 footprintSize = {0.12, 0.17};
        RotatedRectangle robotFootprint = {robot, footprintSize};
        utility::drawing::drawRotatedRectangle(cr, robotFootprint);

        bool intersects = utility::math::geometry::intersection::testConfidenceRegion(circle, circleCov, 0.95, robotFootprint);
        if (intersects) {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.05);
            cairo_stroke(cr);
        } else {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.7);
            cairo_fill(cr);
        }
        // cairo_restore(cr);
    }

    // Draw full confidence footprint of circle:
    Ellipse confEllipse = utility::math::distributions::confidenceRegion(circle.centre, circleCov, 0.95, 2);

    int numSamples = 50;
    cairo_push_group(cr);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    cairo_set_line_width(cr, 0.005);
    for (int i = 0; i < numSamples; i++) {
        double t = i / double(numSamples);
        double lx = 0.5 * confEllipse.size(0) * std::cos(2*arma::datum::pi*t);
        double ly = 0.5 * confEllipse.size(1) * std::sin(2*arma::datum::pi*t);
        Transform2D gp = confEllipse.transform.localToWorld({lx, ly, 0});

        utility::drawing::drawCircle(cr, {gp.xy(), circle.radius});
    }
    utility::drawing::drawEllipse(cr, confEllipse);
    cairo_fill(cr);
    cairo_pop_group_to_source(cr);
    cairo_paint_with_alpha(cr, 0.5);

    // Draw footprint and confidence ellipse:
    // utility::drawing::cairoSetSourceRGB(cr, {0.7,0.7,0.7});
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    cairo_set_line_width(cr, 0.0025);
    // utility::drawing::drawRotatedRectangle(cr, robotFootprint);
    utility::drawing::drawCircle(cr, circle);
    utility::drawing::drawCircle(cr, {circle.centre, 0.001});
    utility::drawing::drawEllipse(cr, confEllipse);
    utility::drawing::drawEllipse(cr, {confEllipse.transform, confEllipse.size + 2*arma::vec2({circle.radius, circle.radius})});
    cairo_stroke(cr);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void drawRobotConfidenceFootprint(cairo_t *cr, arma::vec2 footprintSize, Transform2D state, arma::mat33 stateCov) {
    arma::vec2 confIntervalX = utility::math::distributions::confidenceRegion(state(0), stateCov(0, 0), 0.95, 3);
    arma::vec2 confIntervalY = utility::math::distributions::confidenceRegion(state(1), stateCov(1, 1), 0.95, 3);

    Ellipse confEllipseXY = utility::math::distributions::confidenceRegion(state.head(2), stateCov.submat(0,0,1,1), 0.95, 3);

    // std::cout << arma::min(confEllipseXY.size)/2 << std::endl;
    // std::cout << arma::norm(footprintSize/2) << std::endl;

    arma::vec2 halfFP = footprintSize / 2;
    double ignoreRadius = arma::min(confEllipseXY.size/2) + arma::min(halfFP) - arma::norm(halfFP);

    cairo_push_group(cr);
    double ss = 0.03;
    for (double sx = confIntervalX(0); sx < confIntervalX(1); sx += ss) {
        for (double sy = confIntervalY(0); sy < confIntervalY(1); sy += ss) {

            arma::vec2 sp = {sx, sy};
            if (arma::norm(state.head(2) - sp) < ignoreRadius) {
                continue;
            }

            auto tRange = utility::math::distributions::confidenceEllipsoidZRangeForXY({sx, sy}, state, stateCov, 0.95);

            if (tRange.size() != 2) {
                // if (tRange.size() == 0) {
                //     utility::drawing::cairoSetSourceRGB(cr, {1.0,0.0,0.0});
                //     utility::drawing::drawCircle(cr, {{sx, sy}, ss*0.5});
                //     cairo_fill(cr);
                //     utility::math::distributions::confidenceEllipsoidZRangeForXY({sx, sy}, state, stateCov, 0.95);
                // }
                continue;
            } else if (tRange.size() == 2) {
                // arma::vec2 confIntervalT = utility::math::distributions::confidenceRegion(state(2), stateCov(2, 2), 0.95, 3);
                // double theight = confIntervalT(1) - confIntervalT(0);
                // double span = tRange[1] - tRange[0];
                // utility::drawing::cairoSetSourceRGB(cr, {span / theight,0.0,0.5});
                // utility::drawing::drawCircle(cr, {{sx, sy}, ss*0.5});
                // cairo_fill(cr);
            }

            // for (double st = tRange[0]; st < tRange[1]; st += ss) {
            //     utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
            //     utility::drawing::drawRotatedRectangle(cr, {{sx, sy, st}, footprintSize});
            //     cairo_fill(cr);
            // }

            // utility::drawing::cairoSetSourceRGB(cr, {1.0,0.0,0.0});
            utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
            utility::drawing::drawRectangleRotationRange(cr, {sx, sy}, footprintSize, {tRange[0], tRange[1]});
            cairo_stroke(cr);
        }
    }
    cairo_pop_group_to_source(cr);
    cairo_paint_with_alpha(cr, 0.5);

    // Draw ellipse AABB:
    // utility::drawing::cairoSetSourceRGB(cr, {0.8,0.8,0.8});
    // utility::drawing::drawRotatedRectangle(cr, {{state.xy(), 0}, {confIntervalX(1) - confIntervalX(0), confIntervalY(1) - confIntervalY(0)}});
    // cairo_stroke(cr);
}

void circleRobotConfidenceRegionIntersectionTests(cairo_t *cr) {

    Transform2D state = {0.5, 0.5, 0.5};
    double vxy = 0.002;
    double vxt = 0.004;
    double vyt = 0.01;
    arma::mat33 stateCov = {
            { 0.003, vxy, vxt},
            { vxy, 0.007, vyt},
            { vxt, vyt, 0.02}
    };

    arma::vec eigvals;
    arma::mat33 eigvecs;
    arma::eig_sym(eigvals, eigvecs, stateCov);
    if (eigvals(0) <= 0 || eigvals(1) <= 0 || eigvals(2) <= 0) {
        std::cerr << "Matrix is not a valid covariance matrix." << std::endl << eigvals << std::endl;
        return;
    }

    // arma::vec2 footprintSize = {0.17, 0.12};
    arma::vec2 footprintSize = {0.12, 0.17};


    RotatedRectangle robotFootprint = {state, footprintSize};
    Ellipse confEllipseXY = utility::math::distributions::confidenceRegion(state.head(2), stateCov.submat(0,0,1,1), 0.95, 3);

    int numTrials = 0;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));
        arma::vec randCircle = arma::randu(3);
        // double rad = 0.01 + randCircle(2)*randCircle(2)*0.2;
        double rad = 0.01 + randCircle(2)*0.15;
        // double rad = 0.01;
        Circle circle = {randCircle.rows(0, 1), rad};

        cairo_set_line_width(cr, 0.002);
        utility::drawing::drawCircle(cr, circle);

        bool intersects = utility::math::geometry::intersection::testConfidenceRegion(robotFootprint, stateCov, 0.95, circle);
        if (intersects) {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.1);
            cairo_stroke(cr);
        } else {
            utility::drawing::cairoSetSourceRGBAlpha(cr, col, 0.7);
            cairo_fill(cr);
        }
    }

    // Draw full confidence interval footprint:
    drawRobotConfidenceFootprint(cr, footprintSize, state, stateCov);

    // Draw footprint and confidence ellipse:
    // utility::drawing::cairoSetSourceRGB(cr, {0.8,0.8,0.8});
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    cairo_set_line_width(cr, 0.005);
    utility::drawing::drawRotatedRectangle(cr, robotFootprint);
    cairo_stroke(cr);
    utility::drawing::drawEllipse(cr, confEllipseXY);
    utility::drawing::drawCircle(cr, {state.xy(), 0.001});
    cairo_stroke(cr);
    // utility::drawing::drawRotatedRectangle(cr, confEllipseXY);
    // cairo_stroke(cr);
    // utility::drawing::drawRotatedRectangle(cr, {confEllipseXY.transform, confEllipseXY.size/arma::datum::sqrt2});
    // cairo_stroke(cr);

    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void circleEllipseIntersectionTests(cairo_t *cr) {
    // Set the robot's state and uncertainty:
    arma::vec2 state = {0.5, 0.5};
    arma::mat22 stateCov = {
            { 0.025,  0.01},
            { 0.01,  0.01}
    };
    Ellipse confEllipse = utility::math::distributions::confidenceRegion(state, stateCov * 0.5, 0.95);

    int numTrials = 1000;
    for (int i = 0; i < numTrials; i++) {
        arma::vec3 col = arma::normalise(arma::vec(arma::randu(3)));

        arma::vec randCircle = arma::randu(3);

       Circle circle = {randCircle.rows(0, 1), 0.05};
        // Circle circle = {randCircle.rows(0, 1), 0.01 + randCircle(2)*randCircle(2)*0.2};

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

    // std::cout << "trans " << confEllipse.getTransform() << std::endl;
    // std::cout << "size " << confEllipse.getSize() << std::endl;


    // Draw confidence region of the robot:
    cairo_set_line_width(cr, 0.005);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    utility::drawing::drawEllipse(cr, confEllipse);

    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    utility::drawing::drawRotatedRectangle(cr, confEllipse);
    cairo_stroke(cr);
    std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;
}

void confidenceEllipseTests(cairo_t *cr) {
    {
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
        cairo_stroke(cr);
        utility::drawing::drawEllipse(cr, confEllipseL);

        arma::vec2 stateR = {0.666, 0.5};
        arma::mat22 stateCovR = {
                { 0.015, 0.000},
                { 0.000, 0.03},
        };
        Ellipse confEllipseR = utility::math::distributions::confidenceRegion(stateR.head(2), stateCovR.submat(0,0,1,1), 0.95, 2);
        utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,1.0});
        utility::drawing::drawRotatedRectangle(cr, confEllipseR);
        cairo_stroke(cr);
        utility::drawing::drawEllipse(cr, confEllipseR);
    }

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
    cairo_stroke(cr);

    Ellipse confEllipseXY = utility::math::distributions::confidenceRegion(state.head(2), stateCov.submat(0,0,1,1), 0.95, 3);
    utility::drawing::cairoSetSourceRGB(cr, {0.0,0.0,0.0});
    utility::drawing::drawRotatedRectangle(cr, confEllipseXY);
    cairo_stroke(cr);
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

            for (double sy = smin; sy < smax; sy += step) {

                auto tRange = utility::math::distributions::confidenceEllipsoidZRangeForXY({sx, sy}, state, stateCov, 0.95);

                if (tRange.size() == 2) {
                    utility::drawing::cairoSetSourceRGB(cr, {1.0,0.0,1.0});
                    utility::drawing::drawLine(cr, {sx, tRange[0]}, {sx, tRange[1]});
                    cairo_stroke(cr);
                }

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
            }
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
    kickProbabilityTests(cr);
    cairo_show_page(cr);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    kickBoxTests(cr);
    cairo_show_page(cr);

    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); cairo_paint(cr);
    robotCircleConfidenceRegionIntersectionTests(cr);
    cairo_show_page(cr);

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
