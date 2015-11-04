//
// Created by Mitchell Metcalfe on 23/09/2015.
//

#ifndef NUMP_SEARCHSCENARIO_H
#define NUMP_SEARCHSCENARIO_H

#include <armadillo>
#include <cairo/cairo.h>
#include "math/geometry.h"
#include "BipedRobotModel.h"
#include "Trajectory.h"

namespace numptest {

    using nump::math::Transform2D;
    using nump::math::Circle;

//    template<class StateT, class StateCovT>
    class SearchScenario {
        // typedef arma::vec2 StateT;
        // typedef arma::mat22 StateCovT;

        typedef Transform2D StateT;
        typedef arma::mat33 StateCovT;

    public:
        struct SearchTrialResult {
            bool kickFailure = false;
            bool collisionFailure = false;
            Transform2D finalState = {0,0,0};
            double finishTime = 0;
            Transform2D initialState = {0,0,0};
            bool kickSuccess = false;
            double targetAngleRange = arma::datum::pi/3;
            double timeLimit = 1;
            double replanInterval = 3;
            double searchTimeLimit = 3;
            double chanceConstraint = 0.75;
            double ballObstacleRadiusFactor = 0.75;
            double ballObstacleOffsetFactor = 0.75;
            int numReplans = 0;
            int seed = 0;
        };

        enum class StateType {
            Position,
            PositionBearing,
        };

        struct Config {
            StateType stateType;

            // Search settings:
            int seed;
            int numSamples;
            double searchTimeLimitSeconds;
            double searchTrialDuration;
            double replanInterval;

            struct RRBT {
                double appendRejectCovThreshold = 10;
                double appendRejectCostThreshold = 0.01;
                double propagateTimeStep = 0.1;

                double chanceConstraint = 0.7;
                double minKickProbability = 0.5; // The minimum acceptable probability of a successful kick in the resulting goal state.
            } rrbt;

            double ballObstacleRadiusFactor = 0.13;
            double ballObstacleOffsetFactor = 0.13;

            // Drawing:
            int drawPeriod;
            arma::ivec2 canvasSize;

            // Scenario description:
            arma::vec2 mapSize;
            arma::vec2 footprintSize;
            StateT initialState;
            StateCovT initialCovariance;
            StateT goalState;

            // Ball approach search config:
            Circle ball = {{0, 0}, 1};
            double targetAngle; // The direction in which to kick the ball.
            double targetAngleRange; // The range of acceptable kick angles around the target angle.

            // struct KickBox {
            //     double kickExtent; // The distance the front of the foot travels during a kick.
            //     double footWidth; // The width of the robot's foot.
            //     double footSep; // The distance between the robots feet.
            //     double footFrontX; // The x coordinate of the front of the robot's feet before beginning a kick.
            // } kbConfig;

            nump::BipedRobotModel::KickBox kbConfig;

            std::vector<Circle> obstacles;
            std::vector<Circle> measurementRegions;
        } cfg_;

        void performRRBTSearch(cairo_t* cr, const std::string& scenario_prefix);
        void performRRTsSearch(cairo_t* cr, const std::string& scenario_prefix);
        void execute(const std::string& scenario_prefix);

        // void simulate(cairo_t* cr, nump::Path<nump::BipedRobotModel::State> nominalPath);
        numptest::SearchScenario::SearchTrialResult simulate(cairo_t* cr,
            std::function<nump::Path<nump::BipedRobotModel::State>(nump::BipedRobotModel::State, nump::BipedRobotModel::MotionCov)> replanFunc
        );
        void simulation(cairo_t* cr);


        static SearchScenario fromFile(const std::string& file_path);
    };

}


#endif //NUMP_SEARCHSCENARIO_H
