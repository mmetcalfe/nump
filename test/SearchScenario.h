//
// Created by Mitchell Metcalfe on 23/09/2015.
//

#ifndef NUMP_SEARCHSCENARIO_H
#define NUMP_SEARCHSCENARIO_H

#include <armadillo>
#include "math/geometry.h"

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
        enum class StateType {
            Position,
            PositionBearing,
        };

        struct Config {
            StateType stateType;

            // Search settings:
            int seed;
            int numSamples;

            double rrbtAppendRejectCovThreshold = 10;
            double rrbtAppendRejectCostThreshold = 0.01;

            // Drawing:
            int drawPeriod;
            arma::ivec2 canvasSize;

            // Scenario description:
            arma::vec2 mapSize;
            arma::vec2 footprintSize;
            StateT initialState;
            StateCovT initialCovariance;
            StateT goalState;

            std::vector<Circle> obstacles;
            std::vector<Circle> measurementRegions;
        } cfg_;

        void execute(const std::string& scenario_prefix);

        static SearchScenario fromFile(const std::string& file_path);
    };

}


#endif //NUMP_SEARCHSCENARIO_H