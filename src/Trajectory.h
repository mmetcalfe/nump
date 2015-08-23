//
// Created by Mitchell Metcalfe on 18/08/15.
//

#include "math/geometry.h"

#ifndef NUMP_TRAJECTORY_H
#define NUMP_TRAJECTORY_H

namespace nump {

    using nump::math::Transform2D;

//    template <class StateT>
//    StateT applyControl(StateT x, StateT u, double t);
//
//    template <>
//    arma::vec2 applyControl<arma::vec2>(arma::vec2 x, arma::vec2 u, double t);


    // A trajectory, as a function of t \in [0, tNew]
    // x: initial state
    // u: control to approach state
    // t: time for which to follow control
    template <class StateT>
    struct Trajectory {
        StateT xInit;
        StateT xGoal;
//        StateT u; // control
        double t = 0; // time
        bool reachesTarget = true;

        static StateT sample();

        static Trajectory fromEndpoints(StateT xInit, StateT xGoal);

        // TODO: Use armadillo's linear interpolation feature for more complex controls?
        Trajectory();
        Trajectory(StateT xInit, StateT xGoal, StateT u, double t)
                : xInit(xInit)
                , xGoal(xGoal)
//                , u(u)
                , t(t) {
        }
        StateT operator() (double t) const;
    };
}

#endif //NUMP_TRAJECTORY_H
