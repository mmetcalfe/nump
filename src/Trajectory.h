//
// Created by Mitchell Metcalfe on 18/08/15.
//

#include "math/geometry.h"
#include <list>

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

        static StateT sample(arma::vec2 mapSize);

        static Trajectory fromEndpoints(StateT xInit, StateT xGoal);

        Trajectory();
        Trajectory(StateT xInit, StateT xGoal, StateT u, double t)
                : xInit(xInit)
                , xGoal(xGoal)
//                , u(u)
                , t(t) {
        }
        StateT operator() (double t, double timeStep = 0.01) const;

        struct TrajectoryWalker {
            StateT xCurrent;
            StateT xNext;
            StateT xGoal;
            double t = 0;
            double finishTime = 0;
            double timeStep = 0;

            // Modifies the trajectory to advance its initial state by a single step of t seconds.
            void stepBy();
            arma::vec2 currentControl();
            Transform2D currentOmniControl();
            StateT currentState();
            double currentTime();
            bool isFinished();

            StateT getNext(const StateT& current, double deltaT) const;
        };

        TrajectoryWalker walk(double timeStep) const;
    };

    template <class StateT>
    struct Path {
        typedef Trajectory<StateT> TrajT;
        std::list<TrajT> segments;
        double timeSpentPlanning = 0;

        struct Walker {
            typename std::list<TrajT>::const_iterator currentSegment;
            typename std::list<TrajT>::const_iterator endSegment;
            typename TrajT::TrajectoryWalker currentWalker;
            double t = 0;
            double timeStep = 0;

            void stepTo(double t);
            void stepBy(double deltaT);
            arma::vec2 currentControl();
            Transform2D currentOmniControl();
            StateT currentState();
            bool isFinished();
        };

        Walker walk(double timeStep) const;
    };
}

#endif //NUMP_TRAJECTORY_H
