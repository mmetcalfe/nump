//
// Created by Mitchell Metcalfe on 18/08/15.
//

#include "Trajectory.h"
#include "math/geometry.h"
#include "shared/utility/math/angle.h"

namespace nump {

//    template <>
//    Trajectory<arma::vec2>::Trajectory() {
//        x.zeros();
//        u.zeros();
//    }
//
//    template <>
//    arma::vec2 Trajectory<arma::vec2>::operator() (double t) {
//        return x + u * t;
//    }


    Transform2D walkBetweenFar(const Transform2D& currentState, const Transform2D& targetState) {
        auto diff = arma::vec2(targetState.xy() - currentState.xy());
        auto dir = utility::math::angle::vectorToBearing(diff);
        double wcAngle = utility::math::angle::signedDifference(dir, currentState.angle());
        // TODO: Consider the heading of targetState in planning.
        int angleSign = (wcAngle < 0) ? -1 : 1;

        double walk_far_rotation_speed = 5;
        double rotationSpeed = angleSign * walk_far_rotation_speed;


        return {std::max(0.0, std::cos(wcAngle)), 0, rotationSpeed};
//        return {1, 0, rotationSpeed};
    }

    Transform2D walkBetweenNear(const Transform2D& currentState, const Transform2D& targetState) {
        Transform2D localTarget = currentState.worldToLocal(targetState);

        int angleSign = (localTarget.angle() < 0) ? -1 : 1; // angle must be normalised.
//        // TODO: Consider using a smaller, non-constant speed.
        double walk_near_rotation_speed = 5;
        double rotationSpeed = angleSign * walk_near_rotation_speed;

        arma::vec2 dir = arma::normalise(localTarget.xy());
        arma::vec2 translationVelocity = dir;
        Transform2D velocity = {translationVelocity, rotationSpeed};
        return velocity;
    }

    Transform2D walkBetween(const Transform2D& x1, const Transform2D& x2) {
        double dist = arma::norm(x1.xy() - x2.xy());

        double farDist = 0.6;
        double nearDist = 0.2;
        double blend = std::max(0.0, std::min(1.0, (dist - nearDist) / (farDist - nearDist)));
        return blend * walkBetweenFar(x1, x2) + (1 - blend) * walkBetweenNear(x1, x2);

//        if (dist > 0.2) {
//            return walkBetweenFar(x1, x2);
//        } else {
//            return walkBetweenNear(x1, x2);
//        }
    }

    template <>
    Trajectory<Transform2D>::Trajectory() {
        xInit.zeros();
        xGoal.zeros();
//        u.zeros();
    }

    template <>
    Transform2D Trajectory<Transform2D>::operator() (double t) const {
        double timeStep = 0.01;
        Transform2D pos = xInit;

        for (double currTime = 0; currTime <= t; currTime += timeStep) {
            pos = pos.localToWorld(walkBetween(pos, xGoal) * timeStep);
        }

        return pos;
    }

    template <>
    Trajectory<Transform2D> Trajectory<Transform2D>::fromEndpoints(Transform2D xInit, Transform2D xGoal) {
        Trajectory<Transform2D> traj;

        traj.xInit = xInit;
        traj.xGoal = xGoal;
        traj.reachesTarget = false;
        traj.t = 0;

        int maxSteps = 1000;
        double maxTimeStep = 0.02;
        double minTimeStep = maxTimeStep * 0.05;
        double goalThreshold = 0.005;
        double closeThreshold = goalThreshold * 100;

        Transform2D pos = xInit;

        double totalTime = 0;
        for (int numSteps = 0; numSteps < maxSteps; numSteps++) {
            double timeStep = (arma::norm(pos - xGoal) < closeThreshold) ? minTimeStep : maxTimeStep;

            pos = pos.localToWorld(walkBetween(pos, xGoal) * timeStep);

            totalTime += timeStep;
            traj.t = totalTime; // TODO: Consider removing.

            double goalDist = arma::norm(pos - xGoal);
            if (goalDist < goalThreshold) {
                traj.t = totalTime;
                traj.reachesTarget = true;
                break;
            }
        }

        return traj;
    }


}


//
//template <>
//Trajectory<arma::vec2>::Trajectory() {
//    x.zeros();
//    u.zeros();
//}
//template <>
//Trajectory<Transform2D>::Trajectory() {
//    x.zeros();
//    u.zeros();
//}
//
//template <>
//arma::vec2 Trajectory<arma::vec2>::operator() (double t) {
//    // TODO: Apply the control in local coordinates of x.
//    // TODO: Apply the control continuously over the timespan, instead of just adding it.
//    return x + u * t;
//}