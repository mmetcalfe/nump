//
// Created by Mitchell Metcalfe on 18/08/15.
//

#include "Trajectory.h"
#include "math/geometry.h"
#include "shared/utility/math/angle.h"

namespace nump {

    Transform2D walkBetweenFar(const Transform2D& currentState, const Transform2D& targetState) {
        auto diff = arma::vec2(targetState.xy() - currentState.xy());
        auto dir = utility::math::angle::vectorToBearing(diff);
        double wcAngle = utility::math::angle::signedDifference(dir, currentState.angle());
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

        arma::vec2 translationVec = arma::normalise(localTarget.xy());
        double translationAngle = utility::math::angle::vectorToBearing(translationVec);
        double translationSpeed = (1 - std::abs(translationAngle)*(0.25/M_PI)); // TODO: Ensure translationSpeed matches the distance metrics used.
        arma::vec2 translationVelocity = translationVec * translationSpeed;
        Transform2D velocity = {translationVelocity, rotationSpeed};
        return velocity;
    }

    Transform2D walkBetween(const Transform2D& x1, const Transform2D& x2) {
        Transform2D localTarget = x1.worldToLocal(x2);
        auto targetAngle = utility::math::angle::vectorToBearing(localTarget.xy());

        double dist = arma::norm(localTarget.xy());

//        double farDist = 0.6;
//        double nearDist = 0.2;
//        double blend = std::max(0.0, std::min(1.0, (dist - nearDist) / (farDist - nearDist)));
//        return blend * walkBetweenFar(x1, x2) + (1 - blend) * walkBetweenNear(x1, x2);

//        if (dist > 0.2) {
        if (dist > 0.2 && std::abs(targetAngle) < M_PI*0.25) { // TODO: Choose walkBetweenFar as long as it satisfies the topological property.
            return walkBetweenFar(x1, x2);
        } else {
            return walkBetweenNear(x1, x2);
        }
    }

    template <>
    Trajectory<arma::vec2>::Trajectory() {
        xInit.zeros();
        xGoal.zeros();
    }

    template <>
    Trajectory<Transform2D>::Trajectory() {
        xInit.zeros();
        xGoal.zeros();
//        u.zeros();
    }

    template <>
    arma::vec2 Trajectory<arma::vec2>::operator() (double t) const {
        arma::vec2 diff = arma::normalise(xGoal - xInit);

        return xInit + diff * t;
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
    Trajectory<arma::vec2> Trajectory<arma::vec2>::fromEndpoints(arma::vec2 xInit, arma::vec2 xGoal) {
        Trajectory<arma::vec2> traj;

        traj.xInit = xInit;
        traj.xGoal = xGoal;
        traj.reachesTarget = false;
        traj.t = 0;

        arma::vec2 diff = xGoal - xInit;
        double l = arma::norm(diff);

        double maxDist = 200;
        if (l < maxDist) {
            traj.t = l;
            traj.reachesTarget = true;
        } else {
            traj.t = maxDist;
            traj.reachesTarget = false;
        }

        return traj;
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


    template <>
    arma::vec2 Trajectory<arma::vec2>::sample(arma::vec2 mapSize) {
        arma::vec rvec = arma::randu(2);
        return { rvec(0)*mapSize(0), rvec(1)*mapSize(1) };
    }

    template <>
    Transform2D Trajectory<Transform2D>::sample(arma::vec2 mapSize) {
        arma::vec rvec = arma::randu(3);
        return { rvec(0)*mapSize(0), rvec(1)*mapSize(1), (rvec(2) * 2 - 1) * M_PI };
    }
}
