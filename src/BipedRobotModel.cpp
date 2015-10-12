//
// Created by Mitchell Metcalfe on 12/10/15.
//

#include "BipedRobotModel.h"

#include <vector>
#include <armadillo>
#include "math/geometry.h"
#include "shared/utility/math/angle.h"
#include "Trajectory.h"


namespace nump {
    typedef BipedRobotModel::State StateT;

    /// The A matrix in RRBT's propagate function.
    BipedRobotModel::MotionMatrix BipedRobotModel::driftMatrix(double Δt, const StateT& state) {
        BipedRobotModel::MotionMatrix At;
        At.eye();
        return At;
    }

    /// The B matrix in RRBT's propagate function.
    BipedRobotModel::MotionMatrix BipedRobotModel::controlMatrix(double Δt, const StateT& state) {
        BipedRobotModel::MotionMatrix Bt;
        Bt.eye();
        return Bt;
    }

    /// The K matrix in RRBT's propagate function.
    BipedRobotModel::MotionMatrix BipedRobotModel::regulatorMatrix(double Δt) {
        BipedRobotModel::MotionMatrix Kt;
        Kt.eye();
        Kt(0,0) = 0.5;
        Kt(1,1) = 0.5;
        return Kt;
    }

    /// The C matrix in RRBT's propagate function.
    BipedRobotModel::MeasurementMatrix BipedRobotModel::measurementMatrix(double Δt, const StateT& state) {
        BipedRobotModel::MeasurementMatrix Kt;
        Kt.eye();
        return Kt;
    }

    /// The Q matrix in RRBT's propagate function.
    BipedRobotModel::MotionCov BipedRobotModel::motionNoiseCovariance(double Δt, const StateT& state) {
        BipedRobotModel::MotionCov Qt;
        Qt.eye();
        Qt(0,0) = 0.00002;
        Qt(1,1) = 0.00002;
        return Qt;
    }

    bool anyContain(const std::vector<nump::math::Circle>& regions, const StateT& pos) {
        for (auto &reg : regions) {
            if (reg.contains(pos.position.head(2))) {
                return true;
            }
        }

        return false;
    }

    /// The R matrix in RRBT's propagate function.
    BipedRobotModel::MeasurementCov BipedRobotModel::measurementNoiseCovariance(double Δt, const StateT& state, const std::vector<nump::math::Circle>& measurementRegions) {
        BipedRobotModel::MeasurementCov Rt;
        Rt.eye();
        Rt(0,0) = 1e7;
        Rt(1,1) = 1e7;
        if (anyContain(measurementRegions, state)) {
            Rt(0,0) = 0.0001;
            Rt(1,1) = 0.0001;
        }
        return Rt;
    }


    namespace robotmodel {
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

        //    double farDist = 0.3;
        //    double nearDist = 0.1;
        //    double blend = std::max(0.0, std::min(1.0, (dist - nearDist) / (farDist - nearDist)));
        //    return blend * walkBetweenFar(x1, x2) + (1 - blend) * walkBetweenNear(x1, x2);

        //        if (dist > 0.2) {
            if (dist > 0.2 && std::abs(targetAngle) < M_PI*0.25) { // TODO: Choose walkBetweenFar as long as it satisfies the topological property.
                return walkBetweenFar(x1, x2);
            } else {
                return walkBetweenNear(x1, x2);
            }
        }
    }
    template <>
    StateT Trajectory<StateT>::sample(arma::vec2 mapSize) {
        arma::vec rvec = arma::randu(3);

        StateT result;
        result.position = { rvec(0)*mapSize(0), rvec(1)*mapSize(1), (rvec(2) * 2 - 1) * M_PI };
        return result;
    }

    template <>
    Trajectory<StateT>::Trajectory() {
        xInit.position.zeros();
        xGoal.position.zeros();
    }

    template <>
    StateT Trajectory<StateT>::operator() (double t) const {
        double timeStep = 0.01;
        Transform2D pos = xInit.position;

        for (double currTime = 0; currTime <= t; currTime += timeStep) {
            pos = pos.localToWorld(robotmodel::walkBetween(pos, xGoal.position) * timeStep);
        }

        return {pos};
    }

    template <>
    Trajectory<StateT> Trajectory<StateT>::fromEndpoints(StateT xInit, StateT xGoal) {
        Trajectory<StateT> traj;

        traj.xInit = xInit;
        traj.xGoal = xGoal;
        traj.reachesTarget = false;
        traj.t = 0;

        int maxSteps = 1000;
        double maxTimeStep = 0.02;
        double minTimeStep = maxTimeStep * 0.05;
        double goalThreshold = 0.005;
        double closeThreshold = goalThreshold * 100;

        Transform2D pos = xInit.position;

        double totalTime = 0;
        for (int numSteps = 0; numSteps < maxSteps; numSteps++) {
            double timeStep = (arma::norm(pos - xGoal.position) < closeThreshold) ? minTimeStep : maxTimeStep;

            pos = pos.localToWorld(robotmodel::walkBetween(pos, xGoal.position) * timeStep);

            totalTime += timeStep;
            traj.t = totalTime; // TODO: Consider removing.

            double goalDist = arma::norm(pos - xGoal.position);
            if (goalDist < goalThreshold) {
                traj.t = totalTime;
                traj.reachesTarget = true;
                break;
            }
        }

        return traj;
    }
}
