//
// Created by Mitchell Metcalfe on 12/10/15.
//

#include "BipedRobotModel.h"

#include <vector>
#include <armadillo>
#include "math/geometry.h"
#include "shared/utility/math/control/lqr.h"
#include "shared/utility/math/angle.h"
#include "Trajectory.h"

namespace nump {
    typedef BipedRobotModel::State StateT;
    typedef BipedRobotModel::Control ControlT;

    using nump::math::Circle;

    /// The A matrix in RRBT's propagate function.
    BipedRobotModel::MotionMatrix BipedRobotModel::motionErrorJacobian(double Δt, const StateT& state, const ControlT& control) {
        // The Jacobion of the error in the motion model.
        // (See G_t on page 206 of Sebastian Thrun's Probabilistic Robotics book)

        BipedRobotModel::MotionMatrix At;
        At.eye();

        double radius = control.v() / control.omega();
        double mu = state.position.angle();
        At(0, 2) = radius * (-std::cos(mu) + std::cos(mu + control.omega()*Δt));
        At(1, 2) = radius * (-std::sin(mu) + std::sin(mu + control.omega()*Δt));

        return At;
    }

    /// The B matrix in RRBT's propagate function.
    BipedRobotModel::ControlMatrix BipedRobotModel::controlErrorJacobian(double Δt, const StateT& state, const ControlT& control) {

        double ctw = std::cos(state.position.angle() + control.omega()*Δt);
        double stw = std::sin(state.position.angle() + control.omega()*Δt);
        double ct = std::cos(state.position.angle());
        double st = std::sin(state.position.angle());
        double vt = control.v();
        double wt = control.omega();

        // Note: This is the same as Vt.
        BipedRobotModel::ControlMatrix Bt;
        Bt.zeros();
        Bt(0, 0) = (-st + stw)/wt;
        Bt(1, 0) = ( ct - ctw)/wt;
        Bt(0, 1) =  vt*(st - stw)/(wt*wt) + vt*ctw*Δt/wt;
        Bt(1, 1) = -vt*(ct - ctw)/(wt*wt) + vt*stw*Δt/wt;
        Bt(2, 0) = 0;
        Bt(2, 1) = Δt;

        return Bt;
    }

    /// The K matrix in RRBT's propagate function.
    BipedRobotModel::RegulatorMatrix BipedRobotModel::regulatorMatrix(double Δt, const StateT& state, const ControlT& control) {
        // BipedRobotModel::RegulatorMatrix Kt;
        // Kt.zeros();
        //
        // // TODO: Implement a reasonable regulator matrix.
        //
        // double ct = std::cos(state.position.angle());
        // double st = std::sin(state.position.angle());
        //
        // // double l = std::sqrt(ct*ct + st*st);
        //
        // Kt(0,0) = 0;
        // Kt(0,1) = 0;
        // Kt(1,2) = 0; //-Δt; // Negate angle error.


        MotionMatrix At = motionErrorJacobian(Δt, state, control);
        ControlMatrix Bt = controlErrorJacobian(Δt, state, control);

        MotionCov Qt;
        Qt.eye();
        ControlCov Rt;
        Rt.eye();
        Qt *= 10;
        Rt *= 0.01;

        RegulatorMatrix Kt = utility::math::control::LQR<stateSize, controlSize>::lqrValueIterationSolution(At, Bt, Qt, Rt, 10);

        return -Kt;
    }

    /// The Q matrix in RRBT's propagate function.
    BipedRobotModel::MotionCov BipedRobotModel::motionNoiseCovariance(double Δt, const StateT& state, const ControlT& control) {
        // The motion noise in control space mapped into state space.
        // (See M_t and V_t on page 206 of Sebastian Thrun's Probabilistic Robotics book)

        // Robot specific motion error parameters:
        // (See (5.10) on page 127 of Sebastian Thrun's Probabilistic Robotics book)
        // arma::vec4 alpha = { 10, 2, 2, 5 };
        arma::vec4 alpha = { 200, 2, 2, 200 };

        BipedRobotModel::ControlCov Mt;
        Mt.zeros();
        Mt(0,0) = arma::vec(alpha.head(2).t() * (control % control))(0); // α_0*v^2 + α_1*ω^2
        Mt(1,1) = arma::vec(alpha.tail(2).t() * (control % control))(0); // α_2*v^2 + α_3*ω^2

        std::cout << "Mt" << Mt << std::endl;


        double ctw = std::cos(state.position.angle() + control.omega()*Δt);
        double stw = std::sin(state.position.angle() + control.omega()*Δt);
        double ct = std::cos(state.position.angle());
        double st = std::sin(state.position.angle());
        double vt = control.v();
        double wt = control.omega();

        BipedRobotModel::ControlMatrix Vt;
        Vt.zeros();
        Vt(0, 0) = (-st + stw)/wt;
        Vt(1, 0) = ( ct - ctw)/wt;
        Vt(0, 1) =  vt*(st - stw)/(wt*wt) + vt*ctw*Δt/wt;
        Vt(1, 1) = -vt*(ct - ctw)/(wt*wt) + vt*stw*Δt/wt;
        Vt(2, 0) = 0;
        Vt(2, 1) = Δt;

        BipedRobotModel::MotionCov Qt = Vt*Mt*Vt.t();
        return Qt;
    }

    std::unique_ptr<Circle> firstContaining(const std::vector<Circle>& regions, const StateT& pos) {
        for (auto &reg : regions) {
            if (reg.contains(pos.position.head(2))) {
                return std::make_unique<Circle>(reg);;
            }
        }

        return nullptr;
    }

    /// The C matrix in RRBT's propagate function.
    BipedRobotModel::MeasurementMatrix BipedRobotModel::measurementErrorJacobian(double Δt, const StateT& state, const std::vector<Circle>& measurementRegions) {
        // The Jacobion of the measurement model.
        // (See H^i_t on page 207 of Sebastian Thrun's Probabilistic Robotics book)

        auto landmark = firstContaining(measurementRegions, state);

        if (landmark == nullptr) {
            BipedRobotModel::MeasurementMatrix Ct;
            Ct.eye();
            return Ct;
        }

        arma::vec2 mu = state.position.xy();
        arma::vec2 loc = landmark->centre;

        double dist = arma::norm(mu - loc);
        double lx = loc(0);
        double ly = loc(1);
        double sx = mu(0);
        double sy = mu(1);

        BipedRobotModel::MeasurementMatrix Ct;
        Ct.zeros();

        Ct(0, 0) = -(lx - sx) / dist;
        Ct(0, 1) = -(ly - sy) / dist;
        Ct(1, 0) =  (ly - sy) / (dist*dist);
        Ct(1, 1) = -(lx - sx) / (dist*dist);

        Ct(1, 2) = -1;

        // Ct.eye(); // TODO: Remove test code.

        return Ct;
    }

    /// The R matrix in RRBT's propagate function.
    BipedRobotModel::MeasurementCov BipedRobotModel::measurementNoiseCovariance(double Δt, const StateT& state, const std::vector<Circle>& measurementRegions) {
        BipedRobotModel::MeasurementCov Rt;
        Rt.eye();
        Rt(0,0) = 1e7;
        Rt(1,1) = 1e7;
        if (firstContaining(measurementRegions, state) != nullptr) {
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
