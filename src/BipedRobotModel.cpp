//
// Created by Mitchell Metcalfe on 12/10/15.
//

#include "BipedRobotModel.h"

#include <vector>
#include <cassert>
#include <armadillo>
#include "math/geometry.h"
#include "shared/utility/math/control/lqr.h"
#include "shared/utility/math/angle.h"
#include "shared/utility/math/distributions.h"
#include "shared/utility/math/quadrature.h"
#include "Trajectory.h"


namespace nump {
    typedef BipedRobotModel::State StateT;
    typedef BipedRobotModel::Control ControlT;
    typedef nump::BipedRobotModel::KickBox KickBox;

    using nump::math::Circle;
    using nump::math::RotatedRectangle;

    /// The A matrix in RRBT's propagate function.
    BipedRobotModel::MotionMatrix BipedRobotModel::motionErrorJacobian(double Δt, const StateT& state, const ControlT& control) {
        // The Jacobion of the error in the motion model.
        // (See G_t on page 206 of Sebastian Thrun's Probabilistic Robotics book)

        BipedRobotModel::MotionMatrix At;
        At.eye();

        double mu = state.position.angle();
        if (std::abs(control.omega()) > 1e-8) {
            double radius = control.v() / control.omega();
            At(0, 2) = radius * (-std::cos(mu) + std::cos(mu + control.omega()*Δt));
            At(1, 2) = radius * (-std::sin(mu) + std::sin(mu + control.omega()*Δt));
        } else { // Limits as omega -> 0, using l'Hopital's rule:
            At(0, 2) = -control.v()*Δt*std::sin(mu + control.omega()*Δt);
            At(1, 2) =  control.v()*Δt*std::cos(mu + control.omega()*Δt);
        }

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
        if (control.omega() > 1e-8) {
            Bt(0, 0) = (-st + stw)/wt;
            Bt(1, 0) = ( ct - ctw)/wt;
            Bt(0, 1) =  vt*(st - stw)/(wt*wt) + vt*ctw*Δt/wt;
            Bt(1, 1) = -vt*(ct - ctw)/(wt*wt) + vt*stw*Δt/wt;
        } else { // Limits as omega -> 0, using l'Hopital's rule:
            Bt(0, 0) = Δt*ct;
            Bt(1, 0) = Δt*st;
            Bt(0, 1) = vt*stw*Δt*Δt/2 - vt*Δt*st;
            Bt(1, 1) = vt*ctw*Δt*Δt/2 + vt*Δt*ct;
        }

        Bt(2, 0) = 0;
        Bt(2, 1) = Δt;

        return Bt;
    }

    /// The K matrix in RRBT's propagate function.
    BipedRobotModel::RegulatorMatrix BipedRobotModel::regulatorMatrix(double Δt, const MotionMatrix& At, const ControlMatrix& Bt) {
        MotionCov Qt;
        Qt.eye();
        ControlCov Rt;
        Rt.eye();
        Qt *= 10;
        Rt *= 0.01;

        int numIterations = 3;
        RegulatorMatrix Kt = utility::math::control::LQR<stateSize, controlSize>::lqrValueIterationSolution(At, Bt, Qt, Rt, numIterations);

        if (!arma::is_finite(Kt)) {
            std::cout << "satisfiesChanceConstraint: is_finite fail." << std::endl;
            std::cout << "Kt: " << Kt << std::endl;
            assert(false);
        } else if (std::abs(Kt(0,0)) > 1e10 || std::abs(Kt(0,1)) > 1e10 || std::abs(Kt(0,2)) > 1e10) {
            std::cout << "satisfiesChanceConstraint: std::abs(stateCov(x,y)) > 1e20 fail." << std::endl;
            std::cout << "Kt: " << Kt << std::endl;
            assert(false);
        }

        return -Kt;
    }

    /// The Q matrix in RRBT's propagate function.
    BipedRobotModel::MotionCov BipedRobotModel::motionNoiseCovariance(double Δt, const StateT& state, const ControlT& control, const ControlMatrix& Bt) {
        // The motion noise in control space mapped into state space.
        // (See M_t and V_t on page 206 of Sebastian Thrun's Probabilistic Robotics book)

        // Robot specific motion error parameters:
        // (See (5.10) on page 127 of Sebastian Thrun's Probabilistic Robotics book)
        // arma::mat22 alpha = {
        //     {0.02, 0.001},
        //     {0.001, 0.005}
        // };
        arma::mat22 alpha = {
            {0.1, 0.001},
            {0.001, 0.2}
        };
        ControlT controlSquared = control % control;
        BipedRobotModel::ControlCov Mt = arma::diagmat(alpha*controlSquared);

        // arma::vec4 alpha = { 10, 2, 2, 5 };
        // Mt.zeros();
        // Mt(0,0) = arma::vec(alpha.head(2).t() * controlSquared)(0); // α_0*v^2 + α_1*ω^2
        // Mt(1,1) = arma::vec(alpha.tail(2).t() * controlSquared)(0); // α_2*v^2 + α_3*ω^2

        // BipedRobotModel::MotionCov Qt = Vt*Mt*Vt.t();
        BipedRobotModel::MotionCov Qt = Bt*Mt*Bt.t();
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

    BipedRobotModel::MeasurementMatrix measurementErrorJacobian(double Δt, const StateT& state, const Circle& landmark) {
        arma::vec2 mu = state.position.xy();
        arma::vec2 loc = landmark.centre;

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

        return nump::measurementErrorJacobian(Δt, state, *landmark);
    }

    /// The R matrix in RRBT's propagate function.
    BipedRobotModel::MeasurementCov BipedRobotModel::measurementNoiseCovariance(double Δt, const StateT& state, const Circle& landmark) {
        auto expectedMeas = BipedRobotModel::observeLandmark(state, landmark);
        double r = expectedMeas.r();
        BipedRobotModel::MeasurementCov Rt;
        Rt.eye();
        if (std::abs(expectedMeas.phi()) > arma::datum::pi/2) {
            Rt(0,0) = 1e7;
            Rt(1,1) = 1e7;
        } else {
            // Rt(0,0) = 0.01 * r*r;
            Rt(0,0) = 0.02 * (r*r);
            Rt(1,1) = 0.001;
        }
        return Rt;
    }
    BipedRobotModel::MeasurementCov BipedRobotModel::measurementNoiseCovariance(double Δt, const StateT& state, const std::vector<Circle>& measurementRegions) {
        auto landmark = firstContaining(measurementRegions, state);
        if (landmark != nullptr) {
            return measurementNoiseCovariance(Δt, state, *landmark);
        } else {
            BipedRobotModel::MeasurementCov Rt;
            Rt.eye();
            Rt(0,0) = 1e7;
            Rt(1,1) = 1e7;
            return Rt;
        }
    }

    BipedRobotModel::Measurement BipedRobotModel::observeLandmark(const State& state, const Circle& landmark) {
        Transform2D local = state.position.worldToLocal({landmark.centre, 0});
        Measurement meas = {0, 0};
        meas.r() = arma::norm(local.xy());
        meas.phi() = utility::math::angle::normalizeAngle(std::atan2(local.y(), local.x()));
        return meas;
    }

    void BipedRobotModel::EKF::update(double Δt, Transform2D bipedControl,
                                      std::vector<std::pair<Measurement, BipedRobotModel::MeasurementCov>> measurements,
                                      std::vector<Circle> landmarks) {
        arma::vec2 control = {arma::norm(bipedControl.xy()), bipedControl.angle()};

        Transform2D μbt = mean.position.localToWorld(Δt*bipedControl);

        const BipedRobotModel::MotionCov& Σp = covariance;

        // Gt
        BipedRobotModel::MotionMatrix At = motionErrorJacobian(Δt, mean, control);
        // Vt
        BipedRobotModel::ControlMatrix Bt = controlErrorJacobian(Δt, mean, control);
        // VtMtVt^T
        BipedRobotModel::MotionCov Qt = motionNoiseCovariance(Δt, mean, control, Bt);

        // Step 1 - Covariance prediction (equations 21, 33):
        // Kalman filter process step:
        BipedRobotModel::MotionCov Σbt = At*Σp*At.t() + Qt; // (equation 17)

        for (int i = 0; i < measurements.size(); i++) {
            const Measurement& meas = measurements[i].first;
            const Circle& landmark = landmarks[i];

            const BipedRobotModel::MeasurementCov& Rt = measurements[i].second; // measurementNoiseCovariance(Δt, mean, landmark);

            // z_t
            auto expectedMeas = observeLandmark({μbt}, landmark);

            // Ht
            BipedRobotModel::MeasurementMatrix Ct = nump::measurementErrorJacobian(Δt, {μbt}, landmark);

//            if (std::abs(expectedMeas.phi()) > arma::datum::pi/2) {
//                continue;
//            }

            // Kalman filter measurement update:
            BipedRobotModel::MeasurementCov St = Ct*Σbt*Ct.t() + Rt;
            BipedRobotModel::KalmanGainMatrix Lt = Σbt*Ct.t()*St.i();

            Transform2D μbtBefore = μbt;
            BipedRobotModel::MotionCov ΣbtBefore = Σbt;

            μbt = μbt + Lt*(meas - expectedMeas);
            Σbt = Σbt - Lt*Ct*Σbt;

            // if (arma::norm(μbt) > 20) {
            if (!Σbt.is_finite() || !μbt.is_finite()) {
                std::cout << "mean" << mean.position.t() << std::endl;
                std::cout << "control" << control.t() << std::endl;
                std::cout << "μbtBefore" << μbtBefore.t() << std::endl;
                std::cout << "ΣbtBefore" << Σbt << std::endl;
                std::cout << "meas: " << meas.t() << std::endl;
                std::cout << "expectedMeas: " << expectedMeas.t() << std::endl;
                std::cout << "At: " << At << std::endl;
                std::cout << "Bt: " << Bt << std::endl;
                std::cout << "Qt: " << Qt << std::endl;
                std::cout << "Ct: " << Ct << std::endl;
                std::cout << "Rt: " << Rt << std::endl;
                std::cout << "St: " << St << std::endl;
                std::cout << "Lt: " << Lt << std::endl;
                std::cout << "μbt: " << μbt << std::endl;
                std::cout << "Σbt: " << Σbt << std::endl;
            }
        }

        // if (arma::norm(mean.position) > 20) {
        //     std::cout << "arma::norm(mean.position) > 20: " << mean.position << std::endl;
        //     std::cout << "covariance: " << covariance << std::endl;
        // }

        mean.position = μbt;
        covariance = Σbt;

        // std::cout << "EKF covariance: " << covariance << std::endl;
    }


    bool BipedRobotModel::canKickBall(RotatedRectangle robotFootprint, Circle ball, const KickBox& kbConfig) {
        Circle localBall = {robotFootprint.transform.worldToLocal({ball.centre,0}).xy(), ball.radius};

        if (localBall.centre(0) < kbConfig.footFrontX + ball.radius) {
            return false;
        }

        if (localBall.centre(0) > kbConfig.footFrontX + kbConfig.kickExtent + ball.radius) {
            return false;
        }

        if (std::abs(localBall.centre(1)) < kbConfig.footSep*0.5) {
            return false;
        }

        if (std::abs(localBall.centre(1)) > kbConfig.footSep*0.5 + kbConfig.footWidth) {
            return false;
        }

        return true;
    }

    bool BipedRobotModel::canKickBallAtTarget(RotatedRectangle robotFootprint, Circle ball, const KickBox& kbConfig, double targetAngle, double validAngleRange) {
        if (!canKickBall(robotFootprint, ball, kbConfig)) {
            return false;
        }

        double minAngleRange = utility::math::angle::normalizeAngle(targetAngle - validAngleRange*0.5);
        double maxAngleRange = utility::math::angle::normalizeAngle(targetAngle + validAngleRange*0.5);
        double angle = utility::math::angle::normalizeAngle(robotFootprint.transform.angle());

        if (utility::math::angle::signedDifference(angle, minAngleRange) < 0) {
            return false;
        }
        if (utility::math::angle::signedDifference(angle, maxAngleRange) > 0) {
            return false;
        }

        return true;
    }

    bool BipedRobotModel::canAlmostKickBallAtTarget(RotatedRectangle robotFootprint, Circle ball, const KickBox& kbConfig, double targetAngle, double validAngleRange) {
        // TODO: Rewrite to call canKickBall and pass a tolerance.
        Circle localBall = {robotFootprint.transform.worldToLocal({ball.centre,0}).xy(), ball.radius};

        if (localBall.centre(0) < kbConfig.footFrontX + ball.radius) {
            return false;
        }

        if (localBall.centre(0) > kbConfig.footFrontX + kbConfig.kickExtent + ball.radius) {
            return false;
        }

        if (std::abs(localBall.centre(1)) > kbConfig.footSep*0.5 + kbConfig.footWidth) {
            return false;
        }

        double minAngleRange = utility::math::angle::normalizeAngle(targetAngle - validAngleRange*0.5);
        double maxAngleRange = utility::math::angle::normalizeAngle(targetAngle + validAngleRange*0.5);
        double angle = utility::math::angle::normalizeAngle(robotFootprint.transform.angle());

        if (utility::math::angle::signedDifference(angle, minAngleRange) < 0) {
            return false;
        }
        if (utility::math::angle::signedDifference(angle, maxAngleRange) > 0) {
            return false;
        }

        return true;
    }


    std::vector<RotatedRectangle> BipedRobotModel::getLocalKickBoxes(const KickBox& kbConfig, double ballRadius) {
        double kbX = kbConfig.footFrontX+ballRadius+kbConfig.kickExtent*0.5;
        double kbY = kbConfig.footSep*0.5 + kbConfig.footWidth*0.5;
        RotatedRectangle left = {{kbX, -kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}};
        RotatedRectangle right = {{kbX, kbY, 0}, {kbConfig.kickExtent, kbConfig.footWidth}};
        return {left, right};
    }

    double BipedRobotModel::kickSuccessProbability(
        Transform2D robot,
        arma::mat33 stateCov,
        const KickBox& kbConfig,
        Circle ball,
        double targetAngle,
        double targetAngleRange
    ) {
        Transform2D globalBallTarget = {ball.centre, targetAngle};
        Transform2D localBallTarget = robot.worldToLocal(globalBallTarget);

        arma::mat33 localBallTargetCov = utility::math::distributions::transformToLocalDistribution(robot, stateCov, globalBallTarget);

        auto kickBoxes = BipedRobotModel::getLocalKickBoxes(kbConfig, ball.radius);
        auto densityFunc = [=](auto x){ return utility::math::distributions::dnorm(localBallTarget, localBallTargetCov, x); };

        int order = 4;
        double leftKickBoxProb = utility::math::quadrature::integrateGaussQuad(densityFunc, kickBoxes[0], targetAngleRange, {order, order, order});
        double rightKickBoxProb = utility::math::quadrature::integrateGaussQuad(densityFunc, kickBoxes[1], targetAngleRange, {order, order, order});

        double kickProb = leftKickBoxProb + rightKickBoxProb;

        return kickProb;
    }

    BipedRobotModel::State BipedRobotModel::getIdealKickingPosition(Circle ball, const KickBox& kbConfig, double targetAngle, double offset) {
        Transform2D globalBallTarget = {ball.centre, targetAngle};
        auto kickBoxes = BipedRobotModel::getLocalKickBoxes(kbConfig, ball.radius);
        // Transform2D kickPos = globalBallTarget.localToWorld({-kickBoxes[0].transform.xy(), 0});
        // return {kickPos};

        double kickBoxLength = kickBoxes[0].size(0);
        double kbCentreX = kickBoxes[0].transform.x();
        double kbMax = kbCentreX + kickBoxLength*0.5;
        double effectiveKbMin = kbCentreX - kickBoxLength*0.5 + offset;
        double localX = (kbMax + effectiveKbMin)*0.5;
        Transform2D kickPos = globalBallTarget.localToWorld({-localX, -kickBoxes[0].transform.y(), 0});
        return {kickPos};
    }


    namespace robotmodel {
        // double walk_far_translation_speed = 0.25;
        // double walk_far_rotation_speed = ((2*arma::datum::pi)/8)/5;
        // double walk_near_rotation_speed = ((2*arma::datum::pi)/8)/2;
        // double walk_near_translation_speed = 0.05;

        double walk_far_translation_speed = 0.1;
        double walk_far_rotation_speed = ((2*arma::datum::pi)/8)/2;
        double walk_near_rotation_speed = ((2*arma::datum::pi)/8)/1.2;
        double walk_near_translation_speed = 0.05;

        Transform2D walkBetweenFar(const Transform2D& currentState, const Transform2D& targetState) {
            auto diff = arma::vec2(targetState.xy() - currentState.xy());
            auto dir = utility::math::angle::vectorToBearing(diff);
            double wcAngle = utility::math::angle::signedDifference(dir, currentState.angle());
            int angleSign = (wcAngle < 0) ? -1 : 1;

            double rotationSpeed = angleSign * walk_far_rotation_speed;
            double forwardSpeed = std::cos(wcAngle) * walk_far_translation_speed;

            // {
            //     Transform2D localTarget = currentState.worldToLocal(targetState);
            //     double a = 1;
            //     double b = 2*localTarget.x()+1;
            //     double c = arma::vec(localTarget.xy().t()*localTarget.xy())(0);
            //
            //     double r = (-b + std::sqrt(b*b-4*a*c))/(2*a);
            //     forwardSpeed = std::min(r*walk_far_rotation_speed, walk_far_translation_speed);
            // }

            return {std::max(0.0, forwardSpeed), 0, rotationSpeed};
            // return {forwardSpeed, 0, rotationSpeed};
            //    return {1, 0, rotationSpeed};
        }

        Transform2D walkBetweenNear(const Transform2D& currentState, const Transform2D& targetState) {
            Transform2D localTarget = currentState.worldToLocal(targetState);

            int angleSign = (localTarget.angle() < 0) ? -1 : 1; // angle must be normalised.
        //        // TODO: Consider using a smaller, non-constant speed.
            double rotationSpeed = angleSign * walk_near_rotation_speed;

            arma::vec2 translationVec = arma::normalise(localTarget.xy());
            double translationAngle = utility::math::angle::vectorToBearing(translationVec);
            double translationSpeed = walk_near_translation_speed*(1 - std::abs(translationAngle)*(0.25/M_PI)); // TODO: Ensure translationSpeed matches the distance metrics used.
            arma::vec2 translationVelocity = translationVec * translationSpeed;
            Transform2D velocity = {translationVelocity, rotationSpeed};
            return velocity;
        }

        Transform2D walkBetween(const Transform2D& x1, const Transform2D& x2) {
            Transform2D localTarget = x1.worldToLocal(x2);
            auto targetAngle = utility::math::angle::normalizeAngle(utility::math::angle::vectorToBearing(localTarget.xy()));

            double dist = arma::norm(localTarget.xy());

        //    double farDist = 0.3;
        //    double nearDist = 0.1;
        //    double blend = std::max(0.0, std::min(1.0, (dist - nearDist) / (farDist - nearDist)));
        //    return blend * walkBetweenFar(x1, x2) + (1 - blend) * walkBetweenNear(x1, x2);

            // TODO: Choose walkBetweenFar as long as it satisfies the topological property.
            // if (dist > 0.2 && std::abs(targetAngle) < M_PI*0.25) {

            // double d_near = walk_far_rotation_speed / walk_far_translation_speed;
            double d_near = 0.2;

            if (dist < d_near) {
                return walkBetweenNear(x1, x2);
            } else {
                return walkBetweenFar(x1, x2);
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
    StateT Trajectory<StateT>::operator() (double t, double timeStep) const {
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
        double maxTimeStep = 0.1;
        double minTimeStep = maxTimeStep * 1;

        double goalAngleTolerance = arma::datum::pi/16;
        double goalDistanceTolerance = 0.01;

        double closeAngleThreshold = maxTimeStep * 1;
        double closeDistanceThreshold = maxTimeStep * 1;

        Transform2D pos = xInit.position;

        double totalTime = 0;
        for (int numSteps = 0; numSteps < maxSteps; numSteps++) {
            Transform2D diff = arma::abs(pos - xGoal.position);
            diff.angle() = std::abs(utility::math::angle::normalizeAngle(diff.angle()));

            // If we have reached the goal, return.
            bool angleGoalAchieved = diff.angle() < goalAngleTolerance;
            bool positionGoalAchieved = arma::norm(diff.xy()) < goalDistanceTolerance;
            if (totalTime > 0 && angleGoalAchieved && positionGoalAchieved) {
                traj.t = totalTime;
                traj.reachesTarget = true;
                break;
            }

            double timeStep = maxTimeStep;
            if (arma::norm(diff.xy()) < closeDistanceThreshold || diff.angle() < closeAngleThreshold) {
                timeStep = minTimeStep;
            }

            Transform2D control = robotmodel::walkBetween(pos, xGoal.position);

            arma::vec3 timeSteps = arma::abs(arma::vec3 {
                diff.x() / control.x(),
                diff.y() / control.y(),
                diff.angle() / control.angle(),
            });
            pos = pos.localToWorld(control * std::min(timeStep, arma::max(timeSteps)));
            // pos = pos.localToWorld(control * std::min(std::min(timeSteps(0), timeSteps(1)), std::min(timeSteps(2), timeSteps(3))));
            // pos = pos.localToWorld(control * timeStep);

            totalTime += timeStep;
            traj.t = totalTime; // TODO: Consider removing.
        }

        return traj;
    }

    // struct TrajectoryWalker {
    //     StateT xCurrent;
    //     StateT xNext;
    //     double t = 0;
    //     double finishTime = 0;

        // Modifies the trajectory to advance its initial state by a single step of t seconds.
    template <>
    StateT Trajectory<StateT>::TrajectoryWalker::getNext(const StateT& current, double deltaT) const {
        Transform2D pos = current.position;
        pos = pos.localToWorld(robotmodel::walkBetween(pos, xGoal.position) * deltaT);
        return StateT { pos };
    }
    template <>
    void Trajectory<StateT>::TrajectoryWalker::stepBy() {
        double nextTimeStep = std::min(timeStep, finishTime - t);

        xCurrent = xNext;
        xNext = getNext(xCurrent, nextTimeStep);
        t += nextTimeStep;
    }
    template <>
    arma::vec2 Trajectory<StateT>::TrajectoryWalker::currentControl() {
        Transform2D velTraj = xNext.position - xCurrent.position;
        double angleDiff = utility::math::angle::signedDifference(xNext.position.angle(), xCurrent.position.angle());
        ControlT controlTraj = {
            arma::norm(velTraj.xy()) / timeStep,
            angleDiff / timeStep
        };

        return controlTraj;
    }
    template <>
    Transform2D Trajectory<StateT>::TrajectoryWalker::currentOmniControl() {
        Transform2D velTraj = xNext.position - xCurrent.position;
        double angleDiff = utility::math::angle::signedDifference(xNext.position.angle(), xCurrent.position.angle());
        velTraj.angle() = angleDiff;
        return velTraj / timeStep;
    }
    template <>
    StateT Trajectory<StateT>::TrajectoryWalker::currentState() {
        return xCurrent;
    }
    template <>
    double Trajectory<StateT>::TrajectoryWalker::currentTime() {
        return t;
    }
    template <>
    bool Trajectory<StateT>::TrajectoryWalker::isFinished() {
        return t >= finishTime - timeStep*0.001;
    }

    template <>
    Trajectory<StateT>::TrajectoryWalker Trajectory<StateT>::walk(double timeStep) const {
        Trajectory<StateT>::TrajectoryWalker walker;

        walker.finishTime = t;
        walker.t = 0;
        walker.timeStep = timeStep;
        walker.xCurrent = xInit;
        walker.xGoal = xGoal;
        walker.xNext = walker.getNext(xInit, timeStep);

        return walker;
    }

    template <>
    Path<StateT>::Walker Path<StateT>::walk(double timeStep) const {
        Path<StateT>::Walker walker;

        walker.currentSegment = segments.begin();
        walker.endSegment = segments.end();
        walker.currentWalker = walker.currentSegment->walk(timeStep);
        walker.t = 0;
        walker.timeStep = timeStep;

        return walker;
    }

    template <>
    arma::vec2 Path<StateT>::Walker::currentControl() {
        return currentWalker.currentControl();
    }
    template <>
    Transform2D Path<StateT>::Walker::currentOmniControl() {
        return currentWalker.currentOmniControl();
    }
    template <>
    StateT Path<StateT>::Walker::currentState() {
        return currentWalker.currentState();
    }
    template <>
    bool Path<StateT>::Walker::isFinished() {
        return currentSegment == endSegment;
    }

    template <>
    void Path<StateT>::Walker::stepTo(double targetTime) {
        while (t < targetTime) {
            if (isFinished()) {
                return;
            }

            if (currentWalker.isFinished()) {
                ++currentSegment;
                currentWalker = currentSegment->walk(timeStep);
            }

            currentWalker.stepBy();

            t += timeStep;
        }
    }

    template <>
    void Path<StateT>::Walker::stepBy(double deltaT) {
        stepTo(t+deltaT);
    }
}
