//
// Created by Mitchell Metcalfe on 12/10/15.
//

#include "math/geometry.h"
#include "SearchScenario.h"
#include <armadillo>

#ifndef NUMP_ROBOT_MODEL_H
#define NUMP_ROBOT_MODEL_H

namespace nump {

    using nump::math::Transform2D;
    using nump::math::RotatedRectangle;
    using nump::math::Circle;

    namespace robotmodel {
        Transform2D walkBetween(const Transform2D& x1, const Transform2D& x2);
    };

    struct BipedRobotModel {
        // typedef numptest::SearchScenario::Config::KickBox KickBox;

        struct KickBox {
            double kickExtent; // The distance the front of the foot travels during a kick.
            double footWidth; // The width of the robot's foot.
            double footSep; // The distance between the robots feet.
            double footFrontX; // The x coordinate of the front of the robot's feet before beginning a kick.
        };

        struct State {
            Transform2D position;
            // Transform2D velocity;
        };

        struct Control : public arma::vec2 {
            using arma::vec2::vec2;

            // Translational velocity:
            inline double v() const { return at(0); }
            inline double& v() { return at(0); }

            // Angular velocity:
            inline double omega() const { return at(1); }
            inline double& omega() { return at(1); }
        };

        struct Measurement : public arma::vec2 {
            using arma::vec2::vec2;

            inline double r() const { return at(0); }
            inline double& r() { return at(0); }
            inline double phi() const { return at(1); }
            inline double& phi() { return at(1); }
        };

        static bool canKickBall(RotatedRectangle robotFootprint, Circle ball, const KickBox& kbConfig);
        static bool canKickBallAtTarget(RotatedRectangle robotFootprint, Circle ball, const KickBox& kbConfig, double targetAngle, double validAngleRange);
        static bool canAlmostKickBallAtTarget(RotatedRectangle robotFootprint, Circle ball, const KickBox& kbConfig, double targetAngle, double validAngleRange);

        static std::vector<RotatedRectangle> getLocalKickBoxes(Transform2D robot, const KickBox& kbConfig, double ballRadius);

        static double kickSuccessProbability(
            Transform2D robot,
            arma::mat33 stateCov,
            const KickBox& kbConfig,
            Circle ball,
            double targetAngle,
            double targetAngleRange
        );

    private: // These types are just for clarity:
        static constexpr int stateSize = 3;
        static constexpr int controlSize = 2;
        typedef arma::mat::fixed<stateSize, stateSize> MatState2State;
        typedef arma::mat::fixed<stateSize, controlSize> MatControl2State;
        typedef arma::mat::fixed<controlSize, stateSize> MatState2Control;
        typedef arma::mat::fixed<controlSize, controlSize> MatControl2Control;

    public:
        typedef MatState2State MotionMatrix;
        typedef MatState2State MotionCov;
        typedef MatControl2State ControlMatrix;
        typedef MatControl2Control ControlCov;
        typedef MatState2Control MeasurementMatrix;
        typedef MatControl2Control MeasurementCov;
        typedef MatControl2State KalmanGainMatrix;
        typedef MatState2Control RegulatorMatrix;

        /*
         * Note: Sensor and movement error model is:
         *
         *     x~t = At*x~p + Bt*u~p + wt,    wt ~ N(0, Qt)
         *     z~t = Ct*x~t + vt,             vt ~ N(0, Rt)
         *
         * (where \tilde{x}  -->  x~)
         */

        /// The A matrix in RRBT's propagate function.
        static MotionMatrix motionErrorJacobian(double Δt, const State& state, const Control& control);

        /// The B matrix in RRBT's propagate function.
        static ControlMatrix controlErrorJacobian(double Δt, const State& state, const Control& control);

        /// The K matrix in RRBT's propagate function.
        static RegulatorMatrix regulatorMatrix(double Δt, const MotionMatrix& At, const ControlMatrix& Bt);

        /// The C matrix in RRBT's propagate function.
        static MeasurementMatrix measurementErrorJacobian(double Δt, const State& state, const std::vector<nump::math::Circle>& measurementRegions);

        /// The Q matrix in RRBT's propagate function.
        static MotionCov motionNoiseCovariance(double Δt, const State& state, const Control& control, const ControlMatrix& Bt);

        /// The R matrix in RRBT's propagate function.
        static MeasurementCov measurementNoiseCovariance(double Δt, const State& state, const Circle& landmark);
        static MeasurementCov measurementNoiseCovariance(double Δt, const State& state, const std::vector<nump::math::Circle>& measurementRegions);


        static Measurement observeLandmark(const State& state, const Circle& landmark);

        struct EKF {
            State mean;
            MotionCov covariance;

            void update(double Δt, Transform2D control, std::vector<Measurement> measurements, std::vector<Circle> landmarks);
        };
    };
}

#endif //NUMP_ROBOT_MODEL_H
