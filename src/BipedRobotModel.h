//
// Created by Mitchell Metcalfe on 12/10/15.
//

#include "math/geometry.h"
#include <armadillo>

#ifndef NUMP_ROBOT_MODEL_H
#define NUMP_ROBOT_MODEL_H

namespace nump {

    using nump::math::Transform2D;

    struct BipedRobotModel {
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
        static RegulatorMatrix regulatorMatrix(double Δt);

        /// The C matrix in RRBT's propagate function.
        static MeasurementMatrix measurementErrorJacobian(double Δt, const State& state, const std::vector<nump::math::Circle>& measurementRegions);

        /// The Q matrix in RRBT's propagate function.
        static MotionCov motionNoiseCovariance(double Δt, const State& state, const Control& control);

        /// The R matrix in RRBT's propagate function.
        static MeasurementCov measurementNoiseCovariance(double Δt, const State& state, const std::vector<nump::math::Circle>& measurementRegions);
    };
}

#endif //NUMP_ROBOT_MODEL_H
