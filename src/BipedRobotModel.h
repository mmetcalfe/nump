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

        typedef arma::mat33 MotionMatrix;
        typedef arma::mat33 MotionCov;
        typedef arma::mat33 MeasurementMatrix;
        typedef arma::mat33 MeasurementCov;

        /*
         * Note: Sensor and movement error model is:
         *
         *     x~t = At*x~p + Bt*u~p + wt,    wt ~ N(0, Qt)
         *     z~t = Ct*x~t + vt,             vt ~ N(0, Rt)
         *
         * (where \tilde{x}  -->  x~)
         */

        /// The A matrix in RRBT's propagate function.
        static MotionMatrix driftMatrix(double Δt, const State& state);

        /// The B matrix in RRBT's propagate function.
        static MotionMatrix controlMatrix(double Δt, const State& state);

        /// The K matrix in RRBT's propagate function.
        static MotionMatrix regulatorMatrix(double Δt);

        /// The C matrix in RRBT's propagate function.
        static MeasurementMatrix measurementMatrix(double Δt, const State& state);

        /// The Q matrix in RRBT's propagate function.
        static MotionCov motionNoiseCovariance(double Δt, const State& state);

        /// The R matrix in RRBT's propagate function.
        static MeasurementCov measurementNoiseCovariance(double Δt, const State& state);
    };
}

#endif //NUMP_ROBOT_MODEL_H
