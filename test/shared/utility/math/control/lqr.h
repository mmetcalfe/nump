#include <armadillo>

#ifndef UTILITY_MATH_CONTROL_LQR_H
#define UTILITY_MATH_CONTROL_LQR_H

namespace utility {
namespace math {
namespace control {

    /* Returns the regulator matrix K_i after performing i iterations of value iteration.
    * See slide 9, "Value iteration solution to LQR", from:
    * http://www.cs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf
    *
    * Given:
    * x_t = state at time t. u_t = input/control at time t.
    *
    * Dynamical system:
    * x_{t+1} = A*x_t + B*u_t
    *
    * With cost function:
    * g(x_t, u_t) = x_t.t()*Q*x_t + u_t.t()*R*u_t
    *
    * Find optimal K s.t.:
    * u_t = u_d + K*(x_t - x_d)
    * where u_d and x_d are the desired control and state.
    */
    template <int stateSize, int controlSize> // state size and control size.
    struct LQR {
        typedef arma::mat::fixed<stateSize, stateSize> MatState2State;
        typedef arma::mat::fixed<stateSize, controlSize> MatControl2State;
        typedef arma::mat::fixed<controlSize, stateSize> MatState2Control;
        typedef arma::mat::fixed<controlSize, controlSize> MatControl2Control;

        static MatState2Control lqrValueIterationSolution(
            const MatState2State& A,
            const MatControl2State& B,
            const MatState2State& Q,
            const MatControl2Control& R,
            int numIterations
        ) {
            MatState2State P;
            MatState2Control K;

            // Set P_0 = 0:
            P.zeros();

            MatState2State ABK = A + B*K;
            MatState2State ABKt = ABK.t();

            for (int i = 1; i <= numIterations; i++) {
                // K = -(R + B.t()*P*B).i()*B.t()*P*A;
                // P = Q + K.t()*R*K + (A + B*K).t()*P*(A + B*K);
                MatState2Control BtP = B.t()*P;
                K = -(R + BtP*B).i()*BtP*A;
                P = Q + K.t()*R*K + ABKt*P*ABK;
            }

            // Note: x.t()*P*x is now the cost-to-go function for an i-step horizon.
            return K;
        }
    };


}
}
}


#endif //UTILITY_MATH_CONTROL_LQR_H
