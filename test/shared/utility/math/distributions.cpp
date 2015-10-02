//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#include "shared/utility/math/distributions.h"
#include <cassert>
#include "shared/utility/math/geometry/Ellipse.h"

namespace utility {
namespace math {
namespace distributions {

using nump::math::Transform2D;

double cChiSquare(double percentile, int dof) {
    double chiSquareVal;
    switch (dof) {
        case 1: chiSquareVal = 3.8415; break; // 95th percentile of 1-DOF chi-square distribution.
        case 2: chiSquareVal = 5.991; break; // 95th percentile of 2-DOF chi-square distribution.
        case 3: chiSquareVal = 7.815; break; // 95th percentile of 3-DOF chi-square distribution.
        default:
            std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
                      << "Unsupported DOF value ("
                      << dof << ")."
                      << std::endl;
            break;
    }
    return chiSquareVal;
}

arma::vec2 confidenceRegion(double mean, double var, double conf, int dof) {

    double chiSquareVal = cChiSquare(conf, dof);

    double hw = std::sqrt(chiSquareVal*var);

    return {mean - hw, mean + hw};
}

arma::vec2 eigvals_2d(const arma::mat22& cov) {
    double T = arma::trace(cov); //cov[0][0] + cov[1][1];
    double D = arma::det(cov);

    double lshared = std::sqrt(T*T*0.25-D);
    double L1 = T*0.5 + lshared;
    double L2 = T*0.5 - lshared;

    if (L2 > L1) {
        double tmp = L2;
        L2 = L1;
        L1 = tmp;
    }

    return {L1, L2};
}

void eig_sym_2d(arma::vec2& eigval, arma::mat22& eigvec, const arma::mat22& cov) {
    double b = cov(0,1);
    double c = cov(1,0);

    double eps = 1e-6;
    if (c <= eps && b <= eps) {
        if (cov(0,0) > cov(1,1)) {
            eigval = { cov(0,0), cov(1,1) };
            eigvec.col(0) = arma::vec2({1, 0});
            eigvec.col(1) = arma::vec2({0, 1});
        } else {
            eigval = { cov(1,1), cov(0,0) };
            eigvec.col(0) = arma::vec2({0, 1});
            eigvec.col(1) = arma::vec2({1, 0});
        }
        return;
    }

    eigval = eigvals_2d(cov);

    if (c > eps) {
        eigvec.col(0) = arma::vec2({eigval(0)-cov(1, 1), cov(1, 0)});
        eigvec.col(1) = arma::vec2({eigval(1)-cov(1, 1), cov(1, 0)});
        return;
    } else if (b > eps) {
        eigvec.col(0) = arma::vec2({cov(0, 1), eigval(0)-cov(0, 0)});
        eigvec.col(1) = arma::vec2({cov(0, 1), eigval(1)-cov(0, 0)});
        return;
    }

    // This code should be unreachable.
    std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
              << "Cannot find eigenvalues."
              << std::endl;
    assert(false);
}

Ellipse confidenceRegion(arma::vec2 mean, arma::mat22 cov, double conf, int dof) {
    arma::vec2 eigval;  // eigenvalues are stored in ascending order.
    arma::mat22 eigvec;
    arma::eig_sym(eigval, eigvec, cov, "std");
    // eig_sym_2d(eigval, eigvec, cov);

    // std::cout << __FILE__ << ", " << __LINE__ << ": eigvals: " << eigval.t() << std::endl;
    // std::cout << __FILE__ << ", " << __LINE__ << ": eigvec: " << eigvec.t() << std::endl;

    if (eigval(0) < 0 || eigval(1) < 0) {
        std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
        << "Eigenvalues of covariance matrix must not be negative, but"
        << " eigval = "
        << eigval.t()
        << std::endl;
        assert(false);
    }

    arma::vec2 primaryAxis = arma::vec(eigvec.col(1));
    double angle = std::atan2(primaryAxis(1), primaryAxis(0));

    double chiSquareVal = cChiSquare(conf, dof);
    // double chiSquareVal = 5.991; // for 95% confidence interval
    arma::vec axisLengths = 2*arma::sqrt(chiSquareVal*eigval);
    arma::vec2 size = {axisLengths(1), axisLengths(0)};

    Transform2D trans = {mean, angle};
    return {trans, size};
}

double confidenceRegionArea(arma::mat22 cov, double conf, int dof) {
    arma::vec2 eigval = eigvals_2d(cov);
    double chiSquareVal = cChiSquare(conf, dof);
    // double chiSquareVal = 5.991; // for 95% confidence interval
    arma::vec halfAxisLengths = arma::sqrt(chiSquareVal*eigval);
    return arma::datum::pi * arma::prod(halfAxisLengths);
}

}
}
}
