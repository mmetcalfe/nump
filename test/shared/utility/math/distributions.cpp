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

// Generated in R using: qchisq(seq(0, 1, 0.01), 3, lower.tail=TRUE)
std::vector<double> cumulativeChiSquare3DofTable = {
    0.0000000, 0.1148318, 0.1848318, 0.2450987, 0.3001514, 0.3518463, 0.4011734,
    0.4487387, 0.4949476, 0.5400880, 0.5843744, 0.6279721, 0.6710124, 0.7136022,
    0.7558302, 0.7977714, 0.8394903, 0.8810430, 0.9224790, 0.9638427, 1.0051740,
    1.0465095, 1.0878828, 1.1293252, 1.1708660, 1.2125329, 1.2543524, 1.2963499,
    1.3385499, 1.3809763, 1.4236522, 1.4666009, 1.5098449, 1.5534070, 1.5973096,
    1.6415756, 1.6862278, 1.7312894, 1.7767839, 1.8227354, 1.8691684, 1.9161081,
    1.9635806, 2.0116124, 2.0602315, 2.1094665, 2.1593473, 2.2099052, 2.2611725,
    2.3131835, 2.3659739, 2.4195812, 2.4740450, 2.5294071, 2.5857116, 2.6430053,
    2.7013378, 2.7607619, 2.8213339, 2.8831139, 2.9461661, 3.0105593, 3.0763677,
    3.1436708, 3.2125547, 3.2831125, 3.3554449, 3.4296617, 3.5058824, 3.5842376,
    3.6648708, 3.7479394, 3.8336174, 3.9220975, 4.0135936, 4.1083449, 4.2066193,
    4.3087186, 4.4149844, 4.5258056, 4.6416277, 4.7629638, 4.8904101, 5.0246641,
    5.1665493, 5.3170478, 5.4773439, 5.6488837, 5.8334589, 6.0333271, 6.2513886,
    6.4914577, 6.7586926, 7.0603142, 7.4068800, 7.8147279, 8.3111709, 8.9472875,
    9.8374093,11.3448667, 1e4 // Note: qchisq(1, N) = Infinity
};

inline double qChiSquare3FromTable(double quantile) {
    double percent = quantile*100;
    int minIndex = std::max(int(std::floor(percent)), 0);
    int maxIndex = std::min(int(std::ceil(percent)), 100);

    double minVal = cumulativeChiSquare3DofTable[minIndex];
    double maxVal = cumulativeChiSquare3DofTable[maxIndex];
    double f = percent - minIndex;

    return (1-f)*minVal + f*maxVal;
}

double cChiSquare(double percentile, int dof) {
    double chiSquareVal;

    switch (dof) {
        case 1: chiSquareVal = 3.8415; break; // 95th percentile of 1-DOF chi-square distribution.
        case 2: chiSquareVal = 5.991; break; // 95th percentile of 2-DOF chi-square distribution.
        case 3: { // 3-DOF chi-square distribution.
            return qChiSquare3FromTable(percentile);
            // if (percentile >= 0.9) {
            //     chiSquareVal = 7.8147279; // 95th percentile
            // } else if (percentile >= 0.8) {
            //     chiSquareVal = 5.3170478; // 85th percentile
            // }else if (percentile >= 0.7) {
            //     chiSquareVal = 4.1083449; // 75th percentile
            // // } else if (percentile >= 0.6) {
            // } else {
            //     chiSquareVal = 3.2831125; // 65th percentile
            // }
        } break;
        default:
            std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
                      << "Unsupported DOF value ("
                      << dof << ")."
                      << std::endl;
            assert(false);
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

    if (eigval(0) < 0 || eigval(1) < 0 || !eigval.is_finite()) {
        std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
        << "Eigenvalues of covariance matrix must not be negative, but"
        << " eigval = "
        << eigval.t()
        << std::endl
        << " cov = "
        << cov
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
//    arma::vec2 eigval = eigvals_2d(cov);
    arma::vec2 eigval;  // eigenvalues are stored in ascending order.
    arma::mat22 eigvec;
    arma::eig_sym(eigval, eigvec, cov, "std");

    if (eigval(0) < 0 || eigval(1) < 0 || !eigval.is_finite()) {
        std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
        << "Eigenvalues of covariance matrix must not be negative, but"
        << " eigval = "
        << eigval.t()
        << std::endl
        << " cov = "
        << cov
        << std::endl;
        assert(false);
    }

    double chiSquareVal = cChiSquare(conf, dof);
    // double chiSquareVal = 5.991; // for 95% confidence interval
    arma::vec halfAxisLengths = arma::sqrt(chiSquareVal*eigval);
    return arma::datum::pi * arma::prod(halfAxisLengths);
}

std::vector<double> confidenceEllipsoidZRangeForXY(arma::vec2 xy, arma::vec3 centre, arma::mat33 cov, double conf) {

    // arma::vec3 eigval;  // eigenvalues are stored in ascending order.
    // arma::mat33 eigvec;
    // arma::eig_sym(eigval, eigvec, cov, "std");
    //
    // double chiSquareVal = cChiSquare(conf, 3);
    //
    // // arma::vec axisLengths = 2*arma::sqrt(chiSquareVal*eigval);
    // // arma::vec3 size = {axisLengths(0), axisLengths(1), axisLengths(2)};
    // // arma::vec3 reciprocalHalfAxisLengths = 1.0 / (size*0.5);
    // // arma::vec3 ellipseEigvals = reciprocalHalfAxisLengths % reciprocalHalfAxisLengths;
    //
    // arma::vec3 ellipseEigvals = 1.0 / (chiSquareVal*eigval);
    //
    // arma::mat33 rot = eigvec;
    // arma::mat33 scale = arma::diagmat(ellipseEigvals);
    // arma::mat33 ellipseMat = rot * scale * rot.t();
    //
    // std::cout << "ellipseMat 1: " << ellipseMat << std::endl;
    // std::cout << "ellipseMat 2: " << (cov*chiSquareVal).i() << std::endl;
    //
    // return Ellipse::zRangeForXY(xy, centre, ellipseMat);

    double chiSquareVal = cChiSquare(conf, 3);
    return Ellipse::zRangeForXY(xy, centre, (cov*chiSquareVal).i());
}

double dnorm(arma::vec3 mean, arma::mat33 cov, arma::vec3 pos) {
    double det = arma::det(cov);

    double pi = arma::datum::pi;
    double normaliser = std::sqrt(det*pi*pi*pi*8);

    arma::vec3 diff = pos - mean;
    arma::mat exponent = -0.5*diff.t()*cov.i()*diff;

    return std::exp(exponent(0))/normaliser;
}

arma::mat randn(int n_elem, arma::vec3 mean, arma::mat33 cov) {
    arma::mat stdVals = arma::randn(3, n_elem);

    arma::mat33 chol = arma::chol(cov);

    return arma::repmat(mean, 1, n_elem) + chol.t()*stdVals;
}

arma::vec randn(arma::vec2 mean, arma::mat22 cov) {
    arma::vec2 stdVals = arma::randn(2);
    arma::mat22 chol = arma::chol(cov);
    return mean + chol.t()*stdVals;

    // arma::mat stdVals = arma::randn(2, n_elem);
    //
    // arma::mat22 chol = arma::chol(arma::mat(cov));
    //
    // return arma::repmat(mean, 1, n_elem) + chol.t()*stdVals;
}


arma::mat33 transformToLocalDistribution(Transform2D trans, arma::mat33 transCov, Transform2D pos) {
    Transform2D diff = pos - trans;

    double sinTheta = std::sin(trans.angle());
    double cosTheta = std::cos(trans.angle());

    arma::mat33 J; // Jacobian of trans.worldToLocal(pos) with respect to trans.
    J(0,0) = -cosTheta;
    J(0,1) = -sinTheta;
    J(0,2) = -diff.x()*sinTheta + diff.y()*cosTheta;
    J(1,0) =  sinTheta;
    J(1,1) = -cosTheta;
    J(1,2) = -diff.x()*cosTheta - diff.y()*sinTheta;
    J(2,0) = 0;
    J(2,1) = 0;
    J(2,2) = -1;

    return J*transCov*J.t();
}

}
}
}
