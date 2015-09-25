//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#include "Ellipse.h"


namespace utility {
namespace math {
namespace geometry {

using nump::math::Transform2D;

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
    eigval = eigvals_2d(cov);

    double eps = 1e-6;
    if (cov(1,0) > eps) {
        eigvec.col(0) = arma::vec2({eigval(0)-cov(1, 1), cov(1, 0)});
        eigvec.col(1) = arma::vec2({eigval(1)-cov(1, 1), cov(1, 0)});
    } else if (cov(0,1) > eps) {
        eigvec.col(0) = arma::vec2({cov(0, 1), eigval(0)-cov(0, 0)});
        eigvec.col(1) = arma::vec2({cov(0, 1), eigval(1)-cov(0, 0)});
    } else {
        eigvec.col(0) = arma::vec2({1, 0});
        eigvec.col(1) = arma::vec2({0, 1});
    }
}

Ellipse Ellipse::forConfidenceRegion(arma::vec2 mean, arma::mat22 cov) {
    arma::vec2 eigval;  // eigenvalues are stored in ascending order.
    arma::mat22 eigvec;
    // arma::eig_sym(eigval, eigvec, cov, "std");
    eig_sym_2d(eigval, eigvec, cov);
    arma::vec2 primaryAxis = arma::vec(eigvec.col(1));
    double angle = std::atan2(primaryAxis(1), primaryAxis(0));

    double chiSquareVal = 5.991; // for 95% confidence interval
    arma::vec axisLengths = 2*arma::sqrt(chiSquareVal*eigval);
    arma::vec2 size = {axisLengths(1), axisLengths(0)};

    Transform2D trans = {mean, angle};
    return {trans, size};
}

double Ellipse::confidenceRegionArea(arma::mat22 cov) {
    arma::vec2 eigval = eigvals_2d(cov);
    double chiSquareVal = 5.991; // for 95% confidence interval
    arma::vec halfAxisLengths = arma::sqrt(chiSquareVal*eigval);
    return arma::datum::pi * arma::prod(halfAxisLengths);
}


}
}
}
