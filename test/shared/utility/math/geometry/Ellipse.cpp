//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#include "Ellipse.h"


namespace utility {
namespace math {
namespace geometry {

using nump::math::Transform2D;

Ellipse Ellipse::forConfidenceRegion(arma::vec2 mean, arma::mat22 cov) {
    arma::vec eigval;  // eigenvalues are stored in ascending order.
    arma::mat eigvec;
    arma::eig_sym(eigval, eigvec, cov);
    arma::vec2 primaryAxis = arma::vec(eigvec.col(1));
    double angle = std::atan2(primaryAxis(1), primaryAxis(0));

    Transform2D trans = {mean, angle};

    double chiSquareVal = 5.991; // for 95% confidence interval

    arma::vec axisLengths = 2*arma::sqrt(chiSquareVal*eigval);

    arma::vec2 size = {axisLengths(1), axisLengths(0)};

    return {trans, size};
}


}
}
}
