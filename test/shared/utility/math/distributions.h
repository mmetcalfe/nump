//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#ifndef SHARED_UTILITY_MATH_DISTRIBUTIONS_H
#define SHARED_UTILITY_MATH_DISTRIBUTIONS_H

#include "math/geometry.h"
#include "shared/utility/math/geometry/Ellipse.h"

namespace utility {
namespace math {
namespace distributions {

    using geometry::Ellipse;
    using nump::math::Transform2D;

    double cChiSquare(double percentile, int dof);

    arma::vec2 confidenceRegion(double mean, double var, double conf, int dof = 1);

    Ellipse confidenceRegion(arma::vec2 mean, arma::mat22 cov, double conf, int dof = 2);

    double confidenceRegionArea(arma::mat22 cov, double conf, int dof = 2);

    std::vector<double> confidenceEllipsoidZRangeForXY(arma::vec2 xy, arma::vec3 centre, arma::mat33 cov, double conf);

    double dnorm(arma::vec3 mean, arma::mat33 cov, arma::vec3 pos);
    arma::mat randn(int n_elem, arma::vec3 mean, arma::mat33 cov);
    arma::vec randn(arma::vec2 mean, arma::mat22 cov);

    arma::mat33 transformToLocalDistribution(Transform2D trans, arma::mat33 transCov, Transform2D pos);
}
}
}


#endif //SHARED_UTILITY_MATH_DISTRIBUTIONS_H
