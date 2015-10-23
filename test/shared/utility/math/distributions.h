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

    arma::vec2 confidenceRegion(double mean, double var, double conf, int dof = 1);

    Ellipse confidenceRegion(arma::vec2 mean, arma::mat22 cov, double conf, int dof = 2);

    double confidenceRegionArea(arma::mat22 cov, double conf, int dof = 2);

    std::vector<double> confidenceEllipsoidZRangeForXY(arma::vec2 xy, arma::vec3 centre, arma::mat33 cov, double conf);

    double dnorm(arma::mat22 cov, arma::vec2 pos);
}
}
}


#endif //SHARED_UTILITY_MATH_DISTRIBUTIONS_H
