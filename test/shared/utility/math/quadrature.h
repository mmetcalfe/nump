//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#ifndef SHARED_UTILITY_MATH_QUADRATURE_H
#define SHARED_UTILITY_MATH_QUADRATURE_H

#include "math/geometry.h"
#include "shared/utility/math/geometry/Ellipse.h"

namespace utility {
namespace math {
namespace quadrature {
    using nump::math::RotatedRectangle;

    // extern std::vector<std::vector<double>> quadratureRoots;
    // extern std::vector<std::vector<double>> quadratureCoefficients;

    // Integrates func over the rect, with a z-range of [-zSpan/2, zSpan/2].
    double integrateGaussQuad(std::function<double(arma::vec3)> func, RotatedRectangle rect, double zSpan, arma::ivec3 order);

}
}
}


#endif //SHARED_UTILITY_MATH_QUADRATURE_H
