//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#ifndef SHARED_UTILITY_MATH_GEOMETRY_ELLIPSE_H
#define SHARED_UTILITY_MATH_GEOMETRY_ELLIPSE_H

#include "math/geometry.h"

namespace utility {
namespace math {
namespace geometry {

    using nump::math::RotatedRectangle;

    class Ellipse : public RotatedRectangle {
        using RotatedRectangle::RotatedRectangle;

    public:
        std::vector<double> yRangeForX(double x);

        static std::vector<double> zRangeForXY(arma::vec2 xy, arma::vec3 centre, arma::mat33 A);
    };

}
}
}


#endif //SHARED_UTILITY_MATH_GEOMETRY_ELLIPSE_H
