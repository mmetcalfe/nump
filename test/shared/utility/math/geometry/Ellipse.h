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

        // TODO: Consider making a NormalDistribution class and moving this there.
        static Ellipse forConfidenceRegion(arma::vec2 mean, arma::mat22 cov);
    };

}
}
}


#endif //SHARED_UTILITY_MATH_GEOMETRY_ELLIPSE_H
