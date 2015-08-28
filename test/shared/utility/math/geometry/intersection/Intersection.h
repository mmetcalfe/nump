//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#ifndef SHARED_UTILITY_MATH_GEOMETRY_INTERSECTION_H
#define SHARED_UTILITY_MATH_GEOMETRY_INTERSECTION_H

#include "Intersection.h"
#include "math/geometry.h"
#include "shared/utility/math/geometry/Ellipse.h"

namespace utility {
namespace math {
namespace geometry {
namespace intersection {

    using ::nump::math::Circle;

    bool test(const Circle& circle, const Ellipse& ellipse);

}
}
}
}


#endif //NUMP_TESTCIRCLEELLIPSE_H
