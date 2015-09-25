/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#include "Intersection.h"
#include "math/geometry.h"
#include "shared/utility/math/geometry/Ellipse.h"

namespace utility {
namespace math {
namespace geometry {
namespace intersection {

    using ::nump::math::Circle;
    using nump::math::Transform2D;


    bool test(const Circle& circle, const Ellipse& ellipse) {
        arma::vec2 size = ellipse.getSize();

        double hw = 0.5 * size(0); // half-width
        double hh = 0.5 * size(1); // half-height

        double r = circle.radius;

        // Throwout circles that intersect the circumcircle of the rotated rectangle:
        double centreDist = std::sqrt(hw*hw + hh*hh) + r;
        arma::vec2 diff = circle.centre - ellipse.transform.xy();
        if (arma::dot(diff, diff) > centreDist*centreDist) {
            return false;
        }

        Transform2D trans = ellipse.getTransform();
        Transform2D pos = trans.worldToLocal({circle.centre(0), circle.centre(1), 0});

        // Circle transformed to positive quadrant of local ellipse coords:
        double x = std::abs(pos(0));
        double y = std::abs(pos(1));

        // Throw out circles that don't intersect the rotated-rectangle:
        if (x > hw + r || y > hh + r) {
            return false;
        }

        // Throw out circles that intersect an internal rotated-rectangle:
        double ihw = hw / arma::datum::sqrt2;
        double ihh = hh / arma::datum::sqrt2;
        if (!(x > ihw + r || y > ihh + r)) {
            return true;
        }

        // Throw out circles with centres inside the ellipse:
        if ((x*x)/(hw*hw) + (y*y)/(hh*hh) <= 1) {
            return true;
        }

        // Intersect circle with ellipse:
        Circle localCircle = {{x, y}, r};
        int numSamples = 5;
        for (int i = 0; i <= numSamples; i++) {
            double qhx = 0.5 * i * (hw / double(numSamples));
            double qhy = std::sqrt((hh*hh)*(1.0 - (qhx*qhx)/(hw*hw)));
            if (localCircle.contains({qhx, qhy})) {
                return true;
            }

            double qvy = 0.5 * i * (hh / double(numSamples));
            double qvx = std::sqrt((hw*hw)*(1.0 - (qvy*qvy)/(hh*hh)));
            if (localCircle.contains({qvx, qvy})) {
                return true;
            }
        }

        return false;
    }

}
}
}
}
