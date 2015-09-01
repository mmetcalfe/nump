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
        Transform2D trans = ellipse.getTransform();
        Transform2D pos = trans.worldToLocal({circle.centre(0), circle.centre(1), 0});

        arma::vec2 size = ellipse.getSize();

        double hw = 0.5 * size(0); // half-width
        double hh = 0.5 * size(1); // half-height

        // Circle transformed to positive quadrant of local ellipse coords:
        double r = circle.radius;
        double x = std::abs(pos(0));
        double y = std::abs(pos(1));
        Circle localCircle = {{x, y}, r};

        // Throw out far off circles:
        if (x > hw + r || y > hh + r) {
            return false;
        }


        // Throw out circles with centres inside the ellipse:
        if ((x*x)/(hw*hw) + (y*y)/(hh*hh) <= 1) {
            return true;
        }

        // Intersect circle with ellipse:
        int numSamples = 100;
        for (int i = 0; i <= numSamples; i++) {
            double qx = hw * (i / double(numSamples));
            double qy = std::sqrt((hh*hh)*(1.0 - (qx*qx)/(hw*hw)));

            if (localCircle.contains({qx, qy})) {
                return true;
            }
        }

        return false;
    }

}
}
}
}
