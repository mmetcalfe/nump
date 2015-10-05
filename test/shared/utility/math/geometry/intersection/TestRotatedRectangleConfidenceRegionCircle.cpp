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
#include "shared/utility/math/distributions.h"

namespace utility {
namespace math {
namespace geometry {
namespace intersection {

    using ::nump::math::Circle;
    using nump::math::Transform2D;

    bool testConfidenceRegion(const RotatedRectangle& rect, const arma::mat33& posHeadingCov, double conf, Circle& circle) {

        Ellipse confEllipseXY = utility::math::distributions::confidenceRegion(rect.transform.head(2), posHeadingCov.submat(0,0,1,1), 0.95, 3);

        // Throw out far off circles:
        double maxRectRad = arma::norm(rect.size / 2);
        if (!test({circle.centre, circle.radius + maxRectRad}, confEllipseXY)) {
            return false;
        }

        // Throw out very close circles:
        double minRectRad = std::min(rect.size(0), rect.size(1)) / 2;
        if (test({circle.centre, circle.radius + minRectRad}, confEllipseXY)) {
            return false;
        }


        return true;
    }

}
}
}
}
