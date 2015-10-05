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

        Ellipse ellipse = utility::math::distributions::confidenceRegion(rect.transform.head(2), posHeadingCov.submat(0,0,1,1), 0.95, 3);

        // // Throw out far off circles:
        double maxRectRad = arma::norm(rect.size / 2);
        if (!test({circle.centre, circle.radius + maxRectRad}, ellipse)) {
            return false;
        }

        // Throw out very close circles:
        double minRectRad = std::min(rect.size(0), rect.size(1)) / 2;
        if (test({circle.centre, circle.radius + minRectRad}, ellipse)) {
            return true;
        }

        arma::vec2 size = ellipse.getSize();
        double hw = 0.5 * size(0); // half-width
        double hh = 0.5 * size(1); // half-height

        double r = circle.radius;

        Transform2D trans = ellipse.getTransform();
        Transform2D pos = trans.worldToLocal({circle.centre, 0});

        // Circle transformed to positive quadrant of local ellipse coords:
        double x = std::abs(pos(0));
        double y = std::abs(pos(1));
        Circle localCircle = {{x, y}, r};

        // Perform rejection sampling:
        // Circle globalMaxCircle = {trans.localToWorld({x, y, 0}).xy(), r + maxRectRad};
        int numSamples = 30;
        for (double qx = -hw; qx < hw; qx += 2*hw/numSamples) {
            for (double qy = -hh; qy < hh; qy += 2*hh/numSamples) {
                Transform2D gq = trans.localToWorld({qx, qy, 0});

                // if (!globalMaxCircle.contains(gq.head(2))) {
                //     continue;
                // }

                auto tRange = utility::math::distributions::confidenceEllipsoidZRangeForXY({gq.x(), gq.y()}, rect.transform, posHeadingCov, 0.95);
                if (tRange.size() != 2) {
                    continue;
                }

                double tspan = tRange[1] - tRange[0];
                arma::vec2 trangevec = {tRange[0], tRange[1]};
                for (double gqt = arma::min(trangevec); gqt < arma::max(trangevec); gqt += tspan/numSamples) {
                    if (test(circle, RotatedRectangle {{gq.x(), gq.y(), gqt}, rect.size})) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

}
}
}
}
