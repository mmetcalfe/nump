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

    bool testConfidenceRegion(const Circle& circle, const arma::mat22& posCov, double conf, const RotatedRectangle& rect) {

        Ellipse confEllipse = utility::math::distributions::confidenceRegion(circle.centre, posCov, conf, 2);

        // Throw out far off rectangles:
        double maxEllipseRad = arma::max(confEllipse.size / 2);
        if (!test({circle.centre, circle.radius + maxEllipseRad}, rect)) {
            return false;
        }

        Ellipse internalEllipse = {confEllipse.transform, confEllipse.size + 2*arma::vec2({circle.radius, circle.radius})};

        double ihw = 0.5 * internalEllipse.size(0); // half-width
        double ihh = 0.5 * internalEllipse.size(1); // half-height

        {
            // Transform to rectangle to positive quadrant of local confEllipse coords:
            Transform2D localTrans = confEllipse.transform.worldToLocal(rect.transform);
            if (localTrans.y() < 0) {
                localTrans.angle() = -localTrans.angle();
            }
            if (localTrans.x() < 0) {
                localTrans.angle() = arma::datum::pi - localTrans.angle();
            }
            localTrans.x() = std::abs(localTrans.x());
            localTrans.y() = std::abs(localTrans.y());

            double x = localTrans.x();
            double y = localTrans.y();
            double r = arma::norm(rect.size / 2);

            // Throw out rectangles that don't intersect the rotated-rectangle:
            if (x - r > ihw || y - r > ihh) {
                return false;
            }

            // Throw out rectangles with centres inside the ellipse:
            if ((x*x)/(ihw*ihw) + (y*y)/(ihh*ihh) <= 1) {
                return true;
            }

            // Intersect rectangle with circles around ellipse:
            double chw = 0.5 * confEllipse.size(0);
            double chh = 0.5 * confEllipse.size(1);
            RotatedRectangle localRect = {localTrans, rect.size};
            int numSamples = 5;
            for (int i = 0; i <= numSamples; i++) {
                double t = i / double(numSamples);
                double l = t / arma::datum::sqrt2;

                double qhx = l * chw;
                double qhy = std::sqrt((chh*chh)*(1.0 - (qhx*qhx)/(chw*chw)));
                if (test({{qhx, qhy}, circle.radius}, localRect)) {
                    return true;
                }

                double qvy = l * chh;
                double qvx = std::sqrt((chw*chw)*(1.0 - (qvy*qvy)/(chh*chh)));
                if (test({{qvx, qvy}, circle.radius}, localRect)) {
                    return true;
                }
            }
        }

        return false;
    }

}
}
}
}
