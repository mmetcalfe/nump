//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#include "Ellipse.h"

namespace utility {
namespace math {
namespace geometry {
    std::vector<double> Ellipse::yRangeForX(double sx) {
        // Transform2D transform;
        // arma::vec2 size;

        double x = sx - transform.x();

        arma::vec2 reciprocalHalfAxisLengths = 1.0 / (size*0.5);
        arma::vec2 eigvals = reciprocalHalfAxisLengths % reciprocalHalfAxisLengths;

        arma::mat22 rot = transform.rotation();
        arma::mat22 scale = arma::diagmat(eigvals);
        arma::mat22 ellipseMat = rot * scale * rot.t();

        std::cout << ellipseMat << std::endl;

        double a = ellipseMat(0, 0);
        double b = ellipseMat(0, 1);
        double c = ellipseMat(1, 0);
        double d = ellipseMat(1, 1);

        double discriminant = (c+b)*(c+b)*x*x - 4*d*(a*x*x-1);

        if (discriminant < 0) {
            return {};
        }

        double sqrtDisc = std::sqrt(discriminant);

        double yMax = (-(c+b)*x + sqrtDisc) / (2*d) + transform.y();
        double yMin = (-(c+b)*x - sqrtDisc) / (2*d) + transform.y();

        return {yMin, yMax};
    }
}
}
}
