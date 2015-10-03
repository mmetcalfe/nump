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

    std::vector<double> Ellipse::zRangeForXY(arma::vec2 xy, arma::vec3 centre, arma::mat33 A) {
        arma::vec X = xy - centre.head(2);

        arma::mat22 L = A.submat(0, 0, 1, 1);
        arma::vec2 R = A.submat(0, 2, 1, 2);
        arma::rowvec2 B = A.submat(2, 0, 2, 1);
//        arma::mat R = arma::mat(A.submat(0, 2, 1, 2));
//        arma::mat B = arma::mat(A.submat(2, 0, 2, 1));

        double aq = A(2, 2);
        double bq = arma::mat(B*X + X.t()*R)(0,0);
        double cq = arma::mat(X.t()*L*X - 1.0)(0,0);

        double discriminant = bq*bq - 4*aq*cq;

        if (discriminant < 0) {
            return {};
        }

        double sqrtDisc = std::sqrt(discriminant);

        double zMax = (-bq + sqrtDisc) / (2*aq) + centre(2);
        double zMin = (-bq - sqrtDisc) / (2*aq) + centre(2);

        return {zMin, zMax};
    }

}
}
}
