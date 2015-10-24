//
// Created by Mitchell Metcalfe on 28/08/2015.
//

#include "shared/utility/math/quadrature.h"

namespace utility {
namespace math {
namespace quadrature {

using nump::math::Transform2D;

std::vector<std::vector<double>> quadratureRoots = {
    {}, {}, // Make indexing easy.
    {0.5773502692, -0.5773502692},
    {0.7745966692, 0, -0.7745966692},
    {0.8611363116, 0.3399810436, -0.3399810436, -0.8611363116}
};

std::vector<std::vector<double>> quadratureCoefficients = {
    {}, {}, // Make indexing easy.
    {1, 1},
    {5.0/9.0, 8.0/9.0, 5.0/9.0},
    {0.3478548451, 0.6521451549, 0.6521451549, 0.3478548451}
};

double integrateGaussQuad(std::function<double(arma::vec3)> func, RotatedRectangle rect, double zSpan, arma::ivec3 order) {

    double sum = 0;

    for (int i = 0; i < order(0); i++) {
        double ci = quadratureCoefficients[order(0)][i];
        double u = quadratureRoots[order(0)][i];
        double lx = u*rect.size(0)*0.5;
        for (int j = 0; j < order(1); j++) {
            double cj = quadratureCoefficients[order(1)][j];
            double v = quadratureRoots[order(1)][j];
            double ly = v*rect.size(1)*0.5;
            auto worldXY = rect.transform.localToWorld({lx, ly, 0});
            double x = worldXY.x();
            double y = worldXY.y();
            for (int k = 0; k < order(2); k++) {
                double ck = quadratureCoefficients[order(2)][k];
                double w = quadratureRoots[order(2)][k];
                double z = w*zSpan*0.5;

                double f = func({x, y, z});

                sum += ci*ck*cj*f;
            }
        }
    }

    double rangeFactor = (rect.size(0)/2) * (rect.size(1)/2) * (zSpan/2);

    return sum*rangeFactor;
}


}
}
}
