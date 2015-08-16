//
// Created by Mitchell Metcalfe on 9/08/15.
//

#ifndef NUMP_GEOMETRY_H
#define NUMP_GEOMETRY_H

#include <armadillo>

namespace nump {
namespace math {

    class Transform2D : public arma::vec3 {
        using arma::vec3::vec3; // inherit constructors

    public:
        Transform2D();

        Transform2D(const arma::vec2 xy_, double angle_);

//        static Transform2D lookAt(const arma::vec2 from, arma::vec2 to) {
//            arma::vec2 vecHeading = to - from;
//            double angle = vectorToBearing(vecHeading);
//            return {from, angle};
//        }
        Transform2D localToWorld(const Transform2D& reference) const;
        Transform2D worldToLocal(const Transform2D& reference) const;

        inline double x() const;
        inline double& x();
        inline double y() const;
        inline double& y();
        inline double angle() const;
        inline double& angle();
        inline const arma::subview_col<double> xy() const;
        inline arma::subview_col<double> xy();
    };

    class Circle {
    public:
        arma::vec2 centre;
        double radius;

        bool contains(arma::vec2 pt) const;
    };

    class RotatedRectangle {
    private:
        Transform2D transform;
        arma::vec2 size;

    public:
        RotatedRectangle(const Transform2D& transform_, const arma::vec2& size_);
        Transform2D getTransform() const;
        arma::vec2 getPosition()   const;
        double getRotation()       const;
        arma::vec2 getSize()       const;
    };

    namespace intersection {
        bool test(const Circle& circle, const RotatedRectangle& rect);
    }

}
}

#endif //NUMP_GEOMETRY_H
