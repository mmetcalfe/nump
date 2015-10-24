//
// Created by Mitchell Metcalfe on 16/08/15.
//

//
// Created by Mitchell Metcalfe on 9/08/15.
//

#include <armadillo>
#include "math/geometry.h"
#include "shared/utility/math/angle.h"

namespace nump {
namespace math {

    using utility::math::angle::normalizeAngle;

    bool Circle::contains(arma::vec2 pt) const {
        return arma::norm(centre - pt) < radius;
    }

    Rotation2D::Rotation2D() {
        eye(); // identity matrix by default
    }

    Rotation2D Rotation2D::rotate(double radians) const {
        return *this * createRotation(radians);
    }

    Rotation2D Rotation2D::i() const {
        // http://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
        // The inverse of a rotation matrix is its transpose, which is also a rotation matrix.
        return t();
    }

    Rotation2D Rotation2D::createRotation(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation2D rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << c << -s << arma::endr
                 << s <<  c;
        return rotation;
    }

    Transform2D::Transform2D() {
        zeros();
    }

    Transform2D::Transform2D(const arma::vec2 xy_, double angle_) {
        xy() = xy_;
        angle() = angle_;
    }
    Transform2D Transform2D::localToWorld(const Transform2D& reference) const {
        double cosAngle = std::cos(angle());
        double sinAngle = std::sin(angle());
        // translates to this + rotZ(this.angle) * reference
        return {
                x() + cosAngle * reference.x() - sinAngle * reference.y(),
                y() + sinAngle * reference.x() + cosAngle * reference.y(),
                normalizeAngle(angle() + reference.angle()) // TODO: Add normaliseAngle fix to NUbots codebase!
        };
    }
    Transform2D Transform2D::worldToLocal(const Transform2D& reference) const {
        double cosAngle = std::cos(angle());
        double sinAngle = std::sin(angle());
        Transform2D diff = reference - *this;
        // translates to rotZ(this.angle) * (reference - this)
        return {
                cosAngle * diff.x() + sinAngle * diff.y(),
                -sinAngle * diff.x() + cosAngle * diff.y(),
                normalizeAngle(diff.angle())
        };
    }

    double Transform2D::x() const { return at(0); }
    double& Transform2D::x() { return at(0); }
    double Transform2D::y() const { return at(1); }
    double& Transform2D::y() { return at(1); }
    double Transform2D::angle() const { return at(2); }
    double& Transform2D::angle() { return at(2); }
    const arma::subview_col<double> Transform2D::xy() const { return rows(0,1); }
    arma::subview_col<double> Transform2D::xy() { return rows(0,1); }

    RotatedRectangle::RotatedRectangle(const Transform2D& transform_, const arma::vec2& size_)
            : transform(transform_), size(size_) { }

    Transform2D RotatedRectangle::getTransform() const {
        return transform;
    }

    arma::vec2 RotatedRectangle::getPosition()   const {
        return arma::vec(transform.xy());
    }

    double RotatedRectangle::getRotation()       const {
        return transform.angle();
    }

    arma::vec2 RotatedRectangle::getSize()       const {
        return size;
    }

    bool RotatedRectangle::contains(arma::vec2 pt) const {
        Transform2D local = transform.worldToLocal({pt, 0});
        arma::vec2 absLocal = arma::abs(local.xy());

        if (absLocal(0) > size(0)*0.5 || absLocal(1) > size(1)*0.5) {
            return false;
        }

        return true;
    }


    namespace intersection {
        bool test(const Circle& circle, const RotatedRectangle& rect) {
            /*
                Let E be the centre of the rectangle (i.e. the origin of its local coordinate frame).
                Define the regions A-I as below, where the central region E has the dimensions of the rectangle.

                A │ B │ C
                ──┼───┼──
                D │ E │ F
                ──┼───┼──
                G │ H │ I

                Check distance to side in regions: B, D, F, and H.
                Check distance to corner in regions: A, C, G, and I.
                Region E is always an intersection.
                Note: This diagram and the circle are symmetric, so we can use absolute values to simplify the comparisons.
                i.e.
                  hw
                B │ C
                ──┼── hh
                E │ F
            */

            Transform2D trans = rect.getTransform();
            Transform2D pos = trans.worldToLocal({circle.centre(0), circle.centre(1), 0});

            double hw = 0.5 * rect.getSize()(0);
            double hh = 0.5 * rect.getSize()(1);
            double r = circle.radius;

            double x = std::abs(pos(0));
            double y = std::abs(pos(1));

            if (x < hw && y < hh) { // E
                return true;
            }

            if (x < hw && y > hh) { // B
                return y < hh + r;
            }

            if (x > hw && y < hh) { // F
                return x < hw + r;
            }

            // if (x > hw && y > hh) { // C
            arma::vec2 cornerDiff = { hw - x, hh - y };
            return arma::norm(cornerDiff) < r;
            // }
        }
    }

}
}
