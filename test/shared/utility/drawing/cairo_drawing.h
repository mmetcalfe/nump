//
// Created by Mitchell Metcalfe on 8/08/15.
//

#ifndef NUMP_CAIRO_DRAWING_H
#define NUMP_CAIRO_DRAWING_H

#include <armadillo>
#include "nump.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include <cairo/cairo.h>

namespace utility {
namespace drawing {

    using nump::Transform2D;
    using nump::math::RotatedRectangle;
    using ::utility::math::geometry::Ellipse;

    void drawKickBoxes(cairo_t *cr, Transform2D robot, const nump::BipedRobotModel::KickBox& kbConfig, double ballRadius);

    void cairoTransformToLocal(cairo_t *cr, Transform2D trans);

    arma::vec2 deviceToUser(cairo_t *cr, arma::vec2 pt);
    arma::vec2 deviceToUserDistance(cairo_t *cr, arma::vec2 vec);
    void cairoMoveTo(cairo_t *cr, arma::vec2 pos);
    void cairoLineTo(cairo_t *cr, arma::vec2 pos);
    void drawLine(cairo_t *cr, arma::vec2 from, arma::vec2 to);

    void cairoSetSourceRGB(cairo_t *cr, arma::vec3 rgb);
    void cairoSetSourceRGBAlpha(cairo_t *cr, arma::vec3 rgb, double alpha);


    void showText(cairo_t *cr, arma::vec2 pos, double fontSize, const std::string &str);
    template<typename T, typename... Args>
    inline void showText(cairo_t *cr, arma::vec2 pos, double fontSize, std::string s, T t, Args... args) {
        std::stringstream ss;
        ss << s << t;

        showText(cr, pos, fontSize, ss.str(), args...);
    }
    template<typename T, typename... Args>
    inline void showText(cairo_t *cr, arma::vec2 pos, double fontSize, T t, Args... args) {
        std::stringstream ss;
        ss << t;

        showText(cr, pos, fontSize, ss.str(), args...);
    }

    void fillCircle(cairo_t *cr, arma::vec2 c, float r, arma::vec3 col = {0, 0, 0}, double alpha = 1);

    void fillCircle(cairo_t *cr, nump::math::Circle circle, arma::vec3 col = {0, 0, 0}, double alpha = 1);

    void drawCircle(cairo_t *cr, nump::math::Circle circle);

    void drawTree(cairo_t *cr, const nump::SearchTree::TreeT& tree, double r);

    void drawSearchTree(cairo_t *cr, const nump::SearchTree& searchTree);

    void drawRobot(cairo_t *cr, Transform2D trans, double size, bool noFill = false);
    void drawRobot(cairo_t *cr, arma::vec2 pos, double size, bool noFill = false);
    void drawRobot(cairo_t *cr, Transform2D trans, double size, arma::vec2 footprintSize);

    void drawPath(cairo_t *cr, nump::Path<nump::BipedRobotModel::State> nominalPath, double timeStep, double size);


    void drawRRBT(cairo_t *cr, const nump::RRBT& rrbt);

    bool drawEllipse(cairo_t *cr, const Ellipse& ellipse);

    void drawRotatedRectangle(cairo_t *cr, const RotatedRectangle& rect);

    bool drawErrorEllipse(cairo_t *cr, const arma::vec2& mean, const arma::mat22& cov, double confidence);
    bool drawErrorEllipse(cairo_t *cr, const nump::BipedRobotModel::State& mean, const nump::BipedRobotModel::MotionCov& cov, double confidence);

    void drawRectangleRotationRange(cairo_t *cr, arma::vec2 position, arma::vec2 footprint, arma::vec2 angleRange);

}
}

#endif //NUMP_CAIRO_DRAWING_H
