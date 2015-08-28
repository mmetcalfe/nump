//
// Created by Mitchell Metcalfe on 8/08/15.
//

#ifndef NUMP_SEARCHTREEDRAWING_H
#define NUMP_SEARCHTREEDRAWING_H

#include <armadillo>
#include "nump.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include <cairo/cairo.h>

namespace shared {
namespace utility {
namespace drawing {

    using nump::Transform2D;
    using nump::math::RotatedRectangle;
    using ::utility::math::geometry::Ellipse;

    arma::vec2 deviceToUser(cairo_t *cr, arma::vec2 pt);
    arma::vec2 deviceToUserDistance(cairo_t *cr, arma::vec2 vec);
    void cairoMoveTo(cairo_t *cr, arma::vec2 pos);
    void cairoLineTo(cairo_t *cr, arma::vec2 pos);

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

    void drawRobot(cairo_t *cr, Transform2D trans, double size);

    void drawRRBT(cairo_t *cr, const nump::RRBT& rrbt);

    void drawEllipse(cairo_t *cr, const Ellipse& ellipse);

    void drawRotatedRectangle(cairo_t *cr, const RotatedRectangle& rect);

    void drawErrorEllipse(cairo_t *cr, arma::vec2 mean, arma::mat22 cov, double confidence);

}
}
}

#endif //NUMP_SEARCHTREEDRAWING_H
