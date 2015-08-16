//
// Created by Mitchell Metcalfe on 8/08/15.
//

#ifndef NUMP_SEARCHTREEDRAWING_H
#define NUMP_SEARCHTREEDRAWING_H

#include <armadillo>
#include "nump.h"
#include <cairo/cairo.h>

namespace shared {
namespace utility {
namespace drawing {

    arma::vec2 deviceToUser(cairo_t *cr, arma::vec2 pt);
    arma::vec2 deviceToUserDistance(cairo_t *cr, arma::vec2 vec);
    void cairoMoveTo(cairo_t *cr, arma::vec2 pos);
    void cairoLineTo(cairo_t *cr, arma::vec2 pos);

    void drawString(cairo_t *cr, arma::vec2 pos, double fontSize, const std::string& str);
    template<typename T, typename... Args>
    inline void drawString(cairo_t *cr, arma::vec2 pos, double fontSize, std::string s, T t, Args... args) {
        std::stringstream ss;
        ss << s << t;

        drawString(cr, pos, fontSize, ss.str(), args...);
    }
    template<typename T, typename... Args>
    inline void drawString(cairo_t *cr, arma::vec2 pos, double fontSize, T t, Args... args) {
        std::stringstream ss;
        ss << t;

        drawString(cr, pos, fontSize, ss.str(), args...);
    }

    void drawCircle(cairo_t *cr, arma::vec2 c, float r, arma::vec3 col = {0, 0, 0}, double alpha = 1);

    void drawCircle(cairo_t *cr, nump::math::Circle circle, arma::vec3 col = {0, 0, 0}, double alpha = 1);

    void drawTree(cairo_t *cr, const nump::SearchTree::TreeT& tree, double r);

    void drawSearchTree(cairo_t *cr, const nump::SearchTree& searchTree);

}
}
}

#endif //NUMP_SEARCHTREEDRAWING_H
