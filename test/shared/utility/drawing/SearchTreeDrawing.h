//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include <armadillo>
#include "nump.h"
#include <cairo/cairo.h>

#ifndef NUMP_SEARCHTREEDRAWING_H
#define NUMP_SEARCHTREEDRAWING_H

namespace shared {
namespace utility {
namespace drawing {

    arma::vec2 deviceToUser(cairo_t *cr, arma::vec2 pt) {
        double x = pt(0), y = pt(1);
        cairo_device_to_user(cr, &x, &y);
        return {x, y};
    }

    arma::vec2 deviceToUserDistance(cairo_t *cr, arma::vec2 vec) {
        double x = vec(0), y = vec(1);
        cairo_device_to_user_distance(cr, &x, &y);
        return {x, y};
    }

    void cairoMoveTo(cairo_t *cr, arma::vec2 pos) {
        cairo_move_to(cr, pos(0), pos(1));
    }

    void cairoLineTo(cairo_t *cr, arma::vec2 pos) {
        cairo_line_to(cr, pos(0), pos(1));
    }

    void drawCircle(cairo_t *cr, arma::vec2 c, float r, arma::vec3 col = {0, 0, 0}) {
        cairo_arc(cr, c(0), c(1), r, -M_PI, M_PI);
        cairo_set_source_rgba(cr, col(0), col(1), col(2), 1);
        cairo_fill(cr);
    }

    void drawTree(cairo_t *cr, const nump::Tree<arma::vec2>& tree, double r) {
        // Draw edges:
        cairo_set_line_width(cr, r * 0.5);
        for (auto& node : tree.nodes) {
            if (node->parent == nullptr) {
                continue;
            }

            arma::vec2 pos = node->value;
            arma::vec2 parent = tree.parent(node)->value;

            cairoMoveTo(cr, pos);
            cairoLineTo(cr, parent);

            double hr = tree.depth(node) / (double)tree.height();
            cairo_set_source_rgba(cr, hr, hr, 1 - hr, 1);
            cairo_stroke(cr);
        }

        // Draw vertices:
        for (auto& node : tree.nodes) {
            arma::vec2 pos = node->value;

            drawCircle(cr, pos, r);
        }
    }

    void drawSearchTree(cairo_t *cr, const nump::SearchTree& tree) {
        cairo_save(cr);

        double r = deviceToUserDistance(cr, {2, 0})(0);
        drawCircle(cr, tree.initialState(), r, {1, 0.5, 0.5});
        drawCircle(cr, tree.goalState(), r, {0.5, 1, 0.5});

        drawTree(cr, tree.tree, r);

        cairo_restore(cr);
    }

}
}
}

#endif //NUMP_SEARCHTREEDRAWING_H
