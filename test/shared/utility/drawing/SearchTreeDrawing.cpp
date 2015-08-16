//
// Created by Mitchell Metcalfe on 16/08/15.
//

//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include "SearchTreeDrawing.h"

#include <armadillo>
#include "nump.h"
#include <cairo/cairo.h>

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

    void drawString(cairo_t *cr, arma::vec2 pos, double fontSize, const std::string& str) {
        cairo_save(cr);

        cairo_text_extents_t te;
        cairo_set_source_rgb(cr, 0.0, 0.0, 0.0);
        cairo_select_font_face(cr, "Georgia", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
        cairo_set_font_size(cr, fontSize);
        cairo_text_extents(cr, str.c_str(), &te);
        cairo_move_to(cr, pos(0) - te.width / 2 - te.x_bearing,
                       pos(1) - te.height / 2 - te.y_bearing);
        cairo_show_text(cr, str.c_str());
        cairo_stroke(cr); // TODO: Find out why cairo_show_text leaves behind a path.

        cairo_restore(cr);
    }

    void drawCircle(cairo_t *cr, arma::vec2 c, float r, arma::vec3 col, double alpha) {
        cairo_arc(cr, c(0), c(1), r, -M_PI, M_PI);
        cairo_set_source_rgba(cr, col(0), col(1), col(2), alpha);
        cairo_fill(cr);
    }

    void drawCircle(cairo_t *cr, nump::math::Circle circle, arma::vec3 col, double alpha) {
        drawCircle(cr, circle.centre, circle.radius, col, alpha);
    }

    void drawTree(cairo_t *cr, const nump::SearchTree::TreeT& tree, double r) {
        // Draw edges:
        cairo_set_line_width(cr, r * 0.5);
        for (auto& node : tree.nodes) {
            if (node->parent == nullptr) {
                continue;
            }

            arma::vec2 pos = node->value.state;
            arma::vec2 parent = tree.parent(node)->value.state;

            cairoMoveTo(cr, pos);
            cairoLineTo(cr, parent);

            double hr = tree.depth(node) / (double)tree.height();
            cairo_set_source_rgba(cr, hr, hr, 1 - hr, 1);
            cairo_stroke(cr);
        }

        // Draw vertices:
        for (auto& node : tree.nodes) {
            arma::vec2 pos = node->value.state;

            double hr = tree.depth(node) / (double)tree.height();
            drawCircle(cr, pos, r, {hr, hr, 1 - hr});
        }
    }

    void drawSearchTree(cairo_t *cr, const nump::SearchTree& searchTree) {
        auto& tree = searchTree.tree;

        cairo_save(cr);

        double r = deviceToUserDistance(cr, {2, 0})(0);
        drawCircle(cr, searchTree.initialState(), r, {1, 0.5, 0.5});
        drawCircle(cr, searchTree.goalState(), r, {0.5, 1, 0.5});

        // Draw obstacles:
        for (auto& obs : searchTree.obstacles) {
            drawCircle(cr, obs, {0,0,0}, 0.3);
        }

        // Draw edges:
        cairo_set_line_width(cr, r * 0.5);
        for (auto& node : tree.nodes) {
            if (node->parent == nullptr) {
                continue;
            }

            arma::vec2 pos = node->value.state;
            arma::vec2 parent = tree.parent(node)->value.state;

            cairoMoveTo(cr, pos);
            cairoLineTo(cr, parent);

//            double hr = tree.depth(node) / (double)tree.height();
            double hr = node->value.cost / searchTree.maxCost();
            cairo_set_source_rgba(cr, hr, hr, 1 - hr, 1);
            cairo_stroke(cr);
        }

        // Draw vertices:
        for (auto& node : tree.nodes) {
            arma::vec2 pos = node->value.state;

//            double hr = tree.depth(node) / (double)tree.height();
            double hr = node->value.cost / searchTree.maxCost();
            drawCircle(cr, pos, r, {hr, hr, 1 - hr});
            drawString(cr, pos, r, node->value.cost);
        }

        cairo_restore(cr);
    }

}
}
}
