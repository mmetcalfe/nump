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

using nump::math::Transform2D;
using utility::math::geometry::Ellipse;

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

    void cairoSetSourceRGB(cairo_t *cr, arma::vec3 rgb) {
        cairo_set_source_rgb(cr, rgb(0), rgb(1), rgb(2));
    }

    void cairoSetSourceRGBAlpha(cairo_t *cr, arma::vec3 rgb, double alpha) {
        cairo_set_source_rgba(cr, rgb(0), rgb(1), rgb(2), alpha);
    }


    void cairoTransformToLocal(cairo_t *cr, Transform2D trans) {
        cairo_translate(cr, trans.x(), trans.y());
        cairo_rotate(cr, trans.angle());
    }

    void showText(cairo_t *cr, arma::vec2 pos, double fontSize, const std::string &str) {
        cairo_save(cr);

        cairo_text_extents_t te;
        cairo_select_font_face(cr, "Georgia", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
        cairo_set_font_size(cr, fontSize);
        cairo_text_extents(cr, str.c_str(), &te);
        cairo_move_to(cr, pos(0) - te.width / 2 - te.x_bearing,
                       pos(1) - te.height / 2 - te.y_bearing);
        cairo_show_text(cr, str.c_str());
        cairo_stroke(cr); // TODO: Find out why cairo_show_text leaves behind a path.

        cairo_restore(cr);
    }

    void fillCircle(cairo_t *cr, arma::vec2 c, float r, arma::vec3 col, double alpha) {
        cairo_arc(cr, c(0), c(1), r, -M_PI, M_PI);
        cairo_set_source_rgba(cr, col(0), col(1), col(2), alpha);
        cairo_fill(cr);
    }

    void fillCircle(cairo_t *cr, nump::math::Circle circle, arma::vec3 col, double alpha) {
        fillCircle(cr, circle.centre, circle.radius, col, alpha);
    }

    void drawCircle(cairo_t *cr, nump::math::Circle circle) {
        cairo_arc(cr, circle.centre(0), circle.centre(1), circle.radius, -M_PI, M_PI);
    }

    void drawRobot(cairo_t *cr, arma::vec2 pos, double size) {
        double radius = 1.5 * size*0.15;
        double length = 1.5 * size*0.4;

        cairo_save(cr);

//        cairoTransformToLocal(cr, trans);

        double angle = std::acos(radius/length);
        double tipX = length;
        double baseX = radius * std::cos(angle);
        double baseY = radius * std::sin(angle);

//        cairoMoveTo(cr, {baseX, baseY});
//        cairoLineTo(cr, {tipX, 0});
//        cairoLineTo(cr, {baseX, -baseY});
        cairo_arc(cr, pos(0), pos(1), radius, -M_PI, M_PI);
//        cairo_close_path(cr);

        cairo_fill(cr);
//        cairo_set_line_width(cr, radius * 0.2);
//        cairo_stroke(cr);

        cairo_restore(cr);
    }

    void drawRobot(cairo_t *cr, Transform2D trans, double size) {
        double radius = 1.5 * size*0.15;
        double length = 1.5 * size*0.4;

        cairo_save(cr);

        cairoTransformToLocal(cr, trans);

        double angle = std::acos(radius/length);
        double tipX = length;
        double baseX = radius * std::cos(angle);
        double baseY = radius * std::sin(angle);

        cairoMoveTo(cr, {baseX, baseY});
        cairoLineTo(cr, {tipX, 0});
//        cairoLineTo(cr, {baseX, -baseY});
        cairo_arc_negative(cr, 0, 0, radius, -angle, angle);
        cairo_close_path(cr);

        cairo_fill(cr);
//        cairo_set_line_width(cr, radius * 0.2);
//        cairo_stroke(cr);

        cairo_restore(cr);
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
            fillCircle(cr, pos, r, {hr, hr, 1 - hr});
        }
    }

    void drawNodeTrajectory(cairo_t *cr, const nump::SearchTree::TrajT& traj, double lineWidth, int numCurvePoints = 50) {
        cairo_save(cr);
        cairo_set_line_width(cr, lineWidth);
        cairo_set_line_cap(cr,CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(cr,CAIRO_LINE_JOIN_ROUND);

        cairoMoveTo(cr, traj(0).rows(0,1));
        for (int i = 1; i < numCurvePoints; i++) {
            double t = (i / double(numCurvePoints)) * traj.t;
            nump::SearchTree::StateT posT = traj(t);
            cairoLineTo(cr, posT.rows(0,1));
        }
        cairoLineTo(cr, traj(traj.t).rows(0,1));

        cairo_stroke(cr);

        cairo_restore(cr);
    }

    void drawNodeTrajectoryPoints(cairo_t *cr, const nump::SearchTree::TrajT& traj, double size, int numCurvePoints = 50) {
        cairo_save(cr);

        drawRobot(cr, traj(0), size);

        cairoMoveTo(cr, traj(0).rows(0,1));
        for (int i = 1; i < numCurvePoints; i++) {
            double t = (i / double(numCurvePoints)) * traj.t;
            drawRobot(cr, traj(t), size);
        }
        drawRobot(cr, traj(traj.t), size);

        cairo_restore(cr);
    }

    void drawSearchTree(cairo_t *cr, const nump::SearchTree& searchTree) {
        auto& tree = searchTree.tree;

        cairo_save(cr);

        double r = 0.04; //deviceToUserDistance(cr, {2, 0})(0);
        cairo_set_source_rgb(cr, 1, 0.5, 0.5);
        drawRobot(cr, searchTree.initialState(), r);

        cairo_set_source_rgb(cr, 0.5, 1, 0.5);
        drawRobot(cr, searchTree.goalState(), r);

        // Draw obstacles:
        for (auto& obs : searchTree.obstacles) {
            fillCircle(cr, obs, {0, 0, 0}, 0.3);
        }

        // Draw edges:
        cairo_set_line_cap(cr,CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(cr,CAIRO_LINE_JOIN_ROUND);
        for (auto& node : tree.nodes) {
            if (node->parent == nullptr) {
                continue;
            }

            double hr = node->value.cost / searchTree.maxCost();
            arma::vec3 col = arma::normalise(arma::vec({hr, hr, 1 - hr}));
            cairoSetSourceRGBAlpha(cr, col * 0.5, 1);

            drawNodeTrajectoryPoints(cr, node->value.traj, r * 0.2);
//            drawNodeTrajectory(cr, node->value.traj, r * 0.2);
        }

        // Draw vertices:
        for (auto& node : tree.nodes) {
            nump::SearchTree::StateT state = node->value.state;

//            double hr = tree.depth(node) / (double)tree.height();
            double hr = node->value.cost / searchTree.maxCost();
            arma::vec3 col = arma::normalise(arma::vec({hr, 0, 1 - hr}));
            col(1) = col(0);
            cairoSetSourceRGBAlpha(cr, col, 1);
            drawRobot(cr, state, r);
//            drawRobot(cr, {state, 0}, r); // TODO: Fix for arma::vec2.

            cairo_set_source_rgb(cr, 0.7, 0.7, 0.7);
            showText(cr, state.rows(0, 1), r*0.15, node->value.cost);
        }

        // Draw optimal path:
        auto goalNode = searchTree.createValidNodeForState(searchTree.goalState());
        if (goalNode) {
            cairoSetSourceRGBAlpha(cr, {0, 0.7, 0}, 0.8);
            auto zNearby = searchTree.nearVertices(goalNode, tree.nodes.size());
            searchTree.optimiseParent(goalNode, zNearby);
            for (auto pathNode = goalNode; pathNode != nullptr; pathNode = pathNode->parent) {
                drawRobot(cr, pathNode->value.state, r);
                drawNodeTrajectoryPoints(cr, pathNode->value.traj, r * 0.5);
            }
        }

        cairo_restore(cr);
    }

    void drawRRBT(cairo_t *cr, const nump::RRBT& rrbt) {
        auto& tree = rrbt.graph;

        cairo_save(cr);

        double r = 0.04; //deviceToUserDistance(cr, {2, 0})(0);
        cairo_set_source_rgb(cr, 1, 0.5, 0.5);
        drawRobot(cr, rrbt.init, r);

        cairo_set_source_rgb(cr, 0.5, 1, 0.5);
        drawRobot(cr, rrbt.goal, r);

        // Draw obstacles:
        for (auto& obs : rrbt.obstacles) {
            fillCircle(cr, obs, {0, 0, 0}, 0.3);
        }

//        // Draw edges:
//        cairo_set_line_cap(cr,CAIRO_LINE_CAP_ROUND);
//        cairo_set_line_join(cr,CAIRO_LINE_JOIN_ROUND);
//        for (auto& node : tree.nodes) {
//            if (node->parent == nullptr) {
//                continue;
//            }
//
//            double hr = node->value.cost / rrbt.maxCost();
//            arma::vec3 col = arma::normalise(arma::vec({hr, hr, 1 - hr}));
//            cairoSetSourceRGBAlpha(cr, col * 0.5, 1);
//
//            drawNodeTrajectoryPoints(cr, node->value.traj, r * 0.2);
////            drawNodeTrajectory(cr, node->value.traj, r * 0.2);
//        }

       // Draw vertices:
        for (auto& node : tree.nodes) {
            nump::RRBT::StateT state = node->value.state;
//
////            double hr = tree.depth(node) / (double)tree.height();
//            double hr = node->value.cost / rrbt.maxCost();
//            arma::vec3 col = arma::normalise(arma::vec({hr, 0, 1 - hr}));
//            col(1) = col(0);
          //  cairoSetSourceRGBAlpha(cr, col, 1);
            cairoSetSourceRGBAlpha(cr, {0.7, 0, 1}, 0.5);
            drawRobot(cr, state, r);

            for (auto& node : node->value.beliefNodes) {
                drawErrorEllipse(cr, state, node->stateCov, 0.95);
            }

////            drawRobot(cr, {state, 0}, r); // TODO: Fix for arma::vec2.
//
//            cairo_set_source_rgb(cr, 0.7, 0.7, 0.7);
//            showText(cr, state.rows(0, 1), r*0.15, node->value.cost);
       }

//        // Draw optimal path:
//        auto goalNode = rrbt.createValidNodeForState(rrbt.goalState());
//        if (goalNode) {
//            cairoSetSourceRGBAlpha(cr, {0, 0.7, 0}, 0.8);
//            auto zNearby = rrbt.nearVertices(goalNode, tree.nodes.size());
//            searchTree.optimiseParent(goalNode, zNearby);
//            for (auto pathNode = goalNode; pathNode != nullptr; pathNode = pathNode->parent) {
//                drawRobot(cr, pathNode->value.state, r);
//                drawNodeTrajectoryPoints(cr, pathNode->value.traj, r * 0.5);
//            }
//        }

        cairo_restore(cr);
    }

    void drawErrorEllipse(cairo_t *cr, const arma::vec2& mean, const arma::mat22& cov, double confidence) {
        auto ellipse = Ellipse::forConfidenceRegion(mean, cov);
        drawEllipse(cr, ellipse);
    }

    void drawEllipse(cairo_t *cr, const Ellipse& ellipse) {
        arma::vec2 size = ellipse.getSize();

        if (size(0) + size(1) > 1e3) {
            std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
                     << "Ellipse dimensions too large ("
                     << size(0) << ", "
                     << size(1) << ")"
                     << std::endl;
            return;
        }

        cairo_save(cr);
        cairo_set_line_width(cr, 0.01);
        cairoTransformToLocal(cr, ellipse.getTransform());


        cairo_scale(cr, size(0), size(1));
        cairo_arc(cr, 0, 0, 0.5, -M_PI, M_PI); // diameter 1

        cairo_stroke(cr);

        cairo_restore(cr);
    }

    void drawRotatedRectangle(cairo_t *cr, const RotatedRectangle& rect) {
        cairo_save(cr);
        cairo_set_line_width(cr, 0.01);
        cairoTransformToLocal(cr, rect.getTransform());

        arma::vec2 size = rect.getSize();

        cairo_scale(cr, size(0), size(1));
//        cairo_arc(cr, 0, 0, 0.5, -M_PI, M_PI); // diameter 1
        cairoMoveTo(cr, {-0.5, -0.5});
        cairoLineTo(cr, {0.5, -0.5});
        cairoLineTo(cr, {0.5, 0.5});
        cairoLineTo(cr, {-0.5, 0.5});
        cairo_close_path(cr);

        cairo_stroke(cr);

        cairo_restore(cr);
    }



}
}
}
