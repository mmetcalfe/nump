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

    void drawEdgeRRBT(cairo_t *cr,
        const nump::RRBT::GraphT::Edge& edge,
        const std::vector<nump::math::Circle>& obstacles,
        const std::vector<nump::math::Circle>& measurementRegions) {
        arma::vec3 colProgress = {0.5, 0.5, 0.5};
        arma::vec3 colSuccess = {0.2, 0.8, 0.2};
        arma::vec3 colFailure = {0.9, 0.5, 0.5};
        arma::vec3 colLine = {0.0, 0.0, 0.0};
        double lwHighlight = 0.05;
        double lwNormal = 0.01;
        double size = 0.1;
        double alphaNormal = 0.05;

        const nump::SearchTree::TrajT& traj = edge.value;
        nump::RRBT::BeliefNodePtr n1 = nullptr;
        for (auto nChild : edge.child.lock()->value.beliefNodes) {
            if (nChild->parent == nullptr) {
                std::cout << "NO PARENT: " << nChild << std::endl;
                continue;
            }
            if (nChild->parent->containingNode.lock() != edge.parent.lock()) {
                continue;
            }

            n1 = nChild->parent;
            break;
        }
        if (n1 == nullptr) {
            std::cout << "BAD EDGE" << std::endl;
            return;
        }

        // Perform belief propagation:
        auto n2 = nump::RRBT::propagate(traj, n1, obstacles, measurementRegions, [&](auto t, auto xt, auto nt) {
            double frac = t / traj.t;

            nump::RRBT::StateCovT fullCov = nt->stateCov + nt->stateDistribCov;

            arma::vec3 colt = (1-frac)*colProgress + frac*colSuccess;
            double alpha = alphaNormal;
            cairo_set_line_width(cr, lwNormal);

            if (!nump::RRBT::satisfiesChanceConstraint(xt, fullCov, obstacles)) {
                colt = colFailure;
                alpha = 1;
                cairo_set_line_width(cr, lwHighlight);
            }

            cairoSetSourceRGBAlpha(cr, colt, alpha);
            drawRobot(cr, xt, size * 0.2);
            drawErrorEllipse(cr, xt, fullCov, 0.95);

            cairoSetSourceRGBAlpha(cr, colt * 0.5, alpha);
            drawErrorEllipse(cr, xt, nt->stateCov, 0.95);
        });

        cairoSetSourceRGBAlpha(cr, colSuccess, 1);
        drawRobot(cr, traj(traj.t), size);
    }

    void drawRRBT(cairo_t *cr, const nump::RRBT& rrbt) {
        arma::vec3 colObstacle = {0.0, 0.0, 0.0};
        arma::vec3 colRegion = {0.5, 0.5, 0.5};
        arma::vec3 colInitial = {0.5, 0.5, 0.5};
        arma::vec3 colProgress = {0.5, 0.5, 0.5};
        arma::vec3 colSuccess = {0.5, 0.9, 0.5};
        arma::vec3 colFailure = {0.9, 0.5, 0.5};
        arma::vec3 colNode = {0.5, 0.5, 0.5};
        arma::vec3 colLine = {0.0, 0.0, 0.0};
        arma::vec3 colText = {0.0, 0.0, 1.0};
        double alphaNormal = 0.05;
        double lwHighlight = 0.05;
        double lwNormal = 0.01;

        cairo_save(cr);

        double r = 0.04; //deviceToUserDistance(cr, {2, 0})(0);
        cairo_set_source_rgb(cr, 1, 0.5, 0.5);
        drawRobot(cr, rrbt.init, r);

        cairo_set_source_rgb(cr, 0.5, 1, 0.5);
        drawRobot(cr, rrbt.goal, r);

        // Draw obstacles:
        for (auto& reg : rrbt.measurementRegions) {
            fillCircle(cr, reg, colRegion, 0.3);
        }
        for (auto& obs : rrbt.obstacles) {
            fillCircle(cr, obs, colObstacle, 0.3);
        }

        // Draw graph edges:
        cairo_set_line_cap(cr,CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(cr,CAIRO_LINE_JOIN_ROUND);
        cairo_set_line_width(cr, lwNormal);
        for (auto& edge : rrbt.graph.edges) {
            cairoSetSourceRGBAlpha(cr, colLine, alphaNormal);
            cairoMoveTo(cr, edge.parent.lock()->value.state.rows(0,1));
            cairoLineTo(cr, edge.child.lock()->value.state.rows(0,1));
            cairo_stroke(cr);

            drawEdgeRRBT(cr, edge, rrbt.obstacles, rrbt.measurementRegions);
        }

        // Draw tree edges:
        for (auto& vertex : rrbt.graph.nodes) {
            for (auto queryNode : vertex->value.beliefNodes) {
                auto n = queryNode;
                int depth = 0;
                while (n != rrbt.initialBelief) {
                    cairo_set_line_width(cr, 0.001);
                    cairoSetSourceRGBAlpha(cr, {1.0,0.0,0.0}, 1);
                    if (!n->containingNode.lock()) {
                        std::cout << "BAD NODE" << std::endl;
                        break;
                    }
                    cairoMoveTo(cr, n->containingNode.lock()->value.state);
                    cairoLineTo(cr, n->parent->containingNode.lock()->value.state);
                    cairo_stroke(cr);
                    n = n->parent;
                    ++depth;
                    if (depth > rrbt.graph.nodes.size()) {
                        std::cout << "LOOP" << std::endl;
                        break;
                    }
                }

                // TODO: Edge getEdgeBetween(n->parent, n);

//                drawEdgeRRBT(cr, edge, rrbt.obstacles, rrbt.measurementRegions);
            }
        }

        // Draw vertices:
        for (auto& node : rrbt.graph.nodes) {
            nump::RRBT::StateT state = node->value.state;

            cairoSetSourceRGBAlpha(cr, colNode, 0.5);
            drawRobot(cr, state, r);

            for (auto& bn : node->value.beliefNodes) {
                cairo_set_line_width(cr, lwNormal);
                drawErrorEllipse(cr, state, bn->stateCov + bn->stateDistribCov, 0.95);
            }

            cairoSetSourceRGBAlpha(cr, colText, 1);
            showText(cr, state.rows(0, 1), r*0.6, node->value.beliefNodes.size());
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
