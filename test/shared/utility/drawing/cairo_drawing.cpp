//
// Created by Mitchell Metcalfe on 16/08/15.
//

//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include "cairo_drawing.h"

#include <armadillo>
#include <cairo/cairo.h>

#include "nump.h"
#include "shared/utility/math/distributions.h"

using nump::math::Transform2D;
using nump::math::Rotation2D;
using utility::math::geometry::Ellipse;

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

    void drawLine(cairo_t *cr, arma::vec2 from, arma::vec2 to) {
        cairoMoveTo(cr, from);
        cairoLineTo(cr, to);
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
        cairo_move_to(cr, circle.centre(0) - circle.radius, circle.centre(1));
        cairo_arc(cr, circle.centre(0), circle.centre(1), circle.radius, -M_PI, M_PI);
        cairo_close_path(cr);
    }

    void drawRobot(cairo_t *cr, arma::vec2 pos, double size, bool noFill) {
        double radius = 1.5 * size*0.15;
        cairo_save(cr);
        cairo_arc(cr, pos(0), pos(1), radius, -M_PI, M_PI);
        if (!noFill) {
            cairo_fill(cr);
        }
        cairo_restore(cr);
    }

    void drawRobot(cairo_t *cr, Transform2D trans, double size, bool noFill) {
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

        if (!noFill) {
            cairo_fill(cr);
        }

        cairo_restore(cr);
    }

    void drawRobot(cairo_t *cr, Transform2D trans, double size, arma::vec2 footprintSize) {
        double radius = 1.5 * size*0.15;
        double length = 1.5 * size*0.4;

        cairo_save(cr);

        drawRotatedRectangle(cr, {trans, footprintSize});
        cairo_fill(cr);
        drawRobot(cr, trans, size);

        cairo_restore(cr);
    }

    void drawRobot(cairo_t *cr, nump::BipedRobotModel::State state, double size) {
        drawRobot(cr, state.position, size);
    }
    void drawRobot(cairo_t *cr, nump::BipedRobotModel::State state, double size, arma::vec2 footprintSize) {
        drawRobot(cr, state.position, size, footprintSize);
    }

    void drawTree(cairo_t *cr, const nump::SearchTree::TreeT& tree, double r) {
        // Draw edges:
        cairo_set_line_width(cr, r * 0.5);
        for (auto& node : tree.nodes) {
            if (node->parent == nullptr) {
                continue;
            }

            arma::vec2 pos = node->value.state.position.xy();
            arma::vec2 parent = tree.parent(node)->value.state.position.xy();

            cairoMoveTo(cr, pos);
            cairoLineTo(cr, parent);

            double hr = tree.depth(node) / (double)tree.height();
            cairo_set_source_rgba(cr, hr, hr, 1 - hr, 1);
            cairo_stroke(cr);
        }

        // Draw vertices:
        for (auto& node : tree.nodes) {
            arma::vec2 pos = node->value.state.position.xy();

            double hr = tree.depth(node) / (double)tree.height();
            fillCircle(cr, pos, r, {hr, hr, 1 - hr});
        }
    }

    void drawNodeTrajectory(cairo_t *cr, const nump::SearchTree::TrajT& traj, double lineWidth, int numCurvePoints = 50) {
        cairo_save(cr);
        cairo_set_line_width(cr, lineWidth);
        cairo_set_line_cap(cr,CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(cr,CAIRO_LINE_JOIN_ROUND);

        cairoMoveTo(cr, traj(0).position.rows(0,1));
        for (int i = 1; i < numCurvePoints; i++) {
            double t = (i / double(numCurvePoints)) * traj.t;
            nump::SearchTree::StateT posT = traj(t);
            cairoLineTo(cr, posT.position.rows(0,1));
        }
        cairoLineTo(cr, traj(traj.t).position.rows(0,1));

        cairo_stroke(cr);

        cairo_restore(cr);
    }

    void drawNodeTrajectoryPoints(cairo_t *cr, const nump::SearchTree::TrajT& traj, double size, int numCurvePoints = 50) {
        cairo_save(cr);

        drawRobot(cr, traj(0), size);

        cairoMoveTo(cr, traj(0).position.rows(0,1));
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
        drawRobot(cr, searchTree.scenario.initialState, r);

        cairo_set_source_rgb(cr, 0.5, 1, 0.5);
        drawRobot(cr, searchTree.scenario.goalState, r);

        // Draw obstacles:
        for (auto& obs : searchTree.scenario.obstacles) {
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

            cairo_set_source_rgb(cr, 0.7, 0.7, 0.7);
            showText(cr, state.position.rows(0, 1), r*0.15, node->value.cost);
        }

       // Draw optimal path:
       auto goalNode = searchTree.createValidNodeForState({searchTree.scenario.goalState});
       if (goalNode) {
           cairoSetSourceRGBAlpha(cr, {0, 0.7, 0}, 0.8);
           auto zNearby = searchTree.nearVertices(goalNode, tree.nodes.size());
           searchTree.optimiseParent(goalNode, zNearby);
           for (auto pathNode = goalNode; pathNode != nullptr; pathNode = pathNode->parent) {
               drawRobot(cr, pathNode->value.state, r);
               drawNodeTrajectoryPoints(cr, pathNode->value.traj, r * 0.5);
           }
       }


       double lwNormal = 0.01;
       cairo_set_source_rgb(cr, 0.5, 1, 0.5);
       cairo_set_line_width(cr, lwNormal);
       // drawRobot(cr, rrbt.scenario.goalState, r);
       arma::vec2 target = {std::cos(searchTree.scenario.targetAngle), std::sin(searchTree.scenario.targetAngle)};
       arma::vec2 minTarget = {std::cos(searchTree.scenario.targetAngle-0.5*searchTree.scenario.targetAngleRange), std::sin(searchTree.scenario.targetAngle-0.5*searchTree.scenario.targetAngleRange)};
       arma::vec2 maxTarget = {std::cos(searchTree.scenario.targetAngle+0.5*searchTree.scenario.targetAngleRange), std::sin(searchTree.scenario.targetAngle+0.5*searchTree.scenario.targetAngleRange)};
       utility::drawing::drawLine(cr, searchTree.scenario.ball.centre, searchTree.scenario.ball.centre+target);
       utility::drawing::drawLine(cr, searchTree.scenario.ball.centre, searchTree.scenario.ball.centre+minTarget);
       utility::drawing::drawLine(cr, searchTree.scenario.ball.centre, searchTree.scenario.ball.centre+maxTarget);
       cairo_stroke(cr);


       cairo_restore(cr);
    }

    void drawEdgeRRBT(cairo_t *cr,
        const nump::RRBT::GraphT::Edge& edge,
        const numptest::SearchScenario::Config& scenario) {
        arma::vec3 colProgress = {0.5, 0.5, 0.5};
        arma::vec3 colSuccess = {0.2, 0.8, 0.2};
        arma::vec3 colFailure = {0.9, 0.5, 0.5};
        arma::vec3 colLine = {0.0, 0.0, 0.0};
        double lwHighlight = 0.05;
        double lwNormal = 0.01;
        double size = 0.1;
        double alphaNormal = 0.05;

        const nump::RRBT::TrajT& traj = edge.value;
        nump::RRBT::BeliefNodePtr n1 = nullptr;
        for (auto nChild : edge.child.lock()->value.beliefNodes) {
            if (nChild->parent == nullptr) {
//                std::cout << "NO PARENT: " << nChild << std::endl;
                continue;
            }
            if (nChild->parent->containingNode.lock() != edge.parent.lock()) {
                continue;
            }

            n1 = nChild->parent;
            break;
        }
        if (n1 == nullptr) {
//            std::cout << "b";
            return;
        }

        int stepNum = 0;
        int drawPeriod = 5;
        // Perform belief propagation:
        auto n2 = nump::RRBT::propagate(traj, n1, scenario.footprintSize, scenario.obstacles, scenario.measurementRegions, [&](auto t, auto xt, auto nt) {
            ++stepNum;
            if (stepNum % drawPeriod != 0) {
                return;
            }

            double frac = t / traj.t;

            nump::RRBT::StateCovT fullCov = nt->stateCov + nt->stateDistribCov;

            arma::vec3 colt = (1-frac)*colProgress + frac*colSuccess;
            double alpha = alphaNormal;
            cairo_set_line_width(cr, lwNormal);

            if (!nump::RRBT::satisfiesChanceConstraint(xt, fullCov, scenario.footprintSize, scenario.obstacles)) {
                colt = colFailure;
                alpha = 1;
                cairo_set_line_width(cr, lwHighlight);
            }

            cairoSetSourceRGBAlpha(cr, colt, alpha);
            drawRobot(cr, xt, size * 0.2);
            drawErrorEllipse(cr, xt, fullCov, 0.95);
            cairo_stroke(cr);

            cairoSetSourceRGBAlpha(cr, colt * 0.5, alpha);
            drawErrorEllipse(cr, xt, nt->stateCov, 0.95);
            cairo_stroke(cr);
        });

        cairoSetSourceRGBAlpha(cr, colSuccess, 1);
        drawRobot(cr, traj(traj.t), size);
    }


    void drawPath(cairo_t *cr, const nump::RRBT& rrbt, std::shared_ptr<nump::RRBT::BeliefNode> goalNode) {
        arma::vec3 colObstacle = {0.0, 0.0, 0.0};
        arma::vec3 colRegion = {0.5, 0.5, 1};
        arma::vec3 colInitial = {0.5, 0.5, 0.5};
        arma::vec3 colProgress = {0.5, 0.5, 0.5};
        arma::vec3 colSuccess = {0.5, 0.9, 0.5};
        arma::vec3 colFailure = {0.9, 0.5, 0.5};
        arma::vec3 colNode = {0.5, 0.5, 0.5};
        arma::vec3 colLine = {0.0, 0.0, 0.0};
        arma::vec3 colText = {0.0, 0.0, 1.0};
        double alphaNormal = 0.05;
        double lwHighlight = 0.01;
        double lwNormal = 0.005;

        if (goalNode == nullptr) {
            return;
        }

        auto n = goalNode;
        int depth = 0;
        while (n != rrbt.initialBelief) {

            if (!n->containingNode.lock()) {
                break;
            }

            cairoMoveTo(cr, n->containingNode.lock()->value.state.position.head(2));
            cairoLineTo(cr, n->parent->containingNode.lock()->value.state.position.head(2));
            cairo_stroke(cr);

            cairo_set_line_width(cr, lwHighlight);
            drawErrorEllipse(cr, n->containingNode.lock()->value.state, n->stateCov + n->stateDistribCov, 0.95);
            cairo_stroke(cr);
            n = n->parent;
            ++depth;
            if (depth > rrbt.graph.nodes.size()) {
                break;
            }
        }
    }

    void drawBestPaths(cairo_t *cr, const nump::RRBT& rrbt) {
        arma::vec3 colObstacle = {0.0, 0.0, 0.0};
        arma::vec3 colRegion = {0.5, 0.5, 1};
        arma::vec3 colInitial = {0.5, 0.5, 0.5};
        arma::vec3 colProgress = {0.5, 0.5, 0.5};
        arma::vec3 colSuccess = {0.5, 0.9, 0.5};
        arma::vec3 colFailure = {0.9, 0.5, 0.5};
        arma::vec3 colNode = {0.5, 0.5, 0.5};
        arma::vec3 colLine = {0.0, 0.0, 0.0};
        arma::vec3 colText = {0.0, 0.0, 1.0};
        double alphaNormal = 0.05;
        double lwHighlight = 0.01;
        double lwNormal = 0.005;

        double r = 0.1; //deviceToUserDistance(cr, {2, 0})(0);

        cairo_set_line_width(cr, lwNormal);
        cairoSetSourceRGBAlpha(cr, colSuccess, 0.5);

        for (auto weakGoalVertex : rrbt.goalVertices) {
            auto goalVertex = weakGoalVertex.lock();
            drawRobot(cr, goalVertex->value.state, r);

            for (auto goalNode : goalVertex->value.beliefNodes) {
                drawPath(cr, rrbt, goalNode);
            }
        }

        cairoSetSourceRGBAlpha(cr, {0.8, 0, 0.8}, 0.8);
        drawPath(cr, rrbt, rrbt.findBestGoalStateWithSuccessThreshold());
    }

    void drawRRBT(cairo_t *cr, const nump::RRBT& rrbt) {
        arma::vec3 colObstacle = {0.0, 0.0, 0.0};
        arma::vec3 colRegion = {0.5, 0.5, 1};
        arma::vec3 colInitial = {0.5, 0.5, 0.5};
        arma::vec3 colProgress = {0.5, 0.5, 0.5};
        arma::vec3 colSuccess = {0.5, 0.9, 0.5};
        arma::vec3 colFailure = {0.9, 0.5, 0.5};
        arma::vec3 colNode = {0.5, 0.5, 0.5};
        arma::vec3 colLine = {0.0, 0.0, 0.0};
        arma::vec3 colText = {0.0, 0.0, 1.0};
        double alphaNormal = 0.05;
        double lwHighlight = 0.01;
        double lwNormal = 0.005;

        double r = 0.1; //deviceToUserDistance(cr, {2, 0})(0);

        cairo_save(cr);

        // Draw graph edges:
        cairo_set_line_cap(cr,CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(cr,CAIRO_LINE_JOIN_ROUND);
        cairo_set_line_width(cr, lwNormal);
        for (auto& edge : rrbt.graph.edges) {
            cairoSetSourceRGBAlpha(cr, colLine, alphaNormal);
            cairoMoveTo(cr, edge.parent.lock()->value.state.position.head(2));
            cairoLineTo(cr, edge.child.lock()->value.state.position.head(2));
            cairo_stroke(cr);

            // drawEdgeRRBT(cr, edge, rrbt.scenario);
        }

        // std::cout << __LINE__ << ", CAIRO STATUS: " <<  cairo_status_to_string(cairo_status(cr)) << std::endl;

        // std::cout << "Drawing tree edges: " << cr << std::endl;
        // Draw tree edges:
//         for (auto& vertex : rrbt.graph.nodes) {
//             for (auto queryNode : vertex->value.beliefNodes) {
//                 auto n = queryNode;
//                 int depth = 0;
//                 while (n != rrbt.initialBelief) {
//                     cairo_set_line_width(cr, 0.001);
//                     cairoSetSourceRGBAlpha(cr, {1.0,0.0,0.0}, 1);
//                     if (!n->containingNode.lock()) {
//                         std::cout << "b";
//                         break;
//                     }
//                     cairoMoveTo(cr, n->containingNode.lock()->value.state);
//                     cairoLineTo(cr, n->parent->containingNode.lock()->value.state);
//                     cairo_stroke(cr);
//                     n = n->parent;
//                     ++depth;
//                     if (depth > rrbt.graph.nodes.size()) {
//                         std::cout << "l";
//                         break;
//                     }
//                 }
//
//                 // TODO: Edge getEdgeBetween(n->parent, n);
//
// //                drawEdgeRRBT(cr, edge, rrbt.obstacles, rrbt.measurementRegions);
//             }
//         }
        // std::cout << std::endl;

        // Draw vertices:
        for (auto& node : rrbt.graph.nodes) {
            nump::RRBT::StateT state = node->value.state;

            cairoSetSourceRGBAlpha(cr, colNode, 0.5);
            cairo_set_line_width(cr, lwNormal);
            drawRobot(cr, state, r, rrbt.scenario.footprintSize);

            cairo_set_line_width(cr, lwNormal);
            for (auto& bn : node->value.beliefNodes) {
                drawErrorEllipse(cr, state, bn->stateCov + bn->stateDistribCov, 0.95);
            }
            cairo_stroke(cr);

            cairoSetSourceRGBAlpha(cr, colText, 1);
            showText(cr, state.position.head(2), r*0.2, node->value.beliefNodes.size());
        }

        // Draw obstacles:
        cairo_push_group(cr);
        cairoSetSourceRGB(cr, colRegion);
        for (auto& reg : rrbt.scenario.measurementRegions) {
            drawCircle(cr, reg);
        }
        cairo_fill(cr);
        cairo_pop_group_to_source(cr);
        cairo_paint_with_alpha(cr, 0.3);

        // Draw information regions:
        cairo_push_group(cr);
        cairoSetSourceRGB(cr, colObstacle);
        for (auto& obs : rrbt.scenario.obstacles) {
            drawCircle(cr, obs);
        }
        cairo_fill(cr);
        cairo_pop_group_to_source(cr);
        cairo_paint_with_alpha(cr, 0.3);

        drawBestPaths(cr, rrbt);

        cairo_set_line_width(cr, lwNormal);
        cairo_set_source_rgb(cr, 1, 0.5, 0.5);
        drawRobot(cr, rrbt.scenario.initialState, r);
        drawErrorEllipse(cr, rrbt.initialBelief->containingNode.lock()->value.state, rrbt.initialBelief->stateCov + rrbt.initialBelief->stateDistribCov, 0.95);
        cairo_stroke(cr);

        cairo_set_source_rgb(cr, 0.5, 1, 0.5);
        cairo_set_line_width(cr, lwNormal);
        // drawRobot(cr, rrbt.scenario.goalState, r);
        arma::vec2 target = {std::cos(rrbt.scenario.targetAngle), std::sin(rrbt.scenario.targetAngle)};
        arma::vec2 minTarget = {std::cos(rrbt.scenario.targetAngle-0.5*rrbt.scenario.targetAngleRange), std::sin(rrbt.scenario.targetAngle-0.5*rrbt.scenario.targetAngleRange)};
        arma::vec2 maxTarget = {std::cos(rrbt.scenario.targetAngle+0.5*rrbt.scenario.targetAngleRange), std::sin(rrbt.scenario.targetAngle+0.5*rrbt.scenario.targetAngleRange)};
        utility::drawing::drawLine(cr, rrbt.scenario.ball.centre, rrbt.scenario.ball.centre+target);
        utility::drawing::drawLine(cr, rrbt.scenario.ball.centre, rrbt.scenario.ball.centre+minTarget);
        utility::drawing::drawLine(cr, rrbt.scenario.ball.centre, rrbt.scenario.ball.centre+maxTarget);
        cairo_stroke(cr);

        cairo_restore(cr);
    }

    bool drawErrorEllipse(cairo_t *cr, const arma::vec2& mean, const arma::mat22& cov, double confidence) {
        auto ellipse = utility::math::distributions::confidenceRegion(mean, cov, confidence);
        return drawEllipse(cr, ellipse);
    }

    bool drawErrorEllipse(cairo_t *cr, const nump::BipedRobotModel::State& mean, const nump::BipedRobotModel::MotionCov& cov, double confidence) {
        auto ellipse = utility::math::distributions::confidenceRegion(mean.position.head(2), cov.submat(0,0,1,1), confidence, arma::size(mean.position)(0));
        return drawEllipse(cr, ellipse);
    }

    bool drawEllipse(cairo_t *cr, const Ellipse& ellipse) {
        arma::vec2 size = ellipse.getSize();

        if (size(0) + size(1) > 1e3) {
            std::cerr << __FILE__ << ", " << __LINE__ << " - " << __func__ << ": "
                     << "Ellipse dimensions too large ("
                     << size(0) << ", "
                     << size(1) << ")"
                     << std::endl;
            return false;
        }

        if (!ellipse.getTransform().is_finite()) {
            std::cout << "Warning: Cannot draw non-finite ellipse with trans =" << ellipse.getTransform() << std::endl;
            return false;
        } else if (!ellipse.getSize().is_finite()) {
            std::cout << "Warning: Cannot draw non-finite ellipse with size = " << ellipse.getSize() << std::endl;
            return false;
        }

        cairo_save(cr);

        cairoTransformToLocal(cr, ellipse.getTransform());

        cairo_scale(cr, size(0), size(1));

        drawCircle(cr, {{0.0, 0.0}, 0.5});

        cairo_restore(cr);

        // cairo_stroke(cr);

        return true;
    }

    void drawRotatedRectangle(cairo_t *cr, const RotatedRectangle& rect) {
        cairo_save(cr);
        cairoTransformToLocal(cr, rect.getTransform());

        arma::vec2 size = rect.getSize();

        cairo_scale(cr, size(0), size(1));
//        cairo_arc(cr, 0, 0, 0.5, -M_PI, M_PI); // diameter 1
        cairoMoveTo(cr, {-0.5, -0.5});
        cairoLineTo(cr, {0.5, -0.5});
        cairoLineTo(cr, {0.5, 0.5});
        cairoLineTo(cr, {-0.5, 0.5});
        cairo_close_path(cr);

        cairo_restore(cr);
    }

    // Draws a rectangle swept around its centre through the given range of angles.
    void drawRectangleRotationRange(cairo_t *cr, arma::vec2 position, arma::vec2 footprintSize, arma::vec2 angleRange) {
        cairo_save(cr);

        double rangeSize = std::abs(angleRange(0) - angleRange(1));
        double halfAngle = rangeSize / 2;
        double midAngle = (angleRange(0) + angleRange(1)) / 2;
        cairoTransformToLocal(cr, {position, midAngle});

        double hw = footprintSize(0) / 2;
        double hh = footprintSize(1) / 2;

        double fpRad = arma::norm(footprintSize / 2);

        double lw = hw / std::cos(halfAngle);
        double lh = hh / std::cos(halfAngle);

        auto rot = Rotation2D::createRotation(halfAngle);

        arma::vec2 cornerL = {hw, -hh};
        arma::vec2 cornerR = {hw, hh};

        double cornerRAngle = std::atan2(cornerR(1), cornerR(0));
        double cornerLAngle = std::atan2(cornerL(1), cornerL(0));

        if (lw > fpRad) {
            // Don't draw corners for w.
            if (lh > fpRad) {
                // Don't draw corners for h.
                cairoMoveTo(cr, {fpRad, 0});
                cairo_arc(cr, 0, 0, fpRad, 0, 2*arma::datum::pi);
            } else {
                cairoMoveTo(cr, {0, -lh});
                cairo_arc_negative(cr, 0, 0, fpRad, arma::datum::pi + cornerRAngle + halfAngle, arma::datum::pi + cornerLAngle - halfAngle);
                cairoLineTo(cr, {0, lh});
                cairo_arc_negative(cr, 0, 0, fpRad, cornerRAngle + halfAngle, cornerLAngle - halfAngle);
            }
        } else {
            if (lh > fpRad) {
                // Don't draw corners for h.
                cairoMoveTo(cr, {lw, 0});
                cairo_arc_negative(cr, 0, 0, fpRad, cornerLAngle + halfAngle, arma::datum::pi + cornerRAngle - halfAngle);
                cairoLineTo(cr, {-lw, 0});
                cairo_arc_negative(cr, 0, 0, fpRad, arma::datum::pi + cornerLAngle + halfAngle, cornerRAngle - halfAngle);
            } else {
                cairoMoveTo(cr, {lw, 0});
                cairo_arc_negative(cr, 0, 0, fpRad, cornerLAngle + halfAngle, cornerLAngle - halfAngle);
                cairoLineTo(cr, {0, -lh});
                cairo_arc_negative(cr, 0, 0, fpRad, arma::datum::pi + cornerRAngle + halfAngle, arma::datum::pi + cornerRAngle - halfAngle);
                cairoLineTo(cr, {-lw, 0});
                cairo_arc_negative(cr, 0, 0, fpRad, arma::datum::pi + cornerLAngle + halfAngle, arma::datum::pi + cornerLAngle - halfAngle);
                cairoLineTo(cr, {0, lh});
                cairo_arc_negative(cr, 0, 0, fpRad, cornerRAngle + halfAngle, cornerRAngle - halfAngle);
            }
        }

        cairo_close_path(cr);
        cairo_restore(cr);
    }

}
}
