//
// Created by Mitchell Metcalfe on 16/08/15.
//

//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include "SearchTree.h"

#include <armadillo>
#include "math/geometry.h"
#include "tree.h"
#include "shared/utility/drawing/SearchTreeDrawing.h"

using shared::utility::drawing::drawCircle;

namespace nump {

    typedef SearchTree::StateT StateT;
    typedef SearchTree::NodeT NodeT;

    SearchTree::SearchTree(StateT init, StateT goal)
            : tree(SearchNode {init, 0, Steer()}), init(init), goal(goal) {
    }


    StateT SearchTree::initialState() const {
        return init;
    }

    StateT SearchTree::goalState() const {
        return goal;
    }

    inline double SearchTree::distance(StateT a, StateT b) {
        return arma::norm(a - b);
    }

    NodeT SearchTree::nearest(StateT state) const {

        auto min_it = std::min_element(tree.nodes.begin(), tree.nodes.end(), [state](NodeT a, NodeT b) {
            double distA = distance(a->value.state, state);
            double distB = distance(b->value.state, state);
            return distA < distB;
        });

        return *min_it;
    }

    double SearchTree::maxCost() const {

        auto min_it = std::max_element(tree.nodes.begin(), tree.nodes.end(), [](NodeT a, NodeT b) {
            return a->value.cost < b->value.cost;
        });

        return (*min_it)->value.cost;
    }

    StateT SearchTree::sample() {
        arma::vec rvec = arma::randu(2);
        return {rvec(0), rvec(1)};
    }

    Steer SearchTree::steer(StateT from, StateT to) {
        Steer s;

        arma::vec2 diff = to - from;
        double l = arma::norm(diff);

        s.x = from;
        s.u = arma::normalise(diff);

        double maxDist = 1000;
//            s.t = std::min(l, maxDist);
        if (l < maxDist) {
            s.t = l;
            s.reachesTarget = true;
        } else {
            s.t = maxDist;
            s.reachesTarget = false;
        }

        return s;
    }

    std::vector<NodeT> SearchTree::nearVertices(NodeT queryNode, unsigned long numVertices) const {
        std::vector<NodeT> near;

        double gammaRRTs = 1;
        double cardV = numVertices;
        double threshold = gammaRRTs * pow(log(cardV) / cardV, 1.0 / 2);

        cairo_arc(cairo, queryNode->value.state(0), queryNode->value.state(1), threshold, -M_PI, M_PI);
        cairo_set_source_rgba(cairo, 1, 0.7, 1, 1);
        cairo_set_line_width(cairo, 0.02);
        cairo_stroke_preserve(cairo);
        cairo_set_source_rgba(cairo, 1, 0.7, 1, 0.5);
        cairo_fill(cairo);

        for (auto &node : tree.nodes) {
            if (distance(node->value.state, queryNode->value.state) < threshold) {
                near.push_back(node);
            }
        }

        return near;
    }

    bool SearchTree::obstacleFree(Steer s) const {
        int numSteps = 100;
        for (int i = 0; i <= numSteps; i++) {
            double t = i / double(numSteps);
            arma::vec2 x = s(t*s.t);

            drawCircle(cairo, x, 0.001, {1, 1, 1}, 1);

            for (auto &obs : obstacles) {
                if (obs.contains(x)) {
                    drawCircle(cairo, x, 0.003, {0, 1, 1}, 1);

                    return false;
                }
            }
        }

        return true;
    }

    double SearchTree::J(Steer s) {
        return s.t;
    }

    inline double SearchTree::cost(SearchNode node) {
        return node.cost;
    }

//        def generateRRT(xInit, K, eta):
//            T = Tree(xInit)
//            for i in range(K):
//                    addNodeRRT(T, xInit, K, eta)
//            return T
//        template <class arma::vec2>
    SearchTree SearchTree::fromRRT(cairo_t *cr, StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles) {
        auto tree = SearchTree(init, goal);

        tree.cairo = cr; // TODO: Fix this.

        tree.obstacles = obstacles;

        for (int i = 0; i < n; i++) {
            arma::vec2 zRand = sample();
            tree.extendRRT(zRand);
        }

        return tree;
    }


    void SearchTree::setParent(NodeT node, NodeT parent, Steer edgeTraj, double nodeCost) {
        node->value.traj = edgeTraj;
        node->value.cost = nodeCost;
        node->parent = parent;
    }

//        def addNodeRRT(T, xInit, K, eta):
//            xRand = sampleFree()
//            xNearest = nearest(xRand, T)
//            xNew, u = steer(xNearest, xRand, eta)
//            if obstacleFree(xNearest, xNew):
//            T.add_vertex(xNew)
//            T.add_edge(xNearest, xNew, u)
    bool SearchTree::extendRRT(StateT z) {
        auto zNearest = nearest(z);
        auto xNew = steer(zNearest->value.state, z);
        auto zNew = tree.makeNode(SearchNode {xNew(xNew.t), 0, xNew});

        if (!obstacleFree(xNew)) {
            return false;
        }

        tree.nodes.push_back(zNew);
//            zNew->parent = zNearest;
        setParent(zNew, zNearest, xNew, zNearest->value.cost + J(xNew));

        return true;
    }

    SearchTree SearchTree::fromRRTs(cairo_t *cr, StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles,
                                    std::function<void(const SearchTree &, StateT, bool)> callback) {
        auto tree = SearchTree(init, goal);

        tree.cairo = cr; // TODO: Fix this.

        tree.obstacles = obstacles;


        for (int i = 0; i < n; i++) {
            StateT zRand = sample();
            bool extended = tree.extendRRTs(cr, zRand);

            callback(tree, zRand, extended);
        }

        return tree;
    }

// Note:
// 'z' denotes a node, which has a parent, and contains
// a vertex, a control, and a time for which to follow the control.
    bool SearchTree::extendRRTs(cairo_t *cr, StateT z) {
        auto numVertices = tree.nodes.size();

        auto zNearest = nearest(z);
        auto xNew = steer(zNearest->value.state, z);
        auto zNew = tree.makeNode(SearchNode {xNew(xNew.t), cost(zNearest->value) + J(xNew), xNew});

        if (!obstacleFree(xNew)) {
            return false;
        }

        tree.nodes.push_back(zNew);

        drawCircle(cr, zNearest->value.state, 0.035, {1, 0.5, 0}, 1);

        auto zMin = zNearest;
        auto cMin = cost(zNew->value);
        Steer xMin = xNew;

        auto zNearby = nearVertices(zNew, numVertices);
        for (auto &zNear : zNearby) {
            auto xNear = steer(zNear->value.state, zNew->value.state);

            { // debug
                drawCircle(cr, zNear->value.state, 0.03, {0, 1, 0}, 0.2);
                arma::vec2 mid = (zNear->value.state + zNew->value.state) * 0.5;
                mid[1] -= 0.01;
                shared::utility::drawing::drawString(cr, mid, 0.01, J(xNear), ", ", xNear.reachesTarget, ", ", obstacleFree(xNear));
            }
            if (xNear.reachesTarget && obstacleFree(xNear)) { /*&& xNear(xNear.t) == zNew*/
                double zNearCost = cost(zNear->value) + J(xNear);
                if (zNearCost < cMin) {
                    cMin = zNearCost;
                    zMin = zNear;
                    xMin = xNear;
                }


            }
        }

        drawCircle(cr, zMin->value.state, 0.03, {0, 1, 0});
        setParent(zNew, zMin, xMin, cMin);

        for (auto &zNear : zNearby) {
            if (zNear == zMin) {
                continue;
            }

            auto xNear = steer(zNew->value.state, zNear->value.state);
            double zNearCost = cost(zNew->value) + J(xNear);

            if (xNear.reachesTarget && obstacleFree(xNear) &&
                cost(zNear->value) > zNearCost) { /*xNear(xNear.t) == zNear &&*/
                setParent(zNear, zNew, xNear, zNearCost);
            }

        }

        return true;
    }

}