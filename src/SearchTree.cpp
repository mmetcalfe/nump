//
// Created by Mitchell Metcalfe on 16/08/15.
//

//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include "SearchTree.h"

#include <armadillo>
#include "math/geometry.h"
#include "Tree.h"
#include "shared/utility/drawing/cairo_drawing.h"

using utility::drawing::fillCircle;

namespace nump {

    typedef SearchTree::StateT StateT;
    typedef SearchTree::NodeT NodeT;
    typedef SearchTree::TrajT TrajT;

    // SearchTree::SearchTree(StateT init, StateT goal)
    //         : tree(SearchNode {init, 0, TrajT()}), init(init), goal(goal) {
    // }

    SearchTree::SearchTree(const numptest::SearchScenario::Config& config)
            : scenario(config), tree(SearchNode {config.initialState, 0, TrajT()}) {
        // auto& initNode = graph.nodes.front();
        // auto belief = std::make_shared<BeliefNode>(initNode);
        // belief->stateCov = config.initialCovariance;
        // initNode->value.beliefNodes.push_back(belief);
        // initialBelief = belief;
    }


    // StateT SearchTree::initialState() const {
    //     return init;
    // }
    //
    // StateT SearchTree::goalState() const {
    //     return goal;
    // }

    inline double SearchTree::distance(StateT a, StateT b) {
//        return arma::norm(a - b);
//        return J(steer(a, b)); // TODO: Implement a faster, valid, distance metric.
        return arma::norm(a.position.rows(0,1) - b.position.rows(0,1));
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

    TrajT SearchTree::steer(StateT from, StateT to) {
        return TrajT::fromEndpoints(from, to);
//        TrajT s;
//
//        StateT diff = to - from;
//        double l = arma::norm(diff);
//
//        s.xInit = from;
//        s.xGoal = to;
//        s.u = arma::normalise(diff);
//
//        double maxDist = 1000;
////            s.t = std::min(l, maxDist);
//        if (l < maxDist) {
//            s.t = l;
//            s.reachesTarget = true;
//        } else {
//            s.t = maxDist;
//            s.reachesTarget = false;
//        }
//
//        return s;
    }

    std::vector<NodeT> SearchTree::nearVertices(NodeT queryNode, unsigned long numVertices) const {
        std::vector<NodeT> near;

        double gammaRRTs = 1;
        double cardV = numVertices;
        double threshold = gammaRRTs * pow(log(cardV) / cardV, 1.0 / 2);

//        cairo_arc(cairo, queryNode->value.state(0), queryNode->value.state(1), threshold, -M_PI, M_PI);
//        cairo_set_source_rgba(cairo, 1, 0.7, 1, 1);
//        cairo_set_line_width(cairo, 0.02);
//        cairo_stroke_preserve(cairo);
//        cairo_set_source_rgba(cairo, 1, 0.7, 1, 0.5);
//        cairo_fill(cairo);

        for (auto &node : tree.nodes) {
//            if (node == queryNode) {
//                continue;
//            }

            if (distance(node->value.state, queryNode->value.state) < threshold) {
                near.push_back(node);
            }
        }

        return near;
    }

    bool SearchTree::obstacleFree(TrajT traj) const {
        // int numSteps = 100;
        // for (int i = 0; i <= numSteps; i++) {
        double timeStep = 0.02;
        for (auto walker = traj.walk(timeStep); !walker.isFinished(); ) {
            // double t = i / double(numSteps);
            // StateT x = s(t*s.t);

            walker.stepBy();
            RobotModel::State xTraj = walker.currentState();

////            fillCircle(cairo, {x(0), x(1)}, 0.001, {1, 1, 1}, 1);
//            cairo_set_source_rgba(cairo, 0, 0, 0, 0.5);
//            utility::drawing::drawRobot(cairo, x, 0.01);

            for (auto &obs : scenario.obstacles) {
                if (obs.contains(xTraj.position.xy())) {
//                    fillCircle(cairo, pos, 0.003, {0, 1, 1}, 1);

                    return false;
                }
            }
        }

        return true;
    }

    double SearchTree::J(TrajT s) {
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
//        template <class StateT>
    // SearchTree SearchTree::fromRRT(cairo_t *cr, StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles) {
    //     auto tree = SearchTree(init, goal);
    //
    //     tree.cairo = cr; // TODO: Fix this.
    //
    //     tree.obstacles = obstacles;
    //
    //     for (int i = 0; i < n; i++) {
    //         StateT zRand = TrajT::sample({1, 1});
    //         tree.extendRRT(zRand);
    //     }
    //
    //     return tree;
    // }


    void SearchTree::setParent(NodeT node, NodeT parent, TrajT edgeTraj, double nodeCost) const {
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

    // SearchTree SearchTree::fromRRTs(cairo_t *cr, StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles,
    //                                 std::function<void(const SearchTree &, StateT, bool)> callback) {
    SearchTree SearchTree::fromRRTs(
        const numptest::SearchScenario::Config& scenario,
        cairo_t *cr,
        // StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles,
        std::function<void(const SearchTree&, StateT, bool)> callback) {
        // auto tree = SearchTree(config.initialState, config.goalState);
        auto tree = SearchTree(scenario);
        tree.cairo = cr; // TODO: Fix this.

        for (int i = 0; i < scenario.numSamples; i++) {
            StateT zRand = TrajT::sample(scenario.mapSize);
            bool extended = tree.extendRRTs(cr, zRand);

            callback(tree, zRand, extended);
        }

        return tree;
    }


    NodeT SearchTree::createValidNodeForState(StateT state) const {
        auto zNearest = nearest(state);
        auto xNew = steer(zNearest->value.state, state);
        auto zNew = tree.makeNode(SearchNode {xNew(xNew.t), cost(zNearest->value) + J(xNew), xNew});
        zNew->parent = zNearest;

        if (!obstacleFree(xNew)) {
            return nullptr;
        }

        return zNew;
    }

    void SearchTree::optimiseParent(NodeT zNew, std::vector<NodeT> zNearby) const {
        // Find optimal parent:
        // NodeT findOptimalParent(NodeT zNew, std::vector<NodeT> zNearby) const;
        auto zMin = zNew->parent; // zNearest
        auto cMin = cost(zNew->value);
        TrajT xMin = zNew->value.traj;

        for (auto &zNear : zNearby) {
            auto xNear = steer(zNear->value.state, zNew->value.state);

            if (xNear.reachesTarget && obstacleFree(xNear)) { /*&& xNear(xNear.t) == zNew*/
                double zNearCost = cost(zNear->value) + J(xNear);
                if (zNearCost < cMin) {
                    cMin = zNearCost;
                    zMin = zNear;
                    xMin = xNear;
                }
            }
        }

        // Set parent to the optimal parent:
        setParent(zNew, zMin, xMin, cMin);
    }

    void SearchTree::optimiseNeighbours(NodeT zNew, std::vector<NodeT> zNearby) {
        for (auto &zNear : zNearby) {
            if (zNear == zNew->parent /*zMin*/) { // zMin is the optimal parent of zNew.
                continue;
            }

            auto xNear = steer(zNew->value.state, zNear->value.state);
            double zNearCost = cost(zNew->value) + J(xNear);

            if (xNear.reachesTarget && obstacleFree(xNear) &&
                cost(zNear->value) > zNearCost) { /*xNear(xNear.t) == zNear &&*/
                setParent(zNear, zNew, xNear, zNearCost);
            }
        }
    }

// Note:
// 'z' denotes a node, which has a parent, and contains
// a vertex, a control, and a time for which to follow the control.
    bool SearchTree::extendRRTs(cairo_t *cr, StateT z) {
        auto zNew = createValidNodeForState(z);
        if (!zNew) {
            return false;
        }

        auto zNearby = nearVertices(zNew, tree.nodes.size());

        // Note: Mutates zNew, but nothing else.
        optimiseParent(zNew, zNearby);

        tree.nodes.push_back(zNew);

        optimiseNeighbours(zNew, zNearby);

        return true;
    }

}
