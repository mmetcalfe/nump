//
// Created by Mitchell Metcalfe on 24/08/2015.
//

#include "RRBT.h"

#include <vector>
#include <algorithm> // remove and remove_if
#include <queue>
#include "shared/utility/drawing/SearchTreeDrawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"

using utility::math::geometry::Ellipse;

using shared::utility::drawing::fillCircle;

namespace nump {

    typedef RRBT::StateT StateT;
    typedef RRBT::NodeT NodeT;
    typedef RRBT::TrajT TrajT;
    typedef RRBT::BeliefNode BeliefNode;
    typedef RRBT::BeliefNodePtr BeliefNodePtr;


    RRBT::RRBT(StateT init, StateCovT initCov, StateT goal)
            : graph(Vertex {init, {}}), init(init), goal(goal) {
        auto& initNode = graph.nodes.front();

        auto belief = std::make_shared<BeliefNode>(initNode);
        belief->stateCov = initCov;
        initNode->value.beliefNodes.push_back(belief);
    }

    inline double RRBT::distance(StateT a, StateT b) {
//        return arma::norm(a - b);
//        return J(steer(a, b)); // TODO: Implement a faster, valid, distance metric.
        return arma::norm(a.rows(0,1) - b.rows(0,1));
    }

    NodeT RRBT::nearest(StateT state) const {

        auto min_it = std::min_element(graph.nodes.begin(), graph.nodes.end(), [state](NodeT a, NodeT b) {
            double distA = distance(a->value.state, state);
            double distB = distance(b->value.state, state);
            return distA < distB;
        });

        return *min_it;
    }

//    double RRBT::maxCost() const {
//
//        auto min_it = std::max_element(tree.nodes.begin(), tree.nodes.end(), [](NodeT a, NodeT b) {
//            return a->value.cost < b->value.cost;
//        });
//
//        return (*min_it)->value.cost;
//    }

    TrajT RRBT::steer(StateT from, StateT to) {
        return TrajT::fromEndpoints(from, to);
    }

    std::vector<NodeT> RRBT::nearVertices(NodeT queryNode, unsigned long numVertices) const {
        std::vector<NodeT> near;

        double gammaRRTs = 1;
        double cardV = numVertices;
        double threshold = gammaRRTs * pow(log(cardV) / cardV, 1.0 / 2);

        for (auto &node : graph.nodes) {
//            if (node == queryNode) {
//                continue;
//            }

            if (distance(node->value.state, queryNode->value.state) < threshold) {
                near.push_back(node);
            }
        }

        return near;
    }

    bool RRBT::obstacleFree(TrajT s) const {
        int numSteps = 100;
        for (int i = 0; i <= numSteps; i++) {
            double t = i / double(numSteps);
            StateT x = s(t*s.t);

            arma::vec2 pos = {x(0), x(1)};

            for (auto &obs : obstacles) {
                if (obs.contains(pos)) {
                    return false;
                }
            }
        }

        return true;
    }

    double RRBT::J(TrajT s) {
        return s.t;
    }

    TrajT RRBT::connect(StateT from, StateT to) {
        return steer(from, to);
    }

    bool anyContain(const std::vector<nump::math::Circle>& regions, StateT pos) {
        for (auto &reg : regions) {
            if (reg.contains(pos)) {
                return true;
            }
        }

        return false;
    }

    RRBT RRBT::fromRRBT(cairo_t *cr, StateT init, StateCovT initCov, StateT goal, int n,
                        std::vector<nump::math::Circle> obstacles,
                        std::vector<nump::math::Circle> measurementRegions,
                        std::function<void(const RRBT &, StateT, bool)> callback) {

        // Set initial state and covariance, set stateDistribCov to 0, and cost to 0.
        auto tree = RRBT(init, initCov, goal);
        tree.cairo = cr; // TODO: Fix this.
        tree.obstacles = obstacles;
        tree.measurementRegions = measurementRegions;

        for (int i = 0; i < n; i++) {
            StateT xRand = TrajT::sample();
            bool extended = tree.extendRRBT(cr, xRand);

            callback(tree, xRand, extended);
        }

        return tree;
    }

    bool RRBT::satisfiesChanceConstraint(StateT state, StateCovT stateCov, const std::vector<nump::math::Circle>& obstacles) {
        // TODO: Enhance the intersection test to use the ellipse, rather than its bounding rectangle.
        Ellipse confEllipse = Ellipse::forConfidenceRegion(state, stateCov);
        for (auto& obs : obstacles) {
            // TODO: Enhance test to work for a polygonal robot footprint, rather than just a point robot.
            bool intersects = utility::math::geometry::intersection::test(obs, confEllipse);

            // Return failure if the chance constraint is violated:
            if (intersects) {
                return false;
            }
        }

        return true;
    }

    BeliefNodePtr RRBT::propagate(
            const TrajT& traj,
            BeliefNodePtr belief,
            const std::vector<nump::math::Circle>& obstacles,
            const std::vector<nump::math::Circle>& measurementRegions,
            std::function<void(double, StateT, BeliefNodePtr)> callback
    ) {
        // Note: Notation transliteration conventions:
        // LaTeX  -->  C++
        //  t-1  -->  p
        // e.g. Λ_{t-1}  -->  Λp
        // \bar{}  -->  b
        // e.g. \bar{Σ}_{t}  -->  Σbt

        RRBT::StateCovT Σp = belief->stateCov;
        RRBT::StateCovT Λp = belief->stateDistribCov;

        RRBT::StateCovT Σt, Λt; // declare ouptuts

        // Iterate over trajectory:
        // TODO: Make trajectory iteration efficient.
        // TODO: Determine how to change matrices per timestep so that timesteps do not have to equal in time.
        int numSteps = traj.t / 0.01; // 100
        for (int i = 0; i <= numSteps; i++) {
            double t = (i / double(numSteps)) * traj.t;

            RRBT::StateT xTraj = traj(t);

            // Inputs: Qt, Rt, Σp, Λp.

            // TODO: Determine At, Bt, Ct, and Kt by linearising the robot's dynamics at the current point.
            RRBT::StateCovT At, Bt, Ct, Kt, Qt, Rt;
            // TODO: Have the trajectory use a model to determine Qt (the motion uncertainty).
            // TODO: Use a sensor model to determine Ct and Rt.

            /*
             * Note: Sensor and movement error model is:
             *
             *     x~t = At*x~p + Bt*u~p + wt,    wt ~ N(0, Qt)
             *     z~t = Ct*x~t + vt,             vt ~ N(0, Rt)
             *
             * (where \tilde{x}  -->  x~)
             *
             */

            At.eye();

            Bt.eye();

            Kt.eye();
            Kt(0,0) = 0.5;
            Kt(1,1) = 0.5;

            Ct.eye();

            Qt.eye();
            Qt(0,0) = 0.0001;
            Qt(1,1) = 0.0001;

            Rt.eye();
            Rt(0,0) = 1e7;
            Rt(1,1) = 1e7;
            if (anyContain(measurementRegions, xTraj)) {
                Rt(0,0) = 0.001;
                Rt(1,1) = 0.001;
            }

            // Step 1 - Covariance prediction (equations 21, 33):
            // Kalman filter process step:
            RRBT::StateCovT Σbt = At*Σp*At.t() + Qt; // (equation 17)

            // Kalman filter measurement update:
            RRBT::StateCovT St = Ct*Σbt*Ct.t() + Rt; // (equation 18)
            RRBT::StateCovT Lt = Σbt*Ct.t()*St.i(); // (equation 19)
            Σt = Σbt - Lt*Ct*Σbt; // (equation 21)

            // Distribution over state estimates:
            RRBT::StateCovT Ak = (At - Bt*Kt); // A_{K}
            Λt = Ak*Λp*Ak.t() + Lt*Ct*Σbt; // (equation 33)


            // Distribution over trajectories (at current timestep):
            // x_t ~ N(\check{x}, Λ_{t} + Σ_{t})
            // i.e. x_t ~ N(xTraj, xTrajCov)
            // (where x_t is the true state)
            RRBT::StateCovT xTrajCov = Σt + Λt;

            // Step 2 - Cost expectation evaluation (equation 11):
            // TODO: Work out how to evaluate expected path cost.


            { // DEBUG
                // Callback for debug drawing:
                auto dbgBelief = std::make_shared<BeliefNode>(std::weak_ptr<GraphT::Node>());
                dbgBelief->parent = belief;
                dbgBelief->stateCov = Σt;
                dbgBelief->stateDistribCov = Λt;
                dbgBelief->cost = belief->cost; // + J(traj); // TODO: Implement cost function for partial trajectories.
                callback(t, xTraj, dbgBelief);
            }


            // Step 3 - Chance-constraint checking (equation 13):
            //   P(x_t \in \mathcal{X}_obs) < \delta, \forall t \in [0, T]
            if (!satisfiesChanceConstraint(xTraj, xTrajCov, obstacles)) {
                return nullptr;
            }

            // Update previous values:
            Σp = Σt;
            Λp = Λt;
        }

        // Construct a belief node with the resultant distribution:
        // TODO: Get node from the edge:
        auto newBelief = std::make_shared<BeliefNode>(std::weak_ptr<GraphT::Node>()); // null weak pointer
        newBelief->parent = belief;

        // Assign calculated distributions and cost:
        newBelief->stateCov = Σt;
        newBelief->stateDistribCov = Λt;
        newBelief->cost = belief->cost + J(traj);

        return newBelief;
    }

    // TODO: Make positive definite test more efficient.
    // TODO: Verify method of covariance comparison, + describe in report.
    bool is_positive_definite(const RRBT::StateCovT& X) {
        arma::vec eigval = arma::eig_sym(X);

        for (int i = 0; i < eigval.n_elem; i++) {
            if (eigval(i) <= 0) {
                return false;
            }
        }

        return true;
    }

    // TODO: Check covariance comparison.
    bool compareCovariancesLT(const RRBT::StateCovT& covA, const RRBT::StateCovT& covB) {
        return is_positive_definite(covB - covA);
    }

    bool RRBT::BeliefNode::dominates(BeliefNodePtr belief, double tolerance) {
        // Calculate tolerance matrix:
        RRBT::StateCovT eps;
        eps.eye();
        eps *= tolerance;

        // Perform comparisons:
        bool stateCovDom = compareCovariancesLT(stateCov, belief->stateCov + eps);
        bool stateDistribCovDom = compareCovariancesLT(stateDistribCov, belief->stateDistribCov + eps);
        bool costDom = cost < belief->cost;

        return stateCovDom && stateDistribCovDom && costDom;
    }

    bool RRBT::appendBelief(NodeT node, BeliefNodePtr belief) {

        double tolerance = 0.1; // TODO: Add to config.

        auto& beliefNodes = node->value.beliefNodes;

        // If n is dominated by any nodes in v.N, return failure.
        for (auto& cmpBelief : beliefNodes) {
            if (cmpBelief->dominates(belief, tolerance)) {
                return false;
            }
        }

        // Remove all nodes dominated by n from v.N.
        // TODO: When do we prune edges from the graph?
        beliefNodes.erase(std::remove_if(
                beliefNodes.begin(),
                beliefNodes.end(),
                [belief](BeliefNodePtr n) {
                    return belief->dominates(n);
                }),
                beliefNodes.end());

        // Add n to v.N.
        beliefNodes.push_back(belief);

        return true;
    }

    bool RRBT::extendRRBT(cairo_t *cr, StateT xRand) {

        NodeT vNearest = nearest(xRand);
        TrajT eNearestRand = connect(vNearest->value.state, xRand);

        // If eNearest can not be traversed by any belief node in vNearest without
        // violating the chance constraint, then return failure.
        BeliefNodePtr nRand = nullptr;
        for (auto& node : vNearest->value.beliefNodes) {
            nRand = propagate(eNearestRand, node, obstacles, measurementRegions);
            if (nRand != nullptr) {
                break;
            }
        }
        if (nRand == nullptr) {
            return false;
        }

        // TODO: Check whether this needs a belief node.
        auto vRand = graph.makeNode(Vertex {xRand, {nRand}});
        graph.nodes.push_back(vRand);

        TrajT eRandNearest = connect(vRand->value.state, vNearest->value.state);

        // Add eNearestRand and eRandNearest to the edge set.
        graph.addEdge(vNearest, vRand, eNearestRand);
        graph.addEdge(vRand, vNearest, eRandNearest);

        // TODO: Just use BeliefNode::containingNode instead of a std::pair.
        std::queue<std::pair<BeliefNodePtr, NodeT>> beliefNodeQ;

        // Add all of vNearest's beliefNodes to the queue:
        for (auto& node : vNearest->value.beliefNodes) {
            beliefNodeQ.push({node, vNearest});
        }

        // Add all nearby nodes and edges for consideration:
        auto vNearby = nearVertices(vRand, graph.nodes.size());
        for (auto& vNear : vNearby) {
            TrajT eNearRand = connect(vNear->value.state, vRand->value.state);
            TrajT eRandNear = connect(vRand->value.state, vNear->value.state);

            // Add eNearRand and eRandNear to the edge set.
            graph.addEdge(vNear, vRand, eNearRand);
            graph.addEdge(vRand, vNear, eRandNear);

            // Add all of vNear's beliefNodes to the queue:
            for (auto& node : vNear->value.beliefNodes) {
                beliefNodeQ.push({node, vNear});
            }
        }

        // Consider all nearby nodes, pruning dominated paths:
        // (exhaustively search queue using uniform cost search)
        while (!beliefNodeQ.empty()) {
            BeliefNodePtr nBelief; // The belief node.
            NodeT vBelief; // The vertex containing belief.
            std::tie(nBelief, vBelief) = beliefNodeQ.front();
            beliefNodeQ.pop();

            for (auto& eNeighbour : graph.outgoing(vBelief)) {
                auto vNeighbour = eNeighbour.child.lock();

                auto nNew = propagate(eNeighbour.value, nBelief, obstacles, measurementRegions);

                if (appendBelief(vNeighbour, nNew)) {
                    beliefNodeQ.push({nNew, vNeighbour});
                }
            }
        }

    }

}
