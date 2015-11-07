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
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "shared/utility/math/distributions.h"
#include "BipedRobotModel.h"

using utility::drawing::fillCircle;
using nump::BipedRobotModel;

namespace nump {

    typedef SearchTree::StateT StateT;
    typedef SearchTree::NodeT NodeT;
    typedef SearchTree::TrajT TrajT;

    // SearchTree::SearchTree(StateT init, StateT goal)
    //         : tree(SearchNode {init, 0, TrajT()}), init(init), goal(goal) {
    // }


    nump::Path<BipedRobotModel::State> SearchTree::getSolutionPath() const {
        nump::Path<BipedRobotModel::State> nominalPath;
        auto kickBoxes = BipedRobotModel::getLocalKickBoxes(scenario.kbConfig, scenario.ball.radius);
        double kickBoxLength = kickBoxes[0].size(0);
        double approachClearence = kickBoxLength * scenario.ballObstacleOffsetFactor;
        BipedRobotModel::State kickPos = BipedRobotModel::getIdealKickingPosition(scenario.ball, scenario.kbConfig, scenario.targetAngle, approachClearence);
        auto goalNode = createValidNodeForState(kickPos);

        if (goalNode) {
            auto zNearby = nearVertices(goalNode, tree.nodes.size());
            optimiseParent(goalNode, zNearby);
            for (auto pathNode = goalNode; pathNode != nullptr; pathNode = pathNode->parent) {
                nominalPath.segments.push_front(pathNode->value.traj);
            }
        }

        nominalPath.timeSpentPlanning = timeSpentPlanning;
        return nominalPath;
    }

    Circle SearchTree::getBallObstacle(const numptest::SearchScenario::Config& scenario) {
        // Create the ball obstacle:
        // auto kickBoxes = BipedRobotModel::getLocalKickBoxes(scenario.kbConfig, scenario.ball.radius);
        // double kickBoxLength = kickBoxes[0].size(0);
        // double obsRadius = scenario.ball.radius * scenario.ballObstacleRadiusFactor;
        // double obsOffset = kickBoxLength * scenario.ballObstacleOffsetFactor;
        // Transform2D ballTarget = {scenario.ball.centre, scenario.targetAngle};
        // double obsX = -scenario.ball.radius + obsRadius - obsOffset;
        // Transform2D obsPos = ballTarget.localToWorld({obsX, 0, 0});
        // Circle ballObstacle = {obsPos.xy(), obsRadius};
        // return ballObstacle;

        auto kickBoxes = BipedRobotModel::getLocalKickBoxes(scenario.kbConfig, scenario.ball.radius);
        double kickBoxLength = kickBoxes[0].size(0);
        double backClearence = scenario.ball.radius * scenario.ballObstacleRadiusFactor;
        double approachClearence = kickBoxLength * scenario.ballObstacleOffsetFactor;
        double obsRadius = scenario.ball.radius + 0.5*(backClearence + approachClearence);
        Transform2D ballTarget = {scenario.ball.centre, scenario.targetAngle};
        double obsX = -scenario.ball.radius + obsRadius - approachClearence;
        Transform2D obsPos = ballTarget.localToWorld({obsX, 0, 0});
        Circle ballObstacle = {obsPos.xy(), obsRadius};
        return ballObstacle;
    }

    SearchTree::SearchTree(const numptest::SearchScenario::Config& config)
            : tree(SearchNode {{config.initialState}, 0, TrajT()}), scenario(config) {
        // auto& initNode = graph.nodes.front();
        // auto belief = std::make_shared<BeliefNode>(initNode);
        // belief->stateCov = config.initialCovariance;
        // initNode->value.beliefNodes.push_back(belief);
        // initialBelief = belief;

        auto ballObstacle = getBallObstacle(scenario);
        scenario.obstacles.push_back(ballObstacle);
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
            RotatedRectangle robotFootprint = {xTraj.position, scenario.footprintSize};
            for (auto &obs : scenario.obstacles) {
                bool intersects = utility::math::geometry::intersection::test(obs, robotFootprint);
                if (intersects) {
                // if (obs.contains(xTraj.position.xy())) {
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

        double feasibleSolutionTime = -1;
        double lastSolutionImprovementTime = 0;
        double bestSolutionCost = arma::datum::inf;
        double lastIdealSolutionAttemptTime = 0;
        int lastIdealSolutionAttemptIteration = 0;

        arma::wall_clock searchTimer;
        searchTimer.tic();
        for (int i = 0; i < tree.scenario.numSamples; i++) {
            // Enforce the maximum time limit:
            double elapsedSeconds = searchTimer.toc();
            if (elapsedSeconds > scenario.searchTimeLimitSeconds) {
                std::cout << "SearchTree::fromRRT: Time limit reached after "
                          << elapsedSeconds
                          << " seconds and "
                          << i
                          << " iterations." << std::endl;
                break;
            }

            // Every second, if the ideal kicking position has not been added, add it:
            double timeSinceIdealPositionAttempt = elapsedSeconds - lastIdealSolutionAttemptTime;
            int iterationsSinceIdealPositionAttempt = i - lastIdealSolutionAttemptIteration;
            if (!tree.idealPositionAdded &&
                timeSinceIdealPositionAttempt > 1 &&
                iterationsSinceIdealPositionAttempt > 5) {
                lastIdealSolutionAttemptTime = elapsedSeconds;
                lastIdealSolutionAttemptIteration = i;
                BipedRobotModel::State kickPos = BipedRobotModel::getIdealKickingPosition(tree.scenario.ball, tree.scenario.kbConfig, tree.scenario.targetAngle);
                tree.idealPositionAdded = tree.extendRRTs(cr, kickPos);
                if (tree.idealPositionAdded) {
                    std::cout << "SearchTree::fromRRT: Ideal kicking position added ";
                } else {
                    std::cout << "SearchTree::fromRRT: Failed to add ideal kicking position ";
                }
                std::cout << "at time " << elapsedSeconds << " after " << i << " iterations." << std::endl;
            }

            // If a feasible path has been found:
            auto bgn = tree.findBestGoalState();
            if (bgn) {
                if (feasibleSolutionTime < 0) {
                    // Set the time that the first feasible solution was found.
                    feasibleSolutionTime = elapsedSeconds;
                    std::cout << "SearchTree::fromRRT: Feasible solution found at time " << elapsedSeconds << "." << std::endl;
                }

                double timeSinceFeasible = elapsedSeconds - feasibleSolutionTime;
                if (timeSinceFeasible > scenario.rrtsSearchTimeLimitAfterFeasible) {
                    std::cout << "SearchTree::fromRRT: Feasible time limit reached after "
                              << elapsedSeconds
                              << " seconds, "
                              << timeSinceFeasible
                              << " seconds after the first feasible solution was found, and "
                              << i
                              << " iterations." << std::endl;
                    break;
                }

                // Check for improvement:
                if (bgn->value.cost < bestSolutionCost) {
                    std::cout << "SearchTree::fromRRT: Feasible solution improved at time " << elapsedSeconds << "." << std::endl;
                    bestSolutionCost = bgn->value.cost;
                    lastSolutionImprovementTime = elapsedSeconds;
                } else {
                    double timeSinceImprovement = elapsedSeconds - lastSolutionImprovementTime;
                    if (timeSinceImprovement > scenario.rrtsMaxTimeWithoutImprovement) {
                        std::cout << "SearchTree::fromRRT: Feasible time limit reached after "
                                  << elapsedSeconds
                                  << " seconds, "
                                  << timeSinceImprovement
                                  << " seconds since the last solution improvement was made, and "
                                  << i
                                  << " iterations." << std::endl;
                        break;
                    }
                }
            }

            // StateT zRand = TrajT::sample(tree.scenario.mapSize);
            // Normal distribution:
            double sampleVar = 0.2;
            arma::mat33 sampleCov = {
                { sampleVar, 0, 0 },
                { 0, sampleVar, 0 },
                { 0, 0, 100 }
            };
            arma::mat sample = utility::math::distributions::randn(1, Transform2D{tree.scenario.ball.centre, tree.scenario.targetAngle}, sampleCov);
            StateT zRand = {sample};

            bool extended = tree.extendRRTs(cr, zRand);

            callback(tree, zRand, extended);
        }

        tree.timeSpentPlanning = searchTimer.toc();
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

        // If the new node is a goal node, add it to the set of goal nodes:
        if (RobotModel::canAlmostKickBallAtTarget(
                {zNew->value.state.position, scenario.footprintSize},
                scenario.ball,
                scenario.kbConfig,
                scenario.targetAngle,
                scenario.targetAngleRange)) {
            goalVertices.push_back(zNew);
        }

        return true;
    }

    std::shared_ptr<SearchTree::TreeT::Node> SearchTree::findBestGoalState() const {
        std::shared_ptr<SearchTree::TreeT::Node> bestNode = nullptr;
        double bestCost = arma::datum::inf;

        for (auto weakGoalVertex : goalVertices) {
            auto goalNode = weakGoalVertex.lock();
            if (goalNode->value.cost < bestCost) {
                bestNode = goalNode;
                bestCost = goalNode->value.cost;
            }
        }

        return bestNode;
    }

}
