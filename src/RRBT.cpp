//
// Created by Mitchell Metcalfe on 24/08/2015.
//

#include "RRBT.h"

#include <vector>
#include <algorithm> // remove and remove_if
#include <queue>
#include "shared/utility/drawing/cairo_drawing.h"
#include "shared/utility/math/geometry/Ellipse.h"
#include "shared/utility/math/geometry/intersection/Intersection.h"
#include "shared/utility/math/angle.h"
#include "shared/utility/math/distributions.h"
#include "BipedRobotModel.h"

using utility::math::geometry::Ellipse;

using utility::drawing::fillCircle;

namespace nump {

    typedef RRBT::StateT StateT;
    typedef RRBT::NodeT NodeT;
    typedef RRBT::TrajT TrajT;
    typedef RRBT::BeliefNode BeliefNode;
    typedef RRBT::BeliefNodePtr BeliefNodePtr;


    RRBT::RRBT(const numptest::SearchScenario::Config& config)
            : scenario(config), graph(Vertex {config.initialState, {}}) {

        auto& initNode = graph.nodes.front();

        auto belief = std::make_shared<BeliefNode>(initNode);
        belief->stateCov = config.initialCovariance;
        initNode->value.beliefNodes.push_back(belief);
        initialBelief = belief;
    }
//    RRBT::RRBT(StateT init, StateCovT initCov, StateT goal)
//            : graph(Vertex {init, {}}), init(init), goal(goal) {
//        auto& initNode = graph.nodes.front();
//
//        auto belief = std::make_shared<BeliefNode>(initNode);
//        belief->stateCov = initCov;
//        initNode->value.beliefNodes.push_back(belief);
//        initialBelief = belief;
//    }

    inline double RRBT::distance(StateT a, StateT b) {
//        return arma::norm(a - b);
//        return J(steer(a, b)); // TODO: Implement a faster, valid, distance metric.
        return arma::norm(a.position.rows(0,1) - b.position.rows(0,1));
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
            if (node == queryNode) {
                continue;
            }

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

            arma::vec2 pos = {x.position(0), x.position(1)};

            for (auto &obs : scenario.obstacles) {
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


//    RRBT RRBT::fromRRBT(cairo_t *cr, StateT init, StateCovT initCov, StateT goal, int n,
//                        std::vector<nump::math::Circle> obstacles,
//                        std::vector<nump::math::Circle> measurementRegions,
//                        std::function<void(const RRBT &, StateT, bool)> callback) {
     RRBT RRBT::fromSearchScenario(const numptest::SearchScenario::Config& scenario,
            cairo_t *cr,
            std::function<void(const RRBT &, StateT, bool)> callback) {

        // Set initial state and covariance, set stateDistribCov to 0, and cost to 0.
//        auto tree = RRBT(scenario.init, scenario.initCov, scenario.goal);
        auto tree = RRBT(scenario);
        tree.cairo = cr; // TODO: Fix this.

        arma::wall_clock searchTimer;
        searchTimer.tic();
        for (int i = 0; i < scenario.numSamples; i++) {
            // Enforce the time limit:
            double elapsedSeconds = searchTimer.toc();
            if (elapsedSeconds > scenario.searchTimeLimitSeconds) {
                std::cout << "RRBT::fromSearchScenario: Time limit reached after "
                          << elapsedSeconds
                          << " seconds and "
                          << i
                          << " iterations." << std::endl;
                break;
            }

            arma::wall_clock timer;
            timer.tic();

            // Random sampling:
            StateT xRand = TrajT::sample(scenario.mapSize);

            // // Grid sampling:
            // int sqrtNum = int(std::sqrt(scenario.numSamples));
            // double ix = i % sqrtNum;
            // double iy = i / sqrtNum;
            // arma::vec2 samplePos = (arma::vec2({iy, ix}) / sqrtNum) % scenario.mapSize;
            // StateT xRand = { Transform2D::lookAt(samplePos, scenario.ball.centre) };

            bool extended = tree.extendRRBT(cr, xRand);

            double seconds = timer.toc();
            if (extended) {
                tree.iterationTimes.push_back(seconds);
            }

            callback(tree, xRand, extended);
        }

        return tree;
    }

    bool satisfiesChanceConstraintPointFootprint(RRBT::StateT mean, RRBT::StateCovT cov,
                                   const std::vector<nump::math::Circle>& obstacles) {
        Ellipse confEllipse = utility::math::distributions::confidenceRegion(mean.position.head(2), cov.submat(0,0,1,1), 0.95);
        for (auto& obs : obstacles) {
            bool intersects = utility::math::geometry::intersection::test(obs, confEllipse);

            // Return failure if the chance constraint is violated:
            if (intersects) {
                return false;
            }
        }

        return true;
    }

    bool satisfiesChanceConstraintTransform2D(Transform2D mean, arma::mat33 cov, arma::vec2 footprintSize,
                                   const std::vector<nump::math::Circle>& obstacles) {
        Ellipse confEllipse = utility::math::distributions::confidenceRegion(mean.head(2), cov.submat(0,0,1,1), 0.95, arma::size(mean)(0));
        for (auto& obs : obstacles) {
            bool intersects = utility::math::geometry::intersection::test(obs, confEllipse);

            // Return failure if the chance constraint is violated:
            if (intersects) {
                return false;
            }
        }

        return true;
    }

    bool RRBT::satisfiesChanceConstraint(StateT state, StateCovT stateCov, arma::vec2 footprintSize,
                                         const std::vector<nump::math::Circle>& obstacles) {
        if (!arma::is_finite(stateCov)) {
            std::cout << "satisfiesChanceConstraint: is_finite fail." << std::endl;
            return false;
        } else if (std::abs(stateCov(0,0)) > 1e20 || std::abs(stateCov(0,1)) > 1e20) {
            std::cout << "satisfiesChanceConstraint: std::abs(stateCov(x,y)) > 1e20 fail." << std::endl;
            return false;
        }

        arma::vec size = arma::vec(arma::size(state.position));
        if (size(0) == 2) {
            // TODO: Enhance test to work for a polygonal robot footprint, rather than just a point robot.
            return satisfiesChanceConstraintPointFootprint(state, stateCov, obstacles);
        } else {
            return satisfiesChanceConstraintTransform2D(state.position, stateCov, footprintSize, obstacles);
        }
    }

    BeliefNodePtr RRBT::propagate(
            const TrajT& traj,
            BeliefNodePtr belief,
            arma::vec2 footprintSize,
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

        RobotModel::MotionCov Σp = belief->stateCov;
        RobotModel::MotionCov Λp = belief->stateDistribCov;

        RobotModel::MotionCov Σt, Λt; // declare ouptuts

        // Iterate over trajectory:
        // TODO: Make trajectory iteration efficient.
        // TODO: Determine how to change matrices per timestep so that timesteps do not have to equal in time.
        double timeStep = 0.2;
        for (auto walker = traj.walk(timeStep); !walker.isFinished(); ) {
            // double t = (i / double(numSteps)) * traj.t;
            // RobotModel::State xTraj = traj(t);

            walker.stepBy();
            RobotModel::State xTraj = walker.currentState();
            RobotModel::Control controlTraj = walker.currentControl();

            /*/
             * Note: Sensor and movement error model is:
             *
             *     x~t = At*x~p + Bt*u~p + wt,    wt ~ N(0, Qt)
             *     z~t = Ct*x~t + vt,             vt ~ N(0, Rt)
             *
             * (where \tilde{x}  -->  x~)
             *
            /*/

            // TODO: Fix redundant calculation between these matrixes:
            RobotModel::MotionMatrix At = RobotModel::motionErrorJacobian(timeStep, xTraj, controlTraj);
            RobotModel::ControlMatrix Bt = RobotModel::controlErrorJacobian(timeStep, xTraj, controlTraj);
            RobotModel::MeasurementMatrix Ct = RobotModel::measurementErrorJacobian(timeStep, xTraj, measurementRegions);
            RobotModel::MotionCov Qt = RobotModel::motionNoiseCovariance(timeStep, xTraj, controlTraj, Bt);
            RobotModel::MeasurementCov Rt = RobotModel::measurementNoiseCovariance(timeStep, xTraj, measurementRegions);
            RobotModel::RegulatorMatrix Kt = RobotModel::regulatorMatrix(timeStep, At, Bt);

            // Step 1 - Covariance prediction (equations 21, 33):
            // Kalman filter process step:
            RobotModel::MotionCov Σbt = At*Σp*At.t() + Qt; // (equation 17)

            // Kalman filter measurement update:
            RobotModel::MeasurementCov St = Ct*Σbt*Ct.t() + Rt; // (equation 18)
            RobotModel::KalmanGainMatrix Lt = Σbt*Ct.t()*St.i(); // (equation 19)
            Σt = Σbt - Lt*Ct*Σbt; // (equation 21)

            // Distribution over state estimates:
            RobotModel::MotionMatrix Ak = (At - Bt*Kt); // A_{K}
            Λt = Ak*Λp*Ak.t() + Lt*Ct*Σbt; // (equation 33)

            // std::cout << "controlTraj" << controlTraj.t() << std::endl;
            // std::cout << "Qt: " << Qt << std::endl;
            // std::cout << "Rt: " << Rt << std::endl;
            // std::cout << "At: " << At << std::endl;
            // std::cout << "Bt: " << Bt << std::endl;
            // std::cout << "Ct: " << Ct << std::endl;
            // std::cout << "Kt: " << Kt << std::endl;
            // std::cout << "Bt*Kt: " << (Bt*Kt) << std::endl;
            // std::cout << "Ak: " << Ak << std::endl;
            // std::cout << "Ak*Λp*Ak.t(): " << Ak*Λp*Ak.t() << std::endl;
            // std::cout << "Lt*Ct*Σbt: " << Lt*Ct*Σbt << std::endl;
            // std::cout << "Σt: " << Σt << std::endl;
            // std::cout << "Λt: " << Λt << std::endl;


            // Distribution over trajectories (at current time step):
            // x_t ~ N(\check{x}, Λ_{t} + Σ_{t})
            // i.e. x_t ~ N(xTraj, xTrajCov)
            // (where x_t is the true state)

//            double minVariance = 1e-8;
//            Σt(0,0) = std::max(Σt(0,0), minVariance);
//            Σt(1,1) = std::max(Σt(1,1), minVariance);
//            Σt(2,2) = std::max(Σt(2,2), minVariance);
//            Λt(0,0) = std::max(Λt(0,0), minVariance);
//            Λt(1,1) = std::max(Λt(1,1), minVariance);
//            Λt(2,2) = std::max(Λt(2,2), minVariance);
            RobotModel::MotionMatrix xTrajCov = Σt + Λt;

            
//            utility::math::distributions::confidenceRegionArea(xTrajCov.submat(0,0,1,1), 0.95, 3);
            
            // Step 2 - Cost expectation evaluation (equation 11):
            // TODO: Work out how to evaluate expected path cost.


            { // DEBUG
                // Callback for debug drawing:
                auto dbgBelief = std::make_shared<BeliefNode>(std::weak_ptr<GraphT::Node>());
                dbgBelief->parent = belief;
                dbgBelief->stateCov = Σt;
                dbgBelief->stateDistribCov = Λt;
                dbgBelief->cost = belief->cost; // + J(traj); // TODO: Implement cost function for partial trajectories.
                callback(walker.currentTime(), xTraj, dbgBelief);
            }


            // Step 3 - Chance-constraint checking (equation 13):
            //   P(x_t \in \mathcal{X}_obs) < \delta, \forall t \in [0, T]
            if (!satisfiesChanceConstraint(xTraj, xTrajCov, footprintSize, obstacles)) {
                return nullptr;
            }

            // Update previous values:
            Σp = Σt;
            Λp = Λt;
        }

        if (Σt(1,1) < 6.9532e-100) {
            std::cout << Σt << std::endl;
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

   bool is_positive_definite(const RRBT::StateCovT& X) {
       arma::vec eigval = arma::eig_sym(X);
       // std::cout << __LINE__ << "  eigval: " << eigval << std::endl;

       for (int i = 0; i < eigval.n_elem; i++) {
           if (eigval(i) <= 0) {
               return false;
           }
       }

       return true;
   }

    // TODO: Check covariance comparison.
    bool RRBT::compareCovariancesLT(const RRBT::StateCovT& covA, const RRBT::StateCovT& covB) {
        // std::cout << __LINE__ << ": compareCovariancesLT" << std::endl;
        // std::cout << __LINE__ << ":     covA: " << covA << std::endl;
        // std::cout << __LINE__ << ":     covB: " << covB << std::endl;

        // return is_positive_definite(covB - covA);

        // Ellipse ellipseA = Ellipse::forConfidenceRegion({0,0}, covA);
        // Ellipse ellipseB = Ellipse::forConfidenceRegion({0,0}, covB);
        // arma::vec2 sizeA = ellipseA.getSize();
        // arma::vec2 sizeB = ellipseB.getSize();
        // return sizeA(0)*sizeA(1) < sizeB(0)*sizeB(1);

        // TODO: Enhance covariace comparision to include heading uncertainty.

        // // TODO: Consider the method from http://math.stackexchange.com/a/669115
        // // where X >= Y <==> all eigenvalues of X - Y are >= 0
        double areaA = utility::math::distributions::confidenceRegionArea(covA.submat(0,0,1,1), 0.95);
        double areaB = utility::math::distributions::confidenceRegionArea(covB.submat(0,0,1,1), 0.95);
        return areaA < areaB;
    }

    bool RRBT::BeliefNode::dominates(BeliefNodePtr belief, double tolerance, double costTolerance) {
        // Calculate tolerance matrix:
        RRBT::StateCovT eps;
        eps.eye();
        eps *= tolerance;

        // std::cout << __LINE__ << ": dominates:" << std::endl;
        // std::cout << __LINE__ << ":     tolerance: " << tolerance << std::endl;

        // Perform comparisons:
        bool stateCovDom = compareCovariancesLT(stateCov, belief->stateCov + eps);
        // std::cout << __LINE__ << ":     stateCovDom: " << stateCovDom << std::endl;

        bool stateDistribCovDom = compareCovariancesLT(stateDistribCov, belief->stateDistribCov + eps);
        // std::cout << __LINE__ << ":     stateDistribCovDom: " << stateDistribCovDom << std::endl;

        // TODO: Document tolerance.
        double effectiveCostTolerance = tolerance == 0 ? 0 : costTolerance;
        // bool costDom = cost < belief->cost * (1 + costTolerance);
        bool costDom = cost < belief->cost + effectiveCostTolerance;
        // std::cout << __LINE__ << ":     costDom: " << costDom << std::endl;

        return stateCovDom && stateDistribCovDom && costDom;
    }

    bool RRBT::appendBelief(NodeT node, BeliefNodePtr belief) {
        auto& beliefNodes = node->value.beliefNodes;

        // If n is dominated by any nodes in v.N, return failure.
        for (auto& cmpBelief : beliefNodes) {
            if (cmpBelief->dominates(belief, scenario.rrbtAppendRejectCovThreshold, scenario.rrbtAppendRejectCostThreshold)) {
                return false;
            }
        }

        // std::cout << __LINE__ << ": Prune nodes (" << beliefNodes.size() << ")" << std::endl;

        // Remove all nodes dominated by n from v.N.
        beliefNodes.erase(std::remove_if(
                beliefNodes.begin(),
                beliefNodes.end(),
                [belief,this](BeliefNodePtr n) {
                    return belief->dominates(n);
                }),
                beliefNodes.end());

        // std::cout << __LINE__ << ": Pruning complete (" << beliefNodes.size() << ")" << std::endl;

        // Add n to v.N.
        beliefNodes.push_back(belief);
        belief->containingNode = node;

        return true;
    }

    bool RRBT::extendRRBT(cairo_t *cr, StateT xRand) {
        // std::cout << __LINE__ << ": Nearest." << std::endl;

        NodeT vNearest = nearest(xRand);
        TrajT eNearestRand = connect(vNearest->value.state, xRand);

        // If eNearest can not be traversed by any belief node in vNearest without
        // violating the chance constraint, then return failure.
        BeliefNodePtr nRand = nullptr;
        for (auto& node : vNearest->value.beliefNodes) {
            nRand = propagate(eNearestRand, node, scenario.footprintSize, scenario.obstacles, scenario.measurementRegions);
            if (nRand != nullptr) {
                break;
            }
        }
        if (nRand == nullptr) {
            // std::cout << __LINE__ << ": Add failure." << std::endl;
            return false;
        }

        // TODO: Verify whether belief node should be added.
        // auto vRand = graph.makeNode(Vertex {xRand, {nRand}});
        auto vRand = graph.makeNode(Vertex {xRand, {}});
        nRand->containingNode = vRand; // Set containing vertex of the belief node.
        graph.nodes.push_back(vRand);

        // If the new node is a goal node, add it to the set of goal nodes:
        if (RobotModel::canAlmostKickBallAtTarget(
                {vRand->value.state.position, scenario.footprintSize},
                scenario.ball,
                scenario.kbConfig,
                scenario.targetAngle,
                scenario.targetAngleRange)) {
            goalVertices.push_back(vRand);
        }

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

        // std::cout << __LINE__ << ": Add nearby nodes and edges." << std::endl;
        // Add all nearby nodes and edges for consideration:
        auto vNearby = nearVertices(vRand, graph.nodes.size());
        for (auto& vNear : vNearby) {
            // Avoid duplicate edges:
            if (vNear == vNearest) {
                continue;
            }

            // std::cout << __LINE__ << ": Add node " << vNear << std::endl;

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


        // std::cout << __LINE__ << ": Consider nodes, and prune." << std::endl;
        // Consider all nearby nodes, pruning dominated paths:
        // (exhaustively search queue using uniform cost search)
        while (!beliefNodeQ.empty()) {
            // std::cout << __LINE__ << ": loop start Q size: " << beliefNodeQ.size()  << std::endl;
            BeliefNodePtr nBelief; // The belief node.
            NodeT vBelief; // The vertex containing belief.
            std::tie(nBelief, vBelief) = beliefNodeQ.front();
            beliefNodeQ.pop();

            for (auto& eNeighbour : graph.outgoing(vBelief)) {
                auto vNeighbour = eNeighbour.child.lock();

                // {
                //     cairo_set_source_rgb(cr, 1.0, 1.0, 0.9); cairo_paint_with_alpha (cr, 1);
                //     utility::drawing::drawRRBT(cr, *this);
                //     cairo_set_source_rgba(cr, 0, 0, 0, 0.5);
                //     utility::drawing::showText(cr, {0.5, 0.5}, 0.03, "loop start");
                //     cairo_stroke(cr);
                //
                //     cairo_set_line_width(cr, 0.01);
                //     cairo_set_source_rgba(cr, 1, 0, 0, 0.5);
                //     utility::drawing::drawCircle(cr, {vBelief->value.state, 0.1});
                //     cairo_stroke(cr);
                //     cairo_set_source_rgba(cr, 0, 0, 1, 0.5);
                //     utility::drawing::drawCircle(cr, {vNeighbour->value.state, 0.1});
                //     cairo_stroke(cr);
                //     cairo_show_page(cr);
                // }

                // std::cout << __LINE__ << ": propagate" << std::endl;
                auto nNew = propagate(eNeighbour.value, nBelief, scenario.footprintSize, scenario.obstacles, scenario.measurementRegions);

                // TODO: Confirm the necessity of this test. It is not present in the paper.
                if (nNew == nullptr) {
                    continue;
                }

                // std::cout << __LINE__ << ": appendBelief" << std::endl;
                if (appendBelief(vNeighbour, nNew)) {
                    beliefNodeQ.push({nNew, vNeighbour});
                }
            }

            // std::cout << __LINE__ << ": loop end Q size: " << beliefNodeQ.size()  << std::endl;
        }

        return true;
    }


    std::shared_ptr<BeliefNode> RRBT::findBestGoalStateWithSuccessThreshold() const {
        std::shared_ptr<BeliefNode> bestNode = nullptr;
        double bestCost = arma::datum::inf;
        double bestProb = 0;

        std::shared_ptr<BeliefNode> highSuccessNode = nullptr;
        double bestSuccessProb = 0;


        for (auto weakGoalVertex : goalVertices) {
            auto goalVertex = weakGoalVertex.lock();
            const auto& state = goalVertex->value.state;

            for (auto goalNode : goalVertex->value.beliefNodes) {
                double prob = RobotModel::kickSuccessProbability(
                    state.position,
                    goalNode->stateCov + goalNode->stateDistribCov,
                    scenario.kbConfig,
                    scenario.ball,
                    scenario.targetAngle,
                    scenario.targetAngleRange
                );

                std::cout << "prob: " << prob << std::endl;

                if (prob > bestSuccessProb) {
                    highSuccessNode = goalNode;
                    bestSuccessProb = prob;
                }

                if (prob < scenario.minKickProbability) {
                    continue;
                } else if (goalNode->cost < bestCost) {
                    bestNode = goalNode;
                    bestCost = goalNode->cost;
                    bestProb = prob;
                }
            }
        }

        if (!bestNode) {
            std::cout << "Success chance threshold not met: "
                      << bestSuccessProb << " < " << scenario.minKickProbability
                      << std::endl;
            bestNode = highSuccessNode;
        }

        std::cout << "bestProb: " << bestProb << std::endl;

        return bestNode;
    }

    nump::Path<BipedRobotModel::State> RRBT::getSolutionPath() const {
        auto goalNode = findBestGoalStateWithSuccessThreshold();

        nump::Path<BipedRobotModel::State> nominalPath;
        if (goalNode) {
            // auto zNearby = nearVertices(goalNode, tree.nodes.size());
            // optimiseParent(goalNode, zNearby);
            // for (auto pathNode = goalNode; pathNode != nullptr; pathNode = pathNode->parent) {
            //     nominalPath.segments.push_front(pathNode->value.traj);
            // }


            // auto n = goalNode;
            // int depth = 0;
            // for (auto pathNode = goalNode; pathNode != nullptr; pathNode = pathNode->parent) {
            for (auto pathNode = goalNode; pathNode != initialBelief; pathNode = pathNode->parent) {
            // while (n != rrbt.initialBelief) {

                auto currentVertex = pathNode->containingNode;
                auto parentVertex = pathNode->parent->containingNode;
                auto edge = graph.edgeBetween(parentVertex.lock(), currentVertex.lock());
                nominalPath.segments.push_front(edge->value);

                // if (!n->containingNode.lock()) {
                    // break;
                // }
                // cairoMoveTo(cr, n->containingNode.lock()->value.state.position.head(2));
                // cairoLineTo(cr, n->parent->containingNode.lock()->value.state.position.head(2));
                // cairo_set_line_width(cr, lwHighlight);
                // drawErrorEllipse(cr, n->containingNode.lock()->value.state, n->stateCov + n->stateDistribCov, 0.95);
                // n = n->parent;
                // ++depth;
                // if (depth > rrbt.graph.nodes.size()) {
                    // break;
                // }
            }
        }
        return nominalPath;
    }

}
