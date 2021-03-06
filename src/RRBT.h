//
// Created by Mitchell Metcalfe on 24/08/2015.
//

#ifndef NUMP_RRBT_H
#define NUMP_RRBT_H

#include <vector>
#include <armadillo>
#include <cairo/cairo.h>
#include "math/geometry.h"
//#include "Tree.h"
#include "Graph.h"
#include "Trajectory.h"
#include "SearchScenario.h"
#include "BipedRobotModel.h"

namespace nump {

    /* See the following paper for details:
        @article{Bry:2011eh,
            author = {Bry, Adam and Roy, Nicholas},
            title = {{Rapidly-exploring Random Belief Trees for motion planning under uncertainty.}},
            journal = {ICRA},
            year = {2011},
            pages = {723--730}
        }
     */
    class RRBT {
    public:

        cairo_t *cairo = nullptr;

        struct Vertex;
        struct BeliefNode;

        typedef BipedRobotModel RobotModel;
        typedef RobotModel::State StateT;
        typedef RobotModel::MotionCov StateCovT;
//        typedef arma::vec2 StateT;
//        typedef arma::mat22 StateCovT;
        typedef Trajectory<StateT> TrajT;
        typedef Graph<Vertex, TrajT> GraphT;
        typedef std::shared_ptr<GraphT::Node> NodeT;
        typedef std::shared_ptr<BeliefNode> BeliefNodePtr;

        numptest::SearchScenario::Config scenario;
        GraphT graph;
        BeliefNodePtr initialBelief;

        std::vector<double> iterationTimes;

        std::vector<std::weak_ptr<GraphT::Node>> goalVertices;
        std::weak_ptr<BeliefNode> bestGoalNode;
        bool idealPositionAdded = false;
        double timeSpentPlanning = 0;

        struct BeliefNode { // n \in v.N
            StateCovT stateCov; // Σ
            StateCovT stateDistribCov; // Λ
            double cost; // c
            BeliefNodePtr parent; // parent
            std::weak_ptr<GraphT::Node> containingNode; // The vertex that contains this node.

            BeliefNode(std::weak_ptr<GraphT::Node> _containingNode) {
                stateCov.zeros();
                stateDistribCov.zeros();
                cost = 0;
                parent = nullptr;
                containingNode = _containingNode;
            }

            bool dominates(BeliefNodePtr belief, double tolerance = 0, double costTolerance = 0);
        };
        struct Vertex {
            StateT state; // x
            std::vector<BeliefNodePtr> beliefNodes; // N
        };

    private:
//        RRBT(StateT init, StateCovT initCov, StateT goal);
        RRBT(const numptest::SearchScenario::Config& scenario);

    public:
        bool obstacleFree(TrajT s) const;

        // Dynamics model:
        static inline double distance(StateT a, StateT b);

        static TrajT steer(StateT from, StateT to);

        static double J(TrajT s);

        bool extendRRBT(cairo_t *cr, StateT z);

        // Class and member functions:
//        static RRBT fromRRBT(cairo_t *cr, StateT init, StateCovT initCov, StateT goal, int n,
//                             std::vector<nump::math::Circle> obstacles,
//                             std::vector<nump::math::Circle> measurementRegions,
//                             std::function<void(const RRBT &, StateT, bool)> callback = [](auto a, auto b, auto c){});

        static RRBT fromSearchScenario(
                const numptest::SearchScenario::Config& scenario,
                cairo_t *cr = nullptr,
                std::function<void(const RRBT&, StateT, bool)> callback = [](auto a, auto b, auto c){}
        );


        NodeT nearest(StateT state) const;

        static TrajT connect(StateT from, StateT to);

        std::vector<NodeT> neighbours(NodeT node);

        static BeliefNodePtr propagate(
                const TrajT& traj,
                BeliefNodePtr belief,
                arma::vec2 footprintSize,
                const numptest::SearchScenario::Config::RRBT& rrbtConfig,
                const std::vector<nump::math::Circle>& obstacles,
                const std::vector<nump::math::Circle>& measurementRegions,
                std::function<void(double, StateT, BeliefNodePtr)> callback = [](auto a, auto b, auto c){}
        );

        static bool satisfiesChanceConstraint(
            StateT state, StateCovT stateCov, arma::vec2 footprintSize,
            const std::vector<nump::math::Circle>& obstacles,
            double chanceConstraint
        );

        bool appendBelief(NodeT node, BeliefNodePtr belief);

        std::vector<NodeT> nearVertices(NodeT queryNode, unsigned long numVertices) const;

        double maxCost() const;

        static bool compareCovariancesLT(const RRBT::StateCovT& covA, const RRBT::StateCovT& covB);

        std::shared_ptr<BeliefNode> findBestGoalState() const;
        std::shared_ptr<BeliefNode> findBestGoalStateWithSuccessThreshold() const;

        nump::Path<BipedRobotModel::State> getSolutionPath() const;
    };
}

#endif //NUMP_RRBT_H
