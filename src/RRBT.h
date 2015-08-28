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

namespace nump {

    class RRBT {
    public:

        cairo_t *cairo = nullptr;

        struct Vertex;
        typedef arma::vec2 StateT;
        typedef arma::mat22 StateCovT;
        typedef Trajectory<StateT> TrajT;
        typedef Graph<Vertex, TrajT> GraphT;

        GraphT graph;
        typedef std::shared_ptr<GraphT::Node> NodeT;

        struct BeliefNode;
        typedef std::shared_ptr<BeliefNode> BeliefNodePtr;


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

            bool dominates(BeliefNodePtr belief, double tolerance = 0);
        };
        struct Vertex {
            StateT state; // x
            std::vector<BeliefNodePtr> beliefNodes; // N
        };

        std::vector<nump::math::Circle> obstacles;

        StateT init;
        StateT goal;

    private:
        RRBT(StateT init, StateCovT initCov, StateT goal);

    public:
        bool obstacleFree(TrajT s) const;

        // Dynamics model:
        static inline double distance(StateT a, StateT b);

        static TrajT steer(StateT from, StateT to);

        static double J(TrajT s);

        // Class and member functions:
        static RRBT fromRRBT(cairo_t *cr, StateT init, StateCovT initCov, StateT goal, int n,
                                   std::vector<nump::math::Circle> obstacles,
                                   std::function<void(const RRBT &, StateT, bool)> callback = [](
                const RRBT &, StateT, bool) { });

        bool extendRRBT(cairo_t *cr, StateT z);

        NodeT nearest(StateT state) const;

        TrajT connect(StateT from, StateT to);

        std::vector<NodeT> neighbours(NodeT node);

        BeliefNodePtr propagate(const TrajT& traj, BeliefNodePtr belief);

        bool appendBelief(NodeT node, BeliefNodePtr belief);

        std::vector<NodeT> nearVertices(NodeT queryNode, unsigned long numVertices) const;

        double maxCost() const;

    };
}

#endif //NUMP_RRBT_H
