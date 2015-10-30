//
// Created by Mitchell Metcalfe on 8/08/15.
//

#ifndef NUMP_SEARCHTREE_H
#define NUMP_SEARCHTREE_H

#include <vector>
#include <armadillo>
#include <cairo/cairo.h>
#include "math/geometry.h"
#include "Tree.h"
#include "Trajectory.h"
#include "SearchScenario.h"
#include "BipedRobotModel.h"

namespace nump {

//    template <class arma::vec2>
    class SearchTree {
    public:

        cairo_t *cairo = nullptr;

        struct SearchNode;

        typedef BipedRobotModel RobotModel;
        typedef RobotModel::State StateT;

        // typedef Transform2D StateT;
//        typedef arma::vec2 StateT;
        typedef Trajectory<StateT> TrajT;
        typedef Tree<SearchNode> TreeT;

        struct SearchNode {
            StateT state;
            double cost;
            TrajT traj;
        };

        TreeT tree;
        numptest::SearchScenario::Config scenario;

        // std::vector<nump::math::Circle> obstacles;

        //        typedef std::shared_ptr<nump::Tree<SearchNode>::Node> NodeT;
        typedef std::shared_ptr<TreeT::Node> NodeT;


        nump::Path<BipedRobotModel::State> getSolutionPath() const;

    private:
        // StateT init;
        // StateT goal;

        // SearchTree(StateT init, StateT goal);
        SearchTree(const numptest::SearchScenario::Config& scenario);

    public:

//        // State space:
//        static StateT sample();

        // Collision detection:
        bool obstacleFree(TrajT s) const;

        // Dynamics model:
        static inline double distance(StateT a, StateT b);

        static TrajT steer(StateT from, StateT to);

        static double J(TrajT s);

        // Class and member functions:
        // static SearchTree fromRRT(cairo_t *cr, StateT init, StateT goal, int n,
        //                           std::vector<nump::math::Circle> obstacles);

        static SearchTree fromRRTs(
            const numptest::SearchScenario::Config& scenario,
            cairo_t *cr = nullptr,
            // StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles,
            std::function<void(const SearchTree&, StateT, bool)> callback = [](auto a, auto b, auto c){}
       );

        static inline double cost(SearchNode node);

        // StateT initialState() const;
        //
        // StateT goalState() const;

        NodeT nearest(StateT state) const;

        std::vector<NodeT> nearVertices(NodeT queryNode, unsigned long numVertices) const;

        double maxCost() const;

        void setParent(NodeT node, NodeT parent, TrajT edgeTraj, double nodeCost) const;

        bool extendRRT(StateT z);

        bool extendRRTs(cairo_t *cr, StateT z);

        NodeT createValidNodeForState(StateT state) const;

        void optimiseParent(NodeT zNew, std::vector<NodeT> zNearby) const;

        void optimiseNeighbours(NodeT zNew, std::vector<NodeT> zNearby);

    };

}

#endif //NUMP_SEARCHTREE_H
