//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include <armadillo>
#include "math/geometry.h"
#include "tree.h"
#include <cairo/cairo.h>

#ifndef NUMP_SEARCHTREE_H
#define NUMP_SEARCHTREE_H

namespace nump {

    // xNew: a trajectory, as a function of t \in [0, tNew]
    // uNew: control to approach state
    // tNew: time for which to follow control
    //            xNew, uNew, tNew = steer(zNearest, z);
    //            zNew = xNew(tNew); // follow
    struct Steer {
        arma::vec2 x = {0, 0};
        arma::vec2 u = {0, 0}; // control
        double t = 0; // time

        bool reachesTarget = true;

        Steer()
                : x({0, 0})
                , u({0, 0})
                , t(0) {
        }

        Steer(arma::vec2 x, arma::vec2 u, double t)
                : x(x)
                , u(u)
                , t(t) {
        }

        arma::vec2 operator() (double t) {
            // TODO: Apply the control in local coordinates of x.
            // TODO: Apply the control continuously over the timespan, instead of just adding it.
            return x + u * t;
        }
    };

//    template <class arma::vec2>
    class SearchTree {
    public:

        cairo_t *cairo = nullptr;

        struct SearchNode;
        typedef arma::vec2 StateT;
        typedef Tree<SearchNode> TreeT;

        struct SearchNode {
            StateT state;
            double cost;
            Steer traj;
        };

        TreeT tree;
        std::vector<nump::math::Circle> obstacles;


        //        typedef std::shared_ptr<nump::Tree<SearchNode>::Node> NodeT;
        typedef std::shared_ptr<TreeT::Node> NodeT;


    private:
        StateT init;
        StateT goal;

        SearchTree(StateT init, StateT goal);

    public:

        static inline double distance(StateT a, StateT b);
        static StateT sample();
        static Steer steer(StateT from, StateT to);
        static double J(Steer s);
        static inline double cost(SearchNode node);
        static SearchTree fromRRT(cairo_t *cr, StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles);
        static SearchTree fromRRTs(cairo_t *cr, StateT init, StateT goal, int n, std::vector<nump::math::Circle> obstacles, std::function<void(const SearchTree&, StateT, bool)> callback = [](const SearchTree&, StateT, bool){});

        StateT initialState() const;
        StateT goalState() const;
        NodeT nearest(StateT state) const;
        double maxCost() const;
        std::vector<NodeT> nearVertices(NodeT queryNode, unsigned long numVertices) const;
        bool obstacleFree(Steer s) const;
        void setParent(NodeT node, NodeT parent, Steer edgeTraj, double nodeCost);
        bool extendRRT(StateT z);
        bool extendRRTs(cairo_t *cr, StateT z);
    };

}

#endif //NUMP_SEARCHTREE_H
