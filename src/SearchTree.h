//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include <armadillo>
#include "tree.h"

#ifndef NUMP_SEARCHTREE_H
#define NUMP_SEARCHTREE_H

namespace nump {

    // xNew: a trajectory, as a function of t \in [0, tNew]
    // uNew: control to approach state
    // tNew: time for which to follow control
    //            xNew, uNew, tNew = steer(zNearest, z);
    //            zNew = xNew(tNew); // follow
    struct Steer {
        arma::vec2 x;
        arma::vec2 u; // control
        double t; // time

        arma::vec2 operator() (double t) {
            return x + u * t; // TODO: Apply the control in local coordinates of x.
        }
    };

    struct SearchNode {
        arma::vec2 state;

        double cost;

        double time;
        arma::vec2 control;
    };


//    template <class arma::vec2>
    class SearchTree {
    private:
        arma::vec2 init;
        arma::vec2 goal;

        SearchTree(arma::vec2 init, arma::vec2 goal)
            : tree(init)
            , init(init)
            , goal(goal) {
        }

    public:
        Tree<arma::vec2> tree;

        typedef std::shared_ptr<nump::Tree<arma::vec2>::Node> NodeT;

        arma::vec2 initialState() const {
            return init;
        }

        arma::vec2 goalState() const {
            return goal;
        }

        NodeT nearest(arma::vec2 state) const {

            auto min_it = std::min_element(tree.nodes.begin(), tree.nodes.end(), [state](NodeT a, NodeT b) {
                double distA = arma::norm(a->value - state);
                double distB = arma::norm(b->value - state);
                return distA < distB;
            });

            return *min_it;
        }

        std::vector<NodeT> nearVertices(NodeT node) const {
            return {};
        }

        static arma::vec2 sample() {
            arma::vec rvec = arma::randu(2);
            return {rvec(0), rvec(1)};
        }

        static Steer steer(arma::vec2 from, arma::vec2 to) {
            Steer s;

            arma::vec2 diff = to - from;
            double l = arma::norm(diff);

            s.x = from;
            s.u = arma::normalise(diff);
            s.t = std::min(l, 0.2);

            return s;
        }

        static bool obstacleFree(Steer s) {
            return true;
        }

        static bool cost(arma::vec2 state) {
            return true;
        }

//        def generateRRT(xInit, K, eta):
//            T = Tree(xInit)
//            for i in range(K):
//                    addNodeRRT(T, xInit, K, eta)
//            return T
//        template <class arma::vec2>
        static SearchTree fromRRT(arma::vec2 init, arma::vec2 goal, int n) {
            auto tree = SearchTree(init, goal);

            for (int i = 0; i < n; i++) {
                arma::vec2 zRand = sample();
                tree.extendRRT(zRand);
            }

            return tree;
        }

//        def addNodeRRT(T, xInit, K, eta):
//            xRand = sampleFree()
//            xNearest = nearest(xRand, T)
//            xNew, u = steer(xNearest, xRand, eta)
//            if obstacleFree(xNearest, xNew):
//            T.add_vertex(xNew)
//            T.add_edge(xNearest, xNew, u)
        void extendRRT(arma::vec2 z) {
            auto zNearest = nearest(z);
            auto xNew = steer(zNearest->value, z);
            auto zNew = tree.makeNode(xNew(xNew.t));

            if (!obstacleFree(xNew)) {
                return;
            }

            tree.nodes.push_back(zNew);
            zNew->parent = zNearest;
        }

//        // Note:
//        // 'z' denotes a node, which has a parent, and contains
//        // a vertex, a control, and a time for which to follow the control.
//        void extendRRTs(arma::vec2 z) {
//            auto zNearest = nearest(z);
//
//            // TODO: Return a struct.
//            // xNew: terminal state
//            // uNew: control to approach state
//            // tNew: time for which to follow control
////            xNew, uNew, tNew = steer(zNearest, z);
////            zNew = xNew(tNew); // follow
//            auto xNew = steer(zNearest, z);
//            auto zNew = nump::Tree::Node<arma::vec2>(xNew);
//
//            if (!obstacleFree(xNew)) {
//                return;
//            }
//
//            tree.nodes.push_back(zNew);
//            auto zMin = zNearest;
//            auto cMin = cost(zNew);
//        }
    };

}

#endif //NUMP_SEARCHTREE_H
