//
// Created by Mitchell Metcalfe on 8/08/15.
//

#include <memory>
#include <armadillo>

#ifndef NUMP_GRAPH_H
#define NUMP_GRAPH_H

namespace nump {

    template <class T>
    class Tree {
    public:

//        template <class T>
        class Node;

//        template <class T>
        class Node {
        public:
            Node(T val) : value(val) {}

            std::shared_ptr<Node> parent;
            T value;
        };

        typedef std::shared_ptr<Node> NodeT;

        NodeT makeNode(T val) {
            return std::make_shared<Node>(val);
        }

        Tree(T init) {
            auto z = std::make_shared<Node>(Node(init));
            nodes.push_back(z);
        }

        NodeT parent(NodeT node) const {
            return node->parent;
        }

        unsigned int depth(NodeT node) const {
            unsigned int d = 0;
            for (NodeT n = node; parent(n) != nullptr; n = parent(n)) {
                d++;
            }
            return d;
        }

        unsigned int height() const {
            unsigned int maxDepth = 0;
            for (auto& n : nodes) {
                maxDepth = std::max(maxDepth, depth(n));
            }
            return maxDepth;
        }

//    private:
        std::vector<NodeT> nodes;
    };

}

#endif //NUMP_GRAPH_H
