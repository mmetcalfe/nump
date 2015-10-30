//
// Created by Mitchell Metcalfe on 25/08/2015.
//

#ifndef NUMP_GRAPH_H
#define NUMP_GRAPH_H


#include <memory>

namespace nump {

    template <class NodeValT, class EdgeValT>
    class Graph {
    public:

        struct Node {
            Node(NodeValT val) : value(val) {}

            NodeValT value;
        };

        struct Edge {
            std::weak_ptr<Node> parent;
            std::weak_ptr<Node> child;
            EdgeValT value;

            Edge(std::weak_ptr<Node> _parent, std::weak_ptr<Node> _child, EdgeValT _value)
                : parent(_parent)
                , child(_child)
                , value(_value) { }
        };

        typedef std::shared_ptr<Node> NodeT;

        NodeT makeNode(NodeValT val) const {
            return std::make_shared<Node>(val);
        }

        Graph(NodeValT init) {
            auto z = std::make_shared<Node>(Node(init));
            nodes.push_back(z);
        }

        std::vector<Edge> outgoing(NodeT node) const {
            std::vector<Edge> out;

            for (auto& edge : edges) {
                if (edge.parent.lock() == node) {
                    out.push_back(edge);
                }
            }

            return out;
        }

        std::shared_ptr<Edge> edgeBetween(NodeT from, NodeT to) const {
            for (auto& edge : edges) {
                if (edge.parent.lock() == from && edge.child.lock() == to) {
                    return std::make_shared<Edge>(edge);
                }
            }

            return nullptr;
        }

        NodeT parent(NodeT node) const {
            for (auto& edge : edges) {
                if (edge.child.lock() == node) {
                    return edge.parent;
                }
            }

            return nullptr;
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

        void addNode(NodeT node) {
            // TODO: Throw if node is null.

            nodes.push_back(node);
        }

        void addEdge(NodeT parentNode, NodeT childNode, EdgeValT val) {
            // TODO: Throw if nodes are null or are not in the set of nodes.

            Edge edge = {parentNode, childNode, val};
            edges.push_back(edge);
        }

        std::vector<NodeT> nodes; // TODO: Use a set to allow for fast membership validation checks.
        std::vector<Edge> edges;
    };

}

#endif //NUMP_GRAPH_H
