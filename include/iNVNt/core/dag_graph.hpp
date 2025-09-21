#pragma once

#include <vector>
#include <algorithm>
#include <memory>
#include <limits>
#include "iNVNt/core/math_types.hpp"

namespace nvn
{
    template <typename T>
    struct DirectedAcyclicGraph
    {
        constexpr static index_t InvalidIndex = std::numeric_limits<index_t>::max();

        std::vector<std::unique_ptr<T>> nodes;
        std::vector<index_t> node_parent;
        std::vector<index_t> node_depth;

        DirectedAcyclicGraph() = default;
        inline bool hasNode(index_t i) const noexcept { return i < nodes.size(); }
        inline index_t addNodePtr(std::unique_ptr<T> node, index_t parent = InvalidIndex)
        {
            if (parent >= nodes.size())
                parent = InvalidIndex;
            nodes.emplace_back(std::move(node));
            node_parent.push_back(parent);
            if (parent == InvalidIndex)
            {
                node_depth.push_back(0);
            }
            else
            {
                node_depth.push_back(node_depth[parent] + 1);
            }
            return nodes.size() - 1;
        }

        template <typename U>
        inline index_t addNode(U &&node, index_t parent = InvalidIndex)
        {
            using V = std::remove_cvref_t<U>;
            return addNodePtr(std::make_unique<V>(std::forward<U>(node)), parent);
        }
    };

    template <typename T>
    struct DAGPathSearchResult
    {
        std::vector<T *> up;   // from A to LCA (including A, including LCA)
        std::vector<T *> down; // from LCA to B (including B, excluding LCA)
    };

    template <typename T>
    struct DAGPathSearch
    {
        const DirectedAcyclicGraph<T> *dag;

        DAGPathSearch(const DirectedAcyclicGraph<T> *graph)
            : dag(graph) {}

        inline DAGPathSearchResult<T> operator()(index_t A, index_t B) const
        {
            DAGPathSearchResult<T> result{};
            if (!dag->hasNode(A) || !dag->hasNode(B))
                return result;
            index_t index_up = A,
                    index_down = B;
            index_t depth_up = dag->node_depth[A],
                    depth_down = dag->node_depth[B];
            result.up.push_back(dag->nodes[index_up].get());
            while (index_up != index_down)
            {
                if (depth_up == depth_down)
                {
                    index_up = dag->node_parent[index_up];
                    if (!dag->hasNode(index_up))
                        return {};
                    result.up.push_back(dag->nodes[index_up].get());
                    result.down.push_back(dag->nodes[index_down].get());
                    index_down = dag->node_parent[index_down];
                    if (!dag->hasNode(index_down))
                        return {};
                }
                else if (depth_up > depth_down)
                {
                    index_up = dag->node_parent[index_up];
                    if (!dag->hasNode(index_up))
                        return {};
                    result.up.push_back(dag->nodes[index_up].get());
                    --depth_up;
                }
                else
                {
                    result.down.push_back(dag->nodes[index_down].get());
                    index_down = dag->node_parent[index_down];
                    if (!dag->hasNode(index_down))
                        return {};
                    --depth_down;
                }
            }
            std::reverse(result.down.begin(), result.down.end());
            return result;
        }
    };

}