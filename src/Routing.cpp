#include <iostream>
#include <vector>
#include <climits>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

#include "Routing.hpp"

#include <filesystem>

#include "GridToGraph.hpp"

namespace Routing {

MathStuff::Grid2D<uint32_t> makeEdgeGrid(const std::vector<GridType::Edge> edges, const GridType::Grid& grid)
{
    const int rows = grid.size();
    const int cols = grid[0].size();
    MathStuff::Grid2D<uint32_t> result(cols, rows, -1);
    MathStuff::Grid2D<int> distsFromEdge(cols, rows, -1);

    // Multi-source BFS setup
    std::queue<std::tuple<GridType::Point, uint32_t, int>> q;
    for (int idx=0; idx < edges.size(); ++idx)
    {
        const auto& edge = edges.at(idx);
//        if (edge.toDeadEnd) continue;

        int half = 0;
        int halfCount = edge.path.size() / 2;
	    for (int dist=0; dist < edge.path.size(); ++dist)
	    {
            const auto p = edge.path.at(dist);
            uint32_t val = (dist << 16) | half | idx;
            result.put(p.first, p.second, val);
            if (halfCount && --halfCount == 0) half = GridType::EDGE_HALF;
			q.emplace(p, val, 0);
            distsFromEdge.put(p.first, p.second, 0);
	    }
    }

    // BFS propagation
    while (!q.empty()) {
        auto [pos, v, d] = q.front();
        q.pop();

        ++d;
        int dist = v >> 16;
		for (const auto& dir : GridType::directions8) {
            GridToGraph::Point next = { pos.first + dir.first, pos.second + dir.second };
            if (grid[next.second][next.first] & (GridToGraph::WALL | GridToGraph::NODE|GridType::EDGE)) continue;

            int cell = result.get(next.first, next.second);
            if (cell == -1) {
                result.put(next.first, next.second, v);
                q.emplace(next, v, d);
				distsFromEdge.put(next.first, next.second, d);
            }
            else {
                int curDist = distsFromEdge.get(next.first, next.second);
	            if (d < curDist)
	            {
					result.put(next.first, next.second, v);
					q.emplace(next, v, d);
					distsFromEdge.put(next.first, next.second, d);
	            }
            }
		}
    }
    return result;
}

///////////////////////////////////////////////////////////////////////////////////////////

SparseGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes, const std::vector<GridType::Edge>& baseEdges,
    const GridType::Grid& infoGrid)
{
    SparseGraph g;
    const int numNodes = baseNodes.size();

    // Initialize adjacency lists
    g.forward_adj.resize(numNodes);
    g.reverse_adj.resize(numNodes);
    g.nodeToEdgeIdxs.resize(numNodes);
    g.edgeCosts.reserve(baseEdges.size());
    g.edgeFromTos.reserve(baseEdges.size());
    g.nodePoints = baseNodes;

    for (size_t i = 0; i < baseEdges.size(); ++i) {
        const GridType::Edge& e = baseEdges[i];
        //if (e.toDeadEnd) continue;

        g.edgeCosts.push_back(e.toDeadEnd ? 0xffff : e.path.size());
        g.edgeFromTos.emplace_back(e.from, e.toDeadEnd ? -1 : e.to);

		g.forward_adj[e.from].emplace_back(e.to, i);
        g.reverse_adj[e.from].emplace_back(e.to, i);
        if (!e.toDeadEnd) {
			g.reverse_adj[e.to].emplace_back(e.from, i);
			g.forward_adj[e.to].emplace_back(e.from, i);

            g.nodeToEdgeIdxs[e.from].push_back(i);
            g.nodeToEdgeIdxs[e.to] .push_back(i);
        }
    }

    g.edgeGrid = makeEdgeGrid(baseEdges, infoGrid);

    return g;
}

///////////////////////////////////////////////////////////////////////////////////////////

struct SearchState {
    int node;
    int via_edge;
    int cost;
    int heuristic;

    bool operator>(const SearchState& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

std::vector<int> bidirectionalAStar(const SparseGraph& g, int sourceEdgeIdx, int targetEdgeIdx,
    const std::unordered_set<int>& allowedEdges, const std::unordered_set<int>& allowedNodes)
{
    const auto& [srcFrom, srcTo] = g.edgeFromTos[sourceEdgeIdx];
    const auto& [tgtFrom, tgtTo] = g.edgeFromTos[targetEdgeIdx];

    // Heuristic helper function
    auto heuristic = [&](int node, bool isForward) {
        if (isForward) {
            auto fromDst = std::abs(g.nodePoints[node].first - g.nodePoints[tgtFrom].first)
                + std::abs(g.nodePoints[node].second - g.nodePoints[tgtFrom].second);
            auto toDst = tgtTo == -1 ? fromDst
                : std::abs(g.nodePoints[node].first - g.nodePoints[tgtTo].first)
                + std::abs(g.nodePoints[node].second - g.nodePoints[tgtTo].second);
            return std::min(fromDst, toDst);
        }
		auto fromDst = std::abs(g.nodePoints[node].first - g.nodePoints[srcFrom].first)
			+ std::abs(g.nodePoints[node].second - g.nodePoints[srcFrom].second);
		auto toDst = srcTo == -1 ? fromDst
			: std::abs(g.nodePoints[node].first - g.nodePoints[srcTo].first)
			+ std::abs(g.nodePoints[node].second - g.nodePoints[srcTo].second);
		return std::min(fromDst, toDst);
		};

    // Initialization
    std::priority_queue<SearchState, std::vector<SearchState>, std::greater<>> forward_q, backward_q;
    std::vector<int> forward_cost(g.forward_adj.size(), INT_MAX);
    std::vector<int> backward_cost(g.forward_adj.size(), INT_MAX);
    std::vector<int> forward_edge(g.forward_adj.size(), -1);
    std::vector<int> backward_edge(g.forward_adj.size(), -1);

    // Seed both searches
    forward_q.push({ srcFrom, -1, 0, heuristic(srcFrom, true) });
    if (srcTo != -1) {
        forward_q.push({ srcTo, -1, 0, heuristic(srcTo, true) });
    }
    else {
        const auto& connections = g.nodeToEdgeIdxs[srcFrom];
        for (auto edgeIdx : connections) {
            const auto [f, t] = g.edgeFromTos[edgeIdx];
            if (f != srcFrom) {
                forward_q.push({ f, edgeIdx, 0, heuristic(f, true) });
            }
            if (t != srcFrom && t != -1) {
                forward_q.push({ t, edgeIdx, 0, heuristic(t, true) });
            }
        }
    }

    backward_q.push({ tgtFrom, -1, 0, heuristic(tgtFrom, false) });
    if (tgtTo != -1) {
        backward_q.push({ tgtTo, -1, 0, heuristic(tgtTo, false) });
    }
    else {
		const auto& connections = g.nodeToEdgeIdxs[tgtFrom];
		for (auto edgeIdx : connections) {
			const auto [f, t] = g.edgeFromTos[edgeIdx];
            if (f != tgtFrom) {
				backward_q.push({ f, edgeIdx, 0, heuristic(f, false) });
            }
            if (t != tgtFrom && t != -1) {
				backward_q.push({ t, edgeIdx, 0, heuristic(t, false) });
            }
		}
    }

    int meeting_node = -1;
    bool found_path = false;

    while (!forward_q.empty() && !backward_q.empty() && !found_path) {
        // Expand forward search
        if (!forward_q.empty()) {
            auto f = forward_q.top();
            forward_q.pop();

            if (f.cost >= forward_cost[f.node]) {
                continue;
            }
            if (!allowedNodes.count(f.node)) {
                continue;
            }
				
            forward_cost[f.node] = f.cost;
            forward_edge[f.node] = f.via_edge;

            // Check for intersection
            if (backward_cost[f.node] != INT_MAX) {
                meeting_node = f.node;
                found_path = true;
                break;
            }
            // Expand neighbors
            for (const auto& [neighbor, edge] : g.forward_adj[f.node]) {
                if (!allowedEdges.count(edge)) continue;
                int new_cost = f.cost + g.edgeCosts[edge];
                if (new_cost < forward_cost[neighbor]) {
                    forward_q.push({ neighbor, edge, new_cost, heuristic(neighbor, true) });
                }
            }
        }

        // Expand backward search
        if (!backward_q.empty()) {
            auto b = backward_q.top();
            backward_q.pop();

            if (b.cost >= backward_cost[b.node]) continue;
            if (!allowedNodes.count(b.node)) continue;

            backward_cost[b.node] = b.cost;
            backward_edge[b.node] = b.via_edge;

            // Check for immediate intersection
            if (forward_cost[b.node] != INT_MAX) {
                meeting_node = b.node;
                found_path = true;
                break;
            }

            // Expand neighbors (using reverse adjacency)
            for (const auto& [neighbor, edge] : g.reverse_adj[b.node]) {
                if (!allowedEdges.count(edge)) continue;
                int new_cost = b.cost + g.edgeCosts[edge];
                if (new_cost < backward_cost[neighbor]) {
                    backward_q.push({
                        neighbor,
                        edge,
                        new_cost,
                        heuristic(neighbor, false)
                        });
                }
            }
        }
    }
    if (!found_path) {
        return {};
    }

    // Reconstruct path
    if (meeting_node == -1) return {};

    std::vector<int> path;
    std::unordered_set<int> visited_nodes;
    int current = meeting_node;

    // Backward walk from meeting_node to source
    while (current != -1 && backward_edge[current] != -1) {
        if (visited_nodes.count(current)) {
            return {}; // Return empty path on failure
        }
        visited_nodes.insert(current);

        int edge_idx = backward_edge[current];
        path.push_back(edge_idx);
        const auto& edge = g.edgeFromTos[edge_idx];
        current = (edge.first == current) ? edge.second : edge.first;
    }
    std::reverse(path.begin(), path.end());

    // Forward walk from meeting_node to target
    current = meeting_node;
    while (current != -1 && forward_edge[current] != -1) {
        int edge_idx = forward_edge[current];
        path.push_back(edge_idx);
        const auto& edge = g.edgeFromTos[edge_idx];
        current = (edge.first == current) ? edge.second : edge.first;
    }

    return path;
}

std::vector<int> convertEdgesToNodePath(const SparseGraph& routingGraph, const std::vector<int>& edgePath, int sourceEdgeIdx)
{
    std::vector<int> nodePath;
    if (edgePath.empty()) return nodePath;
    const auto& edges = routingGraph.edgeFromTos;

    // 1. Determine direction of first edge relative to source
    int firstEdge = edgePath[0];
    int srcFrom = edges[sourceEdgeIdx].first;
    int srcTo = edges[sourceEdgeIdx].second;

    bool isForward;
    if (edges[firstEdge].first == srcFrom || edges[firstEdge].first == srcTo) {
        isForward = true;
    }
    else if (edges[firstEdge].second == srcFrom || edges[firstEdge].second == srcTo) {
        isForward = false;
    }
    else {
        std::cout << "ERROR edge doesn't connect to source: 1st:" << firstEdge
    	<< " (" << edges[firstEdge].first << "->" << edges[firstEdge].second << ")"
    	<< " src: " << sourceEdgeIdx << " (" << srcFrom << "->" << srcTo << ")" << std::endl;
    }

    // 2. Add first edge's nodes
    if (isForward) {
        nodePath.push_back(edges[firstEdge].first);
        nodePath.push_back(edges[firstEdge].second);
    }
    else {
        nodePath.push_back(edges[firstEdge].second);
        nodePath.push_back(edges[firstEdge].first);
    }

    // 3. Process subsequent edges
    for (int i = 1; i < edgePath.size(); ++i) {
		auto currentEdge = edgePath[i];
        int prevNode = nodePath.back();

        if (edges[currentEdge].first == prevNode) {
            nodePath.push_back(edges[currentEdge].second);
        }
        else if (edges[currentEdge].second == prevNode) {
            nodePath.push_back(edges[currentEdge].first);
        }
        else {
			std::cout << "ERROR Non-contiguous edge path" << std::endl;
        }
    }

    return nodePath;
}

// Entry point function
std::vector<int> findZonePath(const SparseGraph routingGraph,
    const std::vector<int>& zoneBases,
    const std::vector<int>& zoneEdges,
    int sourceEdgeIdx,
    int targetEdgeIdx)
{
    // Create allowed sets
    std::unordered_set<int> allowedEdges(zoneEdges.begin(), zoneEdges.end());
    std::unordered_set<int> allowedNodes(zoneBases.begin(), zoneBases.end());

    // Run bidirectional A*
    const auto edgePath = bidirectionalAStar(
        routingGraph,
        sourceEdgeIdx,
        targetEdgeIdx,
        allowedEdges,
        allowedNodes
    );
    return convertEdgesToNodePath(routingGraph, edgePath, sourceEdgeIdx);
}

}