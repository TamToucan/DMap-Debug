#include <algorithm>
#include "Vector2.hpp"
#include "Router.hpp"

#include "MathUtils.h"

namespace Router {

    int pickNum(int range, int rnd)
{
    if (range <= 0) {
        return 0;
    }
    // Use modulo to wrap `rnd` into the range [0, range-1]
    return ((rnd % range) + range) % range;
}

float computeAngle(double dx, double dy) {
	if (dx == 0 && dy == 0) {
		return 0.0; // No movement
	}
    return static_cast<float>(std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}



GridType::Point nextPoint(const GridType::Point& from, const GridType::Point& dir) {
	return { from.first + dir.first, from.second + dir.second };
}

int getNextEdge(const GridType::Point& pos, std::vector<GridType::Edge> edges,
    const GridType::Grid& infoGrid, const Routing::SparseGraph& routeGraph)
{
	int edgeIdx = routeGraph.edgeGrid.get(pos.first, pos.second) & 0xffff;
    int cell = infoGrid[pos.second][pos.first];
    if (cell & GridType::NODE) {
        int n = cell & 0xffff;
        const auto& connections = routeGraph.forward_adj[n];
        const int idx = pickNum(connections.size(), pos.first ^ pos.second);
        int edgeIdx = connections[idx].second;

        if (edges[edgeIdx].to == n) {
            edgeIdx |= GridType::EDGE_HALF;
        }
    }
    return edgeIdx;
}

// Helper function to compute angle
GridType::Point nextStep(const GridType::Point& from, const GridType::Point& to) {
	GridType::Point dir = { to.first - from.first, to.second - from.second };
    return nextPoint(from, dir);
}

GridType::Point routeToNext(const GridToGraph::Graph& graph, const GridType::Point& source, const int srcEdgeIdx, const std::vector<int>& routeNodes)
{
	int srcCell = graph.infoGrid[source.second][source.first];
			

	bool incPath = false;
	const auto& fromto = graph.routingGraph.edgeFromTos[srcEdgeIdx];
	if (srcCell & GridType::XPND) {
		int dirIdx = GridType::get_XPND_DIR(srcCell);
		GridType::Point dir = GridType::directions8[dirIdx];
		return nextPoint(source, dir);
	}
	else if (routeNodes.empty()) {
		std::cout << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
		return source;
	}
	else if (fromto.first == routeNodes.front()) {
		incPath = fromto.second == routeNodes[1];
	}
	else if (fromto.second == routeNodes.front()) {
		incPath = !fromto.first == routeNodes[1];
	}
	else if (srcCell & GridType::EDGE) {
		std::cout << "ERROR: " << source.first << "," << source.second << " on edge, but not start of path" << std::endl;
		incPath = (srcCell & GridType::EDGE_HALF);
	}
	else
	{
		std::cout << "ERROR: " << source.first << "," << source.second << " cell "
			<< std::hex << srcCell << std::dec << ", but not start of path" << std::endl;
		return source;
	}

    int edgeCell = graph.routingGraph.edgeGrid.get(source.first, source.second);
	int dist = edgeCell >> 16;
    // Check if on the edge
	if (dist == 0)
	{
		if (routeNodes.size() == 1)
		{
			std::cout << "ERROR: " << source.first << "," << source.second << " edgeCell "
				<< std::hex << edgeCell << std::dec << ", dist 0, but path = 1" << std::endl;
            return source;
		}

		for (const auto& idx : graph.routingGraph.nodeToEdgeIdxs[routeNodes[1]])
		{
			const auto& ft = graph.routingGraph.edgeFromTos[idx];
            const auto& path = graph.baseEdges[idx].path;
			if (ft.first == routeNodes[1]) {
                return nextStep(path.front(), path[1]);
			}
			else if (ft.second == routeNodes[1]) {
                return nextStep(path.back(), path[path.size() - 2]);
            }
		}
        std::cout << "ERROR: " << source.first << "," << source.second << " edgeCell "
            << std::hex << edgeCell << std::dec << ". At node, but no edge going to " << routeNodes[1] << std::endl;
		return source;
	}
	const GridType::Edge& edge = graph.baseEdges[srcEdgeIdx];
	dist = std::clamp(dist + (incPath ? 1 : -1), 0, static_cast<int>(edge.path.size() - 1));
	auto movePnt = edge.path[dist];

    return nextStep(source,movePnt);
}


GridType::Point getNextMove(const GridToGraph::Graph& graph, GridType::Point& source, GridType::Point& target)
{
    int srcCell = graph.infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL)
    {
		if (srcCell & GridType::BOUNDARY) {
            int dirIdx = srcCell & GridType::DIR_MASK;
            source.first += GridType::directions8[dirIdx].first;
            source.second += GridType::directions8[dirIdx].second;
        }

        if (graph.infoGrid[source.second][source.first] & GridType::WALL) {
            std::cout << "ERROR: " << source.first << "," << source.second << " is WALL: SRC ***************************" << std::endl;
			return source;
        }
    }

    int tgtCell = graph.infoGrid[target.second][target.first];
    if (tgtCell & GridType::WALL)
    {
		if (tgtCell & GridType::BOUNDARY) {
            int dirIdx = tgtCell & GridType::DIR_MASK;
            target.first += GridType::directions8[dirIdx].first;
            target.second += GridType::directions8[dirIdx].second;
        }

        if (graph.infoGrid[target.second][target.first] & GridType::WALL) {
            std::cout << "ERROR: " << target.first << "," << target.second << " is WALL: TARG ***************************" << std::endl;
			return source;
        }
    }

    // Check all the levels and find the level at which the zones are adjacent
    // - Not sure if could do top down search, I *think* it's possible to be
    // (say) adjacent at level 2, but same at level 1, because zones are any shape
    int zoneSame = -1;
    int zoneAdjLevel = -1;
    for (int levelIdx=0; levelIdx < graph.abstractLevels.size(); ++levelIdx)
    {
        const auto& ablv = graph.abstractLevels[levelIdx];
        const auto& targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
        const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        // Check if in same zone for this level
        if (zoneSame == -1 && targetZone == sourceZone)
        {
            zoneSame = levelIdx;
            continue;
        }

        // Check if target is one of the source neighbors
        for (int adjZone : ablv.zones[sourceZone].adjacentZones)
        {
            if (adjZone == targetZone)
            {
                zoneAdjLevel = levelIdx;
                break;
            }
        }
        // If found adjacent then we have the lowest level that they are adjacent
        if (zoneAdjLevel != -1)
        {
            break;
        }
    }

    // Check if found adjacent zones
    if (zoneAdjLevel != -1) {
        const auto& ablv = graph.abstractLevels[zoneAdjLevel];
        const auto& sourceZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
        const auto& targetZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
        const auto& subgrid = ablv.subGrids[sourceZone];
        uint16_t sub = subgrid.getCostFlow(source.first, source.second, subgrid.getFlow(targetZone));
        const auto& dir = GridType::directions8[sub & 0xf];
        return nextPoint(source, dir);

    }
    // No adjacent, check if found same zone
	if (zoneSame != -1) {
		const auto& infoGrid = graph.infoGrid;

        //
		// If on XPND then move towards edge
        //
		int srcCell = infoGrid[source.second][source.first];
		if (srcCell & GridType::XPND) {
			int dirIdx = GridType::get_XPND_DIR(srcCell);
			GridType::Point dir = GridType::directions8[dirIdx];
			return nextPoint(source, dir);
		}

		// If not on EDGE then move towards edge path
		// Get the edges to route from/to and get the route
		int sourceEdge = getNextEdge(source, graph.baseEdges, graph.infoGrid, graph.routingGraph);
		int targetEdge = getNextEdge(target, graph.baseEdges, graph.infoGrid, graph.routingGraph);
        //
		// if both on the edge then move towards target
        //
		if ((sourceEdge & GridType::EDGE_MASK) == (targetEdge & GridType::EDGE_MASK))
		{
			const auto& sourcePath = graph.baseEdges[sourceEdge & GridType::EDGE_MASK].path;
			int sourceDist = graph.routingGraph.edgeGrid.get(source.first, source.second) >> 16;
			int targetDist = graph.routingGraph.edgeGrid.get(target.first, target.second) >> 16;
			sourceDist += (sourceDist < targetDist) ? 1 : -1;
			return nextStep(source, sourcePath[sourceDist]);
		}

        //
		// Use the source and target edges to get a route
        //
		int srcEdgeIdx = (sourceEdge & GridType::EDGE_MASK);
		int dstEdgeIdx = (targetEdge & GridType::EDGE_MASK);
		const auto& ablv = graph.abstractLevels[zoneSame];
		int zoneIdx = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;

		const auto& sourceInfo = ablv.zones[zoneIdx];
		const auto& zoneBases = sourceInfo.baseNodeIdxs;
		const auto& zoneEdges = sourceInfo.baseEdgeIdxs;
		const auto routeNodes = Routing::findZonePath(graph.routingGraph, zoneBases, zoneEdges, srcEdgeIdx, dstEdgeIdx);
		return routeToNext(graph, source, srcEdgeIdx, routeNodes);
	}

    //
	// Use last Abstract Level to route.
    //
	int srcEdgeIdx = getNextEdge(source, graph.baseEdges, graph.infoGrid, graph.routingGraph) & GridType::EDGE_MASK;
	int dstEdgeIdx = getNextEdge(target, graph.baseEdges, graph.infoGrid, graph.routingGraph) & GridType::EDGE_MASK;
	const auto routeNodes = Routing::findZonePath(graph.routingGraph,
		graph.routingGraph.abstractBaseNodes, graph.routingGraph.abstractBaseEdges, srcEdgeIdx, dstEdgeIdx);
    if (routeNodes.empty()) {
        return nextStep(source, target);
    }
	return routeToNext(graph, source, srcEdgeIdx, routeNodes);
}
	

//////////////////////////////////////////////////////////////////////////////

float getAngle(const GridToGraph::Graph& graph, const Router::Info& info,
					RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type) {
		GridType::Point fromPnt = { from.x / (info.mCellWidth * 8), from.y / (info.mCellHeight * 8) };
		GridType::Point toPnt = { to.x / (info.mCellWidth * 8), to.y / (info.mCellHeight * 8) };

		if (ctx->type == type) {
			if (fromPnt.first != ctx->next.first || fromPnt.second != ctx->next.second) {
				return ctx->curDir;
			}
		}

		ctx->from = fromPnt;
		ctx->to = toPnt;
		ctx->type = type;

		ctx->next = getNextMove(graph, fromPnt, toPnt);
		ctx->curDir = computeAngle(ctx->next.first - ctx->from.first, ctx->next.second - ctx->from.second);
		return ctx->curDir;
	}
}

