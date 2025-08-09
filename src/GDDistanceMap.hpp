#ifndef GD_DISTANCE_MAP_H
#define GD_DISTANCE_MAP_H

#include <utility>
#include <vector>
#include <map>
#include "GridToGraph.hpp"
#include "Router.hpp"
#include "GridTypes.hpp"
#include "WallDistanceGrid.hpp"
#include "GDDistanceMapApi.h"

namespace godot {
class GDDistanceMap {
protected:
	Router::Info info;

    GridType::Grid wallDistGrid;
    DistanceMap::SightGrid sightGrid;
	GridToGraph::Graph graph;

public:
	GDDistanceMap();
	~GDDistanceMap();


	GDDistanceMap* setCaveSize(godot::Vector2i sz);
	GDDistanceMap* setCellSize(godot::Vector2i sz);
	GDDistanceMap* setFloor(godot::Vector2i floor);
	GDDistanceMap* setTracker() {
		return this;
	}

private:
	Vector2i getMapPos(int x, int y);
};

}

#endif
