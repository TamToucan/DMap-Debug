#pragma once

#include "GridTypes.hpp"
#include "GridToGraph.hpp"
#include "GDDistanceMapApi.h"
#include "Vector2i.hpp"

namespace Router {

	struct RouteCtx {
		GridType::Point from;
		GridType::Point to;
		GridType::Point next;
		int type;
		float curDir;
	};

	struct Info {
		int mCaveWidth = 2;
		int mCaveHeight = 2;
		int mBorderWidth = 1;
		int mBorderHeight = 1;
		int mCellWidth = 1;
		int mCellHeight = 1;
		int mStartCellX = 0;
		int mStartCellY = 0;
		godot::Vector2i mFloor;
		godot::Vector2i mWall;
		int mLayer;
	};

	GDDISTANCE_MAP_API float getAngle( const GridToGraph::Graph& graph, const Info& info, RouteCtx* ctx, godot::Vector2 from, godot::Vector2 to, int type);

}
