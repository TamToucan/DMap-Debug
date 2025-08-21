#include <iostream>
#include <string>
#include "Vector2i.hpp"
#include "Vector2.hpp"
using namespace godot	;


#include "GridTypes.hpp"
#include "GridToGraph.hpp"
#include "Router.hpp"

#include "GDDistanceMap.hpp"

struct FlowField::SubGrid subgrid;

std::pair<float, float> computeDirection(float angleDeg) {
	const double MYPI = 3.14159265358979323846;
	double radians = angleDeg * (MYPI / 180.0);
	return { std::cos(radians), std::sin(radians) };
}

int main(int argc, char** argv)
{
	auto grid = GridToGraph::readGridFromFile("GRID.txt");
	auto graph = GridToGraph::makeGraph(grid);
	auto pathGrid(grid);
	Router::Info info;
	info.mCaveHeight = 32;
	info.mCellWidth = 8;
	info.mCellHeight = 8;
	Vector2 from(300, 250);
	Vector2 to(1830, 986);
	Router::RouteCtx* ctx = new Router::RouteCtx();
	ctx->type = -1;
	int count = 1000;
	int mv = 1;
	do {
		GridType::Point fromPnt = { from.x / (info.mCellWidth * 8), from.y / (info.mCellHeight * 8) };
		pathGrid[fromPnt.second][fromPnt.first] = 'x';
		float ang = Router::getAngle(graph, info, ctx, from, to, 0);
		std::pair<float, float> mv = computeDirection(ang);
		from.x += mv.first * 23;
		from.y += mv.second * 23;
		std::cerr << "CTV MV " << mv.first << "," << mv.second
			<< "  ang " << ang<< " cell: " << fromPnt.first << "," << fromPnt.second << std::endl;
	} while ((ctx->from != ctx->to) && (--count > 0));
	delete ctx;

	for (int row = 0; row < pathGrid.size(); ++row) {
		for (int col = 0; col < pathGrid[0].size(); ++col) {
			int v = pathGrid[row][col];
			if (v == 0) {
				std::cerr << "#";
			}
			else
			if (v == 'x') {
				std::cerr << "x";
			}
			else {
				std::cerr << " ";
			}
		}
		std::cerr << std::endl;
	}

	return 1;
}

