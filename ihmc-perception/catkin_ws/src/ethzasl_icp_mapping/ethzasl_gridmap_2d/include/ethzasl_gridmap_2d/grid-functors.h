#ifndef __GRID_FUNCTORS_H
#define __GRID_FUNCTORS_H

#include "grid-map.h"

struct MapCorrelation
{
	GridMap& map;
	long long int correlation;
	int pointCount;
	
	MapCorrelation(GridMap& map) : map(map), correlation(0), pointCount(0) {}
	
	bool operator()(const int x, const int y, const int texVal, const bool reverseScan)
	{
		const long long int iMapVal(map.atInternalCoord(x, y) / 4);
		correlation += texVal * iMapVal;
		++pointCount;
		return true;
	}
};

struct MapUpdater
{
	GridMap& map;
	
	MapUpdater(GridMap& map) : map(map) {}
	
	bool operator()(const int x, const int y, const int texVal, const bool reverseScan)
	{
		GridMap::Value& mapVal(map.atInternalCoord(x, y));
		int iMapVal(mapVal);
		iMapVal += texVal;
		mapVal = GridMap::saturatedValueFromInt(iMapVal);
		return true;
	}
};

struct MapConstUpdater
{
	GridMap& map;
	const int value;
	
	MapConstUpdater(GridMap& map, const int value) : map(map), value(value) {}
	
	bool operator()(const int x, const int y, const int texVal, const bool reverseScan)
	{
		GridMap::Value& mapVal(map.atInternalCoord(x, y));
		int iMapVal(mapVal);
		iMapVal += value;
		mapVal = GridMap::saturatedValueFromInt(iMapVal);
		return true;
	}
};

struct MapWallFinder
{
	GridMap& map;
	const GridMap::Value wallSeen;
	int wallX, wallY;
	
	MapWallFinder(GridMap& map, const GridMap::Value wallSeen = 0) : map(map), wallSeen(wallSeen), wallX(-1), wallY(-1) {}
	
	bool operator()(const int x, const int y, const int texVal, const bool reverseScan)
	{
		// if we have seen a wall twice, with > 90 % probability each time, we are confident it is a wall
		// as update function set prob value at 0 on wall, consider that as well there
		const GridMap::Value& mapValue(map.atInternalCoord(x, y));
		if (mapValue > wallSeen)
		{
			wallX = x;
			wallY = y;
			if (!reverseScan)
				return false;
		}
		return true;
	}
	
	void clearWall()
	{
		wallX = -1;
		wallY = -1;
	}
};

struct MapEndOfAreaFinder
{
	const GridMap& map;
	int eoaX, eoaY;
	const GridMap::Value areaLabel;
	
	MapEndOfAreaFinder(const GridMap& map, const GridMap::Value areaLabel) : map(map), eoaX(-1), eoaY(-1), areaLabel(areaLabel) {}
	
	bool operator()(const int x, const int y, const int texVal, const bool reverseScan)
	{
		// if we see another label, that means the end of this one
		const GridMap::Value& mapValue(map.atInternalCoord(x, y));
		if (mapValue != areaLabel)
		{
			eoaX = x;
			eoaY = y;
			if (!reverseScan)
				return false;
		}
		return true;
	}
	
	void clearEndOfArea()
	{
		eoaX = -1;
		eoaY = -1;
	}
};

struct Drawer
{
	GridMap& map;
	
	Drawer(GridMap& map) : map(map) {}
	
	bool operator()(const int x, const int y, const int texVal, const bool reverseScan)
	{
		map.atInternalCoord(x,y) = texVal ;
		return true;
	}
};

#endif // __GRID_FUNCTORS_H
