#include "ethzasl_gridmap_2d/grid-map.h"
#include "ethzasl_gridmap_2d/grid-functors.h"
#include "ros/time.h"
#include <cassert>
#include <cstdlib>
#include <algorithm>
#include <queue>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <deque>
#include <set>
#include <limits>
#include <stdexcept>
#include <iostream>

GridMap::GridMap(Group* mapGroup, const Value defaultValue) :
	resolution(0),
	startX(0),
	startY(0),
	width(0),
	height(0),
	defaultValue(defaultValue),
	mapGroup(mapGroup),
	rayCount(0)
{
	assert(mapGroup);
	if (mapGroup->empty())
		throw MapGroupEmpty();
	mapGroup->insert(this);
	const GridMap& that(**(mapGroup->begin()));
	resolution = that.resolution;
	startX = that.startX;
	startY = that.startY;
	width = that.width;
	height = that.height;
	values.resize(width*height, defaultValue);
}

//! Construct using resolution and default value and a null size, optionally initiate group
GridMap::GridMap(const float resolution, const Value defaultValue, Group* mapGroup) :
	resolution(resolution),
	startX(0),
	startY(0),
	width(0),
	height(0),
	defaultValue(defaultValue),
	mapGroup(mapGroup),
	rayCount(0)
{
	initiateMapGroup();
}

//! Construct using user-provided demonsion, optionally initiate group
GridMap::GridMap(const float resolution, const float startX, const float startY, const float width, const float height, const Value defaultValue, Group* mapGroup) :
	resolution(resolution),
	startX(startX / resolution),
	startY(startY / resolution),
	width(width / resolution),
	height(height / resolution),
	defaultValue(defaultValue),
	values(width*height, defaultValue),
	mapGroup(mapGroup),
	rayCount(0)
{
	initiateMapGroup();
}

//! Construct using resolution and data from an external file, optionally initiate group
GridMap::GridMap(const std::string &pgmFileName, const float resolution, const Value defaultValue, Group* mapGroup) :
	resolution(resolution),
	startX(0),
	startY(0),
	defaultValue(defaultValue),
	mapGroup(mapGroup),
	rayCount(0)
{
	initiateMapGroup();
	
	// TODO: move this into a function and add support for loading a map already in a group
	std::ifstream ifs(pgmFileName.c_str(), std::ifstream::in);
	if (!ifs.good())
		throw std::runtime_error("Cannot open file " + pgmFileName);
	std::string magic;
	ifs >> magic;
	if (magic != "P2")
		throw std::runtime_error("Not a PGM file: " + pgmFileName);
	ifs >> width;
	ifs >> height;
	int valuesScale;
	ifs >> valuesScale;
	values.reserve(width*height);
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			int value;
			ifs >> value;
			if (ifs.eof())
				throw std::runtime_error("Early end-of-file: " + pgmFileName);
			values.push_back(Value((value * 65535) / valuesScale - 32768));
		}
	}
}

void GridMap::initiateMapGroup()
{
	if (mapGroup)
	{
		if (!mapGroup->empty())
			throw MapGroupNotEmpty();
		mapGroup->insert(this);
	}
}

GridMap::GridMap(const GridMap& that) :
	resolution(that.resolution),
	startX(that.startX),
	startY(that.startY),
	width(that.width),
	height(that.height),
	defaultValue(that.defaultValue),
	values(that.values),
	mapGroup(that.mapGroup),
	rayCount(0)
{
	if (mapGroup)
		mapGroup->insert(this);
}

GridMap& GridMap::operator=(const GridMap& that)
{
	resolution = that.resolution;
	startX = that.startX;
	startY = that.startY;
	width = that.width;
	height = that.height;
	defaultValue = that.defaultValue;
	values = that.values;
	if (mapGroup)
		mapGroup->erase(this);
	mapGroup = that.mapGroup;
	if (mapGroup)
		mapGroup->insert(this);
	rayCount = 0;
	return *this;
}

GridMap::~GridMap()
{
	// find in group and delete
	if (mapGroup)
		mapGroup->erase(this);
}

GridMap::Value& GridMap::atInternalCoord(const int x, const int y)
{
	assert(isWithinBoundsInternal(x,y));
	return values[y * width + x];
}

GridMap::Value GridMap::atInternalCoord(const int x, const int y) const 
{
	assert(isWithinBoundsInternal(x,y));
	return values[y * width + x];
}

GridMap::Vector GridMap::fromInternalCoord(const int x, const int y) const
{
	const float halfResolution(resolution / 2);
	return Vector((x + startX) * resolution + halfResolution, (y + startY) * resolution + halfResolution);
}

GridMap::Vector GridMap::fromInternalCoord(const float x, const float y) const
{
	const float halfResolution(resolution / 2);
	return Vector((x + startX) * resolution + halfResolution, (y + startY) * resolution + halfResolution);
}

void GridMap::toInternalCoord(const Vector& pos, int& internalX, int& internalY) const
{
	internalX = pos.x() / resolution - startX;
	internalY = pos.y() / resolution - startY;
}

void GridMap::toInternalCoordSuperSampled(const Vector& pos, const int superSampleRes, int& internalX, int& internalY) const 
{
	internalX = (pos.x() * superSampleRes) / resolution - (startX * superSampleRes);
	internalY = (pos.y() * superSampleRes) / resolution - (startY * superSampleRes);
}

GridMap::Value GridMap::getValueNearest(const Vector& pos) const
{
	int x, y;
	toInternalCoord(pos, x, y);
	return atInternalCoord(x,y);
}

void GridMap::setNearestValue(const Vector& pos, const Value value)
{
	int x, y;
	toInternalCoord(pos, x, y);
	atInternalCoord(x,y) = value;
}

void GridMap::addNearestValueSaturated(const Vector& pos, const int delta)
{
	int x, y;
	toInternalCoord(pos, x, y);
	Value &v(atInternalCoord(x,y));
	v = saturatedValueFromInt(int(v) + delta);
}

GridMap::Value GridMap::getValue(const Vector& pos) const
{
	int x, y;
	toInternalCoord(pos, x, y);
	
	// we cannot make interpolation if we do not have the four points, so we return the non-interpolated point
	if ((x >= width - 1) || (y >= height - 1))
		return atInternalCoord(x,y);
	
	// interpolation
	const float halfResolution(resolution / 2);
	const Vector d((pos - fromInternalCoord(x, y) + Vector(halfResolution,halfResolution)) / float(resolution));
	const float vx0y0(atInternalCoord(x,y));
	const float vx0y1(atInternalCoord(x,y+1));
	const float vx1y0(atInternalCoord(x+1,y));
	const float vx1y1(atInternalCoord(x+1,y+1));
	const float vxNy0(vx0y0 + d.x() * (vx1y0-vx0y0));
	const float vxNy1(vx0y1 + d.x() * (vx1y1-vx0y1));
	return vxNy0 + d.y() * (vxNy1-vxNy0);
}

GridMap::Vector GridMap::getSlope(const Vector& pos, const float limit) const
{
	int x, y;
	toInternalCoord(pos, x, y);
	
	// we cannot make slope if we do not have the four points
	if ((x >= width - 1) || (y >= height - 1))
		return Vector(0,0);
	
	const float halfResolution(resolution / 2);
	const Vector d((pos - fromInternalCoord(x, y) + Vector(halfResolution,halfResolution)) / float(resolution));
	const float vx0y0(atInternalCoord(x,y));
	const float vx0y1(atInternalCoord(x,y+1));
	const float vx1y0(atInternalCoord(x+1,y));
	const float vx1y1(atInternalCoord(x+1,y+1));
	const float vdxy0(std::min(std::max(vx1y0 - vx0y0, -limit), limit));
	const float vdxy1(std::min(std::max(vx1y1 - vx0y1, -limit), limit));
	const float vx0dy(std::min(std::max(vx0y1 - vx0y0, -limit), limit));
	const float vx1dy(std::min(std::max(vx1y1 - vx1y0, -limit), limit));
	return Vector(
		d.y() * vdxy1 + (1 - d.y()) * vdxy0,
		d.x() * vx1dy + (1 - d.x()) * vx0dy
	);
}

bool GridMap::isWithinBoundsInternal(const int x, const int y) const
{
	return (x >= 0) && (y >= 0) && (x < width) && (y < height);
}

bool GridMap::isWithinBounds(const Vector& pos) const
{
	int x,y;
	toInternalCoord(pos, x, y);
	return isWithinBoundsInternal(x, y);
}

void GridMap::extendToFit(const Vector& pos)
{
	int x, y;
	toInternalCoordSuperSampled(pos, 256, x, y);
	extendMap(x / 256 - 1, y / 256 - 1, x / 256, y / 256);
}

nav_msgs::OccupancyGrid GridMap::toOccupancyGrid(const std::string& frame_id, const GridMap* knownMap) const
{
	if (knownMap)
	{
		if (!mapGroup || (mapGroup != knownMap->mapGroup))
			throw WrongKnownMap();
	}
	
	nav_msgs::OccupancyGrid og;
	
	og.header.stamp = ros::Time::now();
	og.header.frame_id = frame_id;
	
	og.info.map_load_time = ros::Time::now();
	og.info.resolution = float(resolution);
	og.info.width = width;
	og.info.height = height;
	og.info.origin.position.x = float(startX) * og.info.resolution;
	og.info.origin.position.y = float(startY) * og.info.resolution;
	og.info.origin.position.z = 0;
	og.info.origin.orientation.x = 0;
	og.info.origin.orientation.y = 0;
	og.info.origin.orientation.z = 0;
	og.info.origin.orientation.w = 1;
	
	assert(int(values.size()) == width * height);
	assert((!knownMap) || (knownMap->values.size() == values.size()));
	og.data.resize(width * height);
	for (size_t i = 0; i < values.size(); ++i)
	{
		if (knownMap && (knownMap->values[i] == -1))
			og.data[i] = -1;
		else
			og.data[i] = ((int(values[i]) + 32768) * 100) / 65535;
	}
	
	return og;
}

// line update function, with sub-pixel accuracy

template<typename F>
void GridMap::lineScan(const Vector& start, const Vector& stop, F& functor, const Value& value)
{
	Value values[2] = {value, value};
	lineScan<F>(start, stop, functor, values, 2);
}

template<typename F>
void GridMap::lineScan(const Vector& start, const Vector& stop, F& functor, const Value texture[], const unsigned textureLength)
{
	int x0, y0, x1, y1;

	rayCount++;

	toInternalCoordSuperSampled(start, 256, x0, y0);
	toInternalCoordSuperSampled(stop, 256, x1, y1);
	
	// see whether we should extend map, and if so, reget coordinates
	// we remove 1 to cope with the fact that ]-1:1[ -> 0 when subsampling
	if (extendMap(std::min(x0,x1) / 256 - 1, std::min(y0,y1) / 256 - 1, std::max(x0,x1) / 256, std::max(y0,y1) / 256))
	{
		toInternalCoordSuperSampled(start, 256, x0, y0);
		toInternalCoordSuperSampled(stop, 256, x1, y1);
	}
	
	// TODO: remove once we are confident in resize code
	assert(x0 >= 0);
	assert(x0 >> 8 < width);
	assert(x1 >= 0);
	assert(x1 >> 8 < width);
	assert(y0 >= 0);
	assert(y0 >> 8 < height);
	assert(y1 >= 0);
	assert(y1 >> 8 < height);
	
	const bool steep(abs(y1 - y0) > abs(x1 - x0));
	if (steep)
	{
		std::swap(x0, y0);
		std::swap(x1, y1);
	}
	assert(textureLength > 1);
	const int maxTexIndex(textureLength - 1);
	const int deltatex((maxTexIndex << 16)/(x1 - x0));
	
	bool reverseScan;
	if (x0 > x1)
	{
		std::swap(x0, x1);
		std::swap(y0, y1);
		reverseScan = true;
	}
	else
		reverseScan = false;
	const int deltax(x1 - x0);
	const int deltay(y1 - y0);
	
	// we now compute the sub-pixel displacement on the texture
	const int subx0( (x0 & 0xff) - 128);
	const int suby0( (y0 & 0xff) - 128);
	const int t((deltay * suby0) / deltax);
	const int xp(subx0 + t);
	const int hypSlope(sqrtf(deltay*deltay + deltax*deltax));
	const int texDiff((xp * deltax) / hypSlope);
	//cerr << "(" << x0 << "," << y0 << ") (" << deltax << "," << deltay << ") (" << subx0 << "," << suby0 << ") "<< t << " " << xp << " " << hypSlope << endl;
	//cerr << "resulting tex diff " << texDiff << endl;
	const int maxTex(textureLength << 8);
	int tex;
	if (deltatex >= 0)
	{
		tex = 0;
	}
	else
	{
		tex = maxTexIndex << 8;
	}
	// correct pixels aliasing
	tex -= (deltatex * texDiff) >> 8;
	// correct texture aliasing
	tex += 128;
	
	int y(y0);
	const int ystep = (deltay << 8) / deltax;
	
	// TODO: if required, optimize this with direct pointers, however the compiler should do so.
	
	bool functorRet;
	
	// first point may be outside texture
	if ((tex >= 0) && (tex < maxTex))
	{
		// apply functor
		if (steep)
			functorRet = functor(y >> 8, x0 >> 8, texture[tex >> 8], reverseScan);
		else
			functorRet = functor(x0 >> 8, y >> 8, texture[tex >> 8], reverseScan);
		// check whether we have an early stop
		if (!functorRet)
			return;
	}
	// increment texture and y position
	tex += deltatex;
	y += ystep;
	// x is main counter
	int x = x0 + 256;
	for (; x < x1 - 256; x += 256)
	{
		
		// apply functor
		if (steep)
			functorRet = functor(y >> 8, x >> 8, texture[tex >> 8], reverseScan);
		else
			functorRet = functor(x >> 8, y >> 8, texture[tex >> 8], reverseScan);
		// check whether we have an early stop
		if (!functorRet)
			return;
		assert(tex < maxTex);
		// increment texture and y position
		tex += deltatex;
		y += ystep;
	}
	// last point may be outside texture
	if ((tex >= 0) && (tex < maxTex))
	{
		// apply functor
		if (steep)
			functor(y >> 8, x >> 8, texture[tex >> 8], reverseScan);
		else
			functor(x >> 8, y >> 8, texture[tex >> 8], reverseScan);
	}
}

// force instantiation of these templates
template void GridMap::lineScan<MapCorrelation>(const Vector& start, const Vector& stop, MapCorrelation& functor, const Value& value);
template void GridMap::lineScan<MapUpdater>(const Vector& start, const Vector& stop, MapUpdater& functor, const Value& value);
template void GridMap::lineScan<MapConstUpdater>(const Vector& start, const Vector& stop, MapConstUpdater& functor, const Value& value);
template void GridMap::lineScan<MapWallFinder>(const Vector& start, const Vector& stop, MapWallFinder& functor, const Value& value);
template void GridMap::lineScan<MapEndOfAreaFinder>(const Vector& start, const Vector& stop, MapEndOfAreaFinder& functor, const Value& value);
template void GridMap::lineScan<Drawer>(const Vector& start, const Vector& stop, Drawer& functor, const Value& value);


bool GridMap::extendMap(int xMin, int yMin, int xMax, int yMax)
{
	// sanity check
	if (mapGroup)
	{
		// we assume that all maps of group are already in sync
		for (Group::iterator it = mapGroup->begin(); it != mapGroup->end(); ++it)
		{
			const GridMap& that(**it);
			assert(that.startX == startX);
			assert(that.startY == startY);
			assert(that.width == width);
			assert(that.height == height);
		}
	}
	
	// extend coordinates
	int deltaStartX = 0;
	int deltaStartY = 0;
	if (xMin < 0)
	{
		/*
		// minimum extension
		deltaStartX = xMin;
		xMax -= deltaStartX;
		xMin = 0;*/
		// constant extension
		deltaStartX = (xMin - 31) & (~0x1F);
		xMax -= deltaStartX;
		xMin -= deltaStartX;
		assert(xMin >= 0);
	}
	if (yMin < 0)
	{
		/*
		// minimum extension
		deltaStartY = yMin;
		yMax -= deltaStartY;
		yMin = 0;
		*/
		// constant extension
		deltaStartY = (yMin - 31) & (~0x1F);
		yMax -= deltaStartY;
		yMin -= deltaStartY;
		assert(yMin >= 0);
	}
	/*
	// minimum extension
	const int newWidth = std::max(width - deltaStartX, xMax + 1);
	const int newHeight = std::max(height - deltaStartY, yMax + 1);
	*/
	// constant extension
	int newWidth = width - deltaStartX;
	int newHeight = height - deltaStartY;
	if (newWidth < xMax + 1)
		newWidth += (xMax + 1 - newWidth + 31) & (~0x1F);
	if (newHeight < yMax + 1)
		newHeight += (yMax + 1 - newHeight + 31) & (~0x1F);
	
	// if nothing to do, return
	if ((deltaStartX == 0) && (deltaStartY == 0) && (newWidth == width) && (newHeight == height))
		return false;
	
	// if we must extend the map, extend others as well
	if (mapGroup)
	{
		// we assume that all maps of group are already in sync
		for (Group::iterator it = mapGroup->begin(); it != mapGroup->end(); ++it)
			(*it)->extendMapInternal(deltaStartX, deltaStartY, newWidth, newHeight);
	}
	else
		extendMapInternal(deltaStartX, deltaStartY, newWidth, newHeight);
	
	return true;
}

void GridMap::extendMapInternal(int deltaStartX, int deltaStartY, int newWidth, int newHeight)
{
	// create new map
	Values newValues(newWidth * newHeight, defaultValue);
	
	// copy, we know that we have enough room
	Values::const_iterator xitIn = values.begin();
	for (int y = 0; y < height; ++y)
	{
		Values::iterator xitOut = newValues.begin() + (y - deltaStartY) * newWidth - deltaStartX;
		for (int x = 0; x < width; ++x)
		{
			*xitOut++ = *xitIn++;
		}
	}
	
	// exchange old map for new map 
	swap(values, newValues);
	
	// update dimensions
	startX += deltaStartX;
	startY += deltaStartY;
	width = newWidth;
	height = newHeight;
}

void GridMap::invert()
{
	for (Values::iterator it = values.begin(); it != values.end(); ++it)
	{
		const Value value(*it);
		if (value > -32768)
			*it = -value;
		else
			*it = 32767;
	}
}

void GridMap::threshold(const Value threshold, const Value lowValue, const Value highValue)
{
	for (Values::iterator it = values.begin(); it != values.end(); ++it)
	{
		const Value value(*it);
		if (value >= threshold)
			*it = highValue;
		else
			*it = lowValue;
	}
}

static const int lookupMap8[][2] = {
	{ -1, -1 },
	{ 0, -1 },
	{ 1, -1 },
	{ -1, 0 },
	{ 1, 0 },
	{ -1, 1 },
	{ 0, 1 },
	{ 1, 1 },
};

static const int lookupMap4[][2] = {
	{ 0, -1 },		// top
	{ -1, 0 },		// left
	{ 1, 0 },		// right
	{ 0, 1 },		// bottom
};

// TODO: when "amount" is large, these are not efficient. A wavefront-based algorithm would be faster, but would require more work to implement

void GridMap::dilate4(const unsigned amount, const unsigned delta)
{
	dilateN(amount, lookupMap4, 4, delta);
}

void GridMap::dilate8(const unsigned amount, const unsigned delta)
{
	dilateN(amount, lookupMap8, 8, delta);
}

void GridMap::dilateN(const unsigned amount, const int lookup[][2], const size_t lookupSize, const unsigned delta)
{
	Values newValues(values.size());
	for (unsigned pass = 0; pass < amount; ++pass)
	{
		for (int y = 1; y < height - 1; ++y)
		{
			for (int x = 1; x < width - 1; ++x)
			{
				// TODO: optimize by putting delta pixels into the lookup values
				Value newValue(values[y * width + x]);
				for (size_t i = 0; i < lookupSize; ++i)
					newValue = std::max<int>(newValue, int(atInternalCoord(x + lookup[i][0], y + lookup[i][1])) - delta);
				newValues[y * width + x] = newValue;
			}
		}
		std::swap(values, newValues);
	}
}

void GridMap::erode4(const unsigned amount, const unsigned delta)
{
	erodeN(amount, lookupMap4, 4, delta);
}

void GridMap::erode8(const unsigned amount, const unsigned delta)
{
	erodeN(amount, lookupMap8, 8, delta);
}

void GridMap::erodeN(const unsigned amount, const int lookup[][2], const size_t lookupSize, const unsigned delta)
{
	Values newValues(values.size());
	for (unsigned pass = 0; pass < amount; ++pass)
	{
		for (int y = 1; y < height - 1; ++y)
		{
			for (int x = 1; x < width - 1; ++x)
			{
				// TODO: optimize by putting delta pixels into the lookup values
				Value newValue(values[y * width + x]);
				for (size_t i = 0; i < lookupSize; ++i)
					newValue = std::min<int>(newValue, int(atInternalCoord(x + lookup[i][0], y + lookup[i][1])) + delta);
				newValues[y * width + x] = newValue;
			}
		}
		std::swap(values, newValues);
	}
}

// A multimap that provides a method to push in unique key,value pairs
template <typename Key, typename Value>
class UniquePairsMultiMap: public std::multimap<Key, Value>
{
public:
	typedef std::multimap<Key, Value> MapType;
	void insertUniquePair(const Key& key, const Value& value)
	{
		std::pair<typename MapType::iterator, typename MapType::iterator> range = this->equal_range (key);
		if (range.first != range.second)
		{
			for (typename MapType::iterator it = range.first; it != range.second; ++it)
				if (it->second == value)
					return;
		}
		this->insert(std::pair<Key, Value>(key, value));
	}
	void erasePair(const Key& key, const Value& value)
	{
		std::pair<typename MapType::iterator, typename MapType::iterator> range = this->equal_range (key);
		for (typename MapType::iterator it = range.first; it != range.second; ++it)
			if (it->second == value)
			{
				this->erase(it);
				return;
			}
	}
};

GridMap::Labels GridMap::labelize(const Value threshold)
{
	if (width < 1 || height < 1)
		return Labels();
	
	typedef long long Int64;
	typedef std::vector<Int64> Int64Vector;
	typedef UniquePairsMultiMap<Value,Value> EquivalentLabels;
	Int64Vector xs;
	Int64Vector ys;
	Int64Vector counters;
	EquivalentLabels equivalentLabels;
	Values labeledValues(values.size(), -1);
	
	// Part 1: segment area in address-increasing order, and note equivalent relations
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const int index(y * width + x);
			if (values[index] >= threshold)
			{
				// four cases
				if (y > 0 && labeledValues[index-width] >= 0)
				{
					// top is labeled
					if (x > 0 && labeledValues[index-1] >= 0)
					{
						// left is labeled
						if (labeledValues[index-1] != labeledValues[index-width])
						{
							// note equivalent relation
							equivalentLabels.insertUniquePair(labeledValues[index-width], labeledValues[index-1]);
							equivalentLabels.insertUniquePair(labeledValues[index-1], labeledValues[index-width]);
						}
					}
					// extend top label
					const Value label(labeledValues[index-width]);
					xs[label] += Int64(x); ys[label] += Int64(y); counters[label] += 1;
					labeledValues[index] = label;
				}
				else
				{
					// top is not labeled
					if (x > 0 && labeledValues[index-1] >= 0)
					{
						// left is labeled
						// extend left label
						const Value label(labeledValues[index-1]);
						xs[label] += Int64(x); ys[label] += Int64(y); counters[label] += 1;
						labeledValues[index] = label;
					}
					else
					{
						// left is not labeled
						// new area
						const Value label(counters.size());
						assert(counters.size() < size_t(std::numeric_limits<Value>::max()));
						xs.push_back(Int64(x)); ys.push_back(Int64(y)); counters.push_back(1);
						labeledValues[index] = label;
					}
				}
			}
		}
	}
	
	// Part 2: fuse equivalent relations
	typedef std::vector<Value> LabelsLookup;
	LabelsLookup labelsLookup(counters.size(), -1);
	Int64Vector targetsXs;
	Int64Vector targetsYs;
	Int64Vector targetsCounters;
	while (!equivalentLabels.empty())
	{
		// build a new queue and create target label
		typedef std::deque<Value> LabelsQueue;
		LabelsQueue workQueue;
		workQueue.push_back(equivalentLabels.begin()->first);
		const Value targetLabel(targetsCounters.size());
		targetsXs.push_back(0);
		targetsYs.push_back(0);
		targetsCounters.push_back(0);
		while (!workQueue.empty())
		{
			// process this one and its attached nodes
			const Value current(workQueue.front());
			workQueue.pop_front();
			labelsLookup[current] = targetLabel;
			targetsXs[targetLabel] += xs[current];
			targetsYs[targetLabel] += ys[current];
			targetsCounters[targetLabel] += counters[current];
			std::pair<EquivalentLabels::iterator, EquivalentLabels::iterator> range = equivalentLabels.equal_range(current);
			while (range.first != range.second)
			{
				EquivalentLabels::iterator currentIt = range.first;
				// push to work queue
				workQueue.push_back(currentIt->second);
				// remove relation from graph
				equivalentLabels.erasePair(currentIt->second, currentIt->first);
				equivalentLabels.erase(currentIt);
				// get range again
				range = equivalentLabels.equal_range(current);
			}
		}
	}
	// copy labels with no equivalent relations
	for (size_t i = 0; i < labelsLookup.size(); ++i)
	{
		if (labelsLookup[i] < 0)
		{
			const Value targetLabel(targetsCounters.size());
			labelsLookup[i] = targetLabel;
			targetsXs.push_back(xs[i]);
			targetsYs.push_back(ys[i]);
			targetsCounters.push_back(counters[i]);
		}
	}
	
	// Part 3: labelize final image and build center of mass vectors
	// TODO: optimize with a single loop
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const int index(y * width + x);
			if (labeledValues[index] >= 0)
				values[index] = labelsLookup[labeledValues[index]];
			else
				values[index] = -1;
		}
	}
	
	Labels labels;
	labels.reserve(targetsCounters.size());
	for (size_t i = 0; i < targetsCounters.size(); ++i)
	{
		labels.push_back(
			boost::make_tuple(
				i,
				fromInternalCoord(
					float(targetsXs[i]) / float(targetsCounters[i]),
					float(targetsYs[i]) / float(targetsCounters[i])
				),
				targetsCounters[i]
			)
		);
	}
	return labels;
}

//! Return a number in [0;1[ in a uniform distribution
/*! \ingroup an */
static float uniformRand(void)
{
	return float(rand())/RAND_MAX;
}

//! Return a random number with a gaussian distribution of a certain mean and standard deviation.
/*! \ingroup an */
static float gaussianRand(float mean, float sigm)
{
	// Generation using the Polar (Box-Mueller) method.
	// Code inspired by GSL, which is a really great math lib.
	// http://sources.redhat.com/gsl/
	// C++ wrapper available.
	// http://gslwrap.sourceforge.net/
	float r, x, y;
	
	// Generate random number in unity circle.
	do
	{
		x = uniformRand()*2 - 1;
		y = uniformRand()*2 - 1;
		r = x*x + y*y;
	}
	while (r > 1.0 || r == 0);
	
	// Box-Muller transform.
	return sigm * y * sqrt (-2.0 * log(r) / r) + mean;
}

static GridMap::Vector getRandomPoint(const GridMap::Label& area, const int resolution)
{
	return GridMap::Vector(
		gaussianRand(area.get<1>().x(), float(resolution)*sqrtf(area.get<2>())/2),
		gaussianRand(area.get<1>().y(), float(resolution)*sqrtf(area.get<2>())/2)
	);
}

GridMap::VectorPair GridMap::closestPoints(const Label& area0, const Label& area1, const unsigned monteCarloSteps) const
{
	const Value id0(area0.get<0>());
	//const Vector c0(area0.get<1>());
	const Value id1(area1.get<0>());
	//const Vector c1(area1.get<1>());
	//const Vector pc0c1u((c1-c0).perp().unitary());
	float bestDist(std::numeric_limits<float>::max());
	VectorPair bestPoints;
	for (unsigned i = 0; i < monteCarloSteps; ++i)
	// only one step for now, as we use centers
	{
		// two points
		Vector p0, p1;
		do
		{
			p0 = getRandomPoint(area0, resolution);
		}
		while (!isWithinBounds(p0) || getValueNearest(p0) != id0);
		do
		{
			p1 = getRandomPoint(area1, resolution);
		}
		while (!isWithinBounds(p1) || getValueNearest(p1) != id1);
		/*// use center for now
		Vector p0 = area0.get<1>();
		while (!isWithinBounds(p0) || getValueNearest(p0) != id0)
			p0 = getRandomPoint(area0, resolution);
		Vector p1 = area1.get<1>();
		while (!isWithinBounds(p1) || getValueNearest(p1) != id1)
			p1 = getRandomPoint(area1, resolution);
		*/
		// NOTE: 	We use ugly const cast before it allows us to have only a non const version of linescan
		//			We thus save code in return for some uglyness.
		// find wall of area 0
		MapEndOfAreaFinder eoa0Finder(*this, id0);
		const_cast<GridMap&>(*this).lineScan(p0, p1, eoa0Finder);
		if (eoa0Finder.eoaX >= 0 && eoa0Finder.eoaY >= 0)
			p0 = fromInternalCoord(eoa0Finder.eoaX, eoa0Finder.eoaY);
		// find wall of area 1
		MapEndOfAreaFinder eoa1Finder(*this, id1);
		const_cast<GridMap&>(*this).lineScan(p1, p0, eoa1Finder);
		if (eoa1Finder.eoaX >= 0 && eoa1Finder.eoaY >= 0)
			p1 = fromInternalCoord(eoa1Finder.eoaX, eoa1Finder.eoaY);
		
		// is it better ?
		const float dist((p1-p0). squaredNorm());
		/*const Vector pm((p1+p0)/2);
		const Vector r(p0 - pm);
		const float dC(fabsf(pc0c1u * r));
		const float score(dist + dC / 5);*/
		if (dist < bestDist)
		{
			bestDist = dist;
			bestPoints.first = p0;
			bestPoints.second = p1;
		}
	}
	return bestPoints;
}

GridMap GridMap::buildGradient(const Vector& goal, bool &isSuccess) const
{
	typedef std::pair<int,int> Cell;
	typedef std::queue<Cell> CellsQueue;

	// note: we consider 6 bits sub-precision, that leads us to 512 cells range of pathfinding
	const int sp(6);
	const int oneSp(1<<sp);
	const float oneSpF(oneSp);
	const Value maxValue(std::numeric_limits<Value>::max());
	GridMap gradientMap(resolution, startX, startY, width, height, maxValue);
	
	// goal
	int goalX, goalY;
	toInternalCoord(goal, goalX, goalY);
	if (atInternalCoord(goalX, goalY) == 0)
	{
		std::cerr << "GridMap::buildGradient(" << goal << ") : cannot create, goal is an obstacle (internal coord =" << goalX << "," << goalY << ")" << std::endl;
		isSuccess = false;
		return gradientMap;
	}
	gradientMap.atInternalCoord(goalX, goalY) = 0;
	
	// build initial queue of cells
	CellsQueue *workQueue = new CellsQueue;
	// S
	if ((goalY > 0) && (atInternalCoord(goalX, goalY-1) != 0))
		workQueue->push(Cell(goalX, goalY-1));
	// W
	if ((goalX > 0) && (atInternalCoord(goalX-1, goalY) != 0))
		workQueue->push(Cell(goalX-1, goalY));
	// E
	if ((goalX < width - 1) && (atInternalCoord(goalX+1, goalY) != 0))
		workQueue->push(Cell(goalX+1, goalY));
	// N
	if ((goalY < height - 1) && (atInternalCoord(goalX, goalY+1) != 0))
		workQueue->push(Cell(goalX, goalY+1));
	
	if (workQueue->empty())
	{
		std::cerr << "GridMap::buildGradient(" << goal << ") : goal is locked (internal coord =" << goalX << "," << goalY << ")" << std::endl;
	}
	
	// propagate
	while (!workQueue->empty())
	{
		const Cell& cell(workQueue->front());
		const int x(cell.first);
		const int y(cell.second);
		workQueue->pop();
		
		// get values of neightbours
		int smallestValue(maxValue);
		int secondSmallestValue(maxValue);
		// S
		if (y > 0)
		{
			const int value(gradientMap.atInternalCoord(x, y-1));
			if (value < smallestValue)
				secondSmallestValue = smallestValue, smallestValue = value;
			else
				secondSmallestValue = std::min(value, secondSmallestValue);
		}
		// W
		if (x > 0)
		{
			const int value(gradientMap.atInternalCoord(x-1, y));
			if (value < smallestValue)
				secondSmallestValue = smallestValue, smallestValue = value;
			else
				secondSmallestValue = std::min(value, secondSmallestValue);
		}
		// E
		if (x < width - 1)
		{
			const int value(gradientMap.atInternalCoord(x+1, y));
			if (value < smallestValue)
				secondSmallestValue = smallestValue, smallestValue = value;
			else
				secondSmallestValue = std::min(value, secondSmallestValue);
		}
		// N
		if (y < height - 1)
		{
			const int value(gradientMap.atInternalCoord(x, y+1));
			if (value < smallestValue)
				secondSmallestValue = smallestValue, smallestValue = value;
			else
				secondSmallestValue = std::min(value, secondSmallestValue);
		}
		
		// if we haven't found any parent wave, that means that something is wrong there
		assert(smallestValue != maxValue);
		
		// compute new value, from Philippsen TR E* 06 but simplified for uniform cells
		// switch to float for computation
		const float F(float(atInternalCoord(x, y)) / 32767.f);
		assert(F >= 0.f);
		const float Ta(float(smallestValue) / oneSpF);
		const float Tc(float(secondSmallestValue) / oneSpF);
		float T;
		if (Tc - Ta >= 1.f/F)
		{
			T = Ta + 1.f/F;
		}
		else
		{
			const float beta(Ta + Tc);
			const float gamma(Ta*Ta + Tc*Tc - 1.f/(F*F));
			T = 0.5f * (beta + sqrtf(beta*beta-2.f*gamma));
		}
		const int newValue = int(T * oneSpF);
		
		// update value
		const int currentValue(gradientMap.atInternalCoord(x,y));
		if (newValue >= currentValue)
			continue;
		//std::cout << "updating " << x << "," << y << "; old value = "<< currentValue << ", new value = " << newValue << std::endl;
		gradientMap.atInternalCoord(x,y) = newValue;
		
		// propagate
		// S
		if ((y > 0) && (atInternalCoord(x, y-1) != 0))
			workQueue->push(Cell(x, y-1));
		// W
		if ((x > 0) && (atInternalCoord(x-1, y) != 0))
			workQueue->push(Cell(x-1, y));
		// E
		if ((x < width - 1) && (atInternalCoord(x+1, y) != 0))
			workQueue->push(Cell(x+1, y));
		// N
		if ((y < height - 1) && (atInternalCoord(x, y+1) != 0))
			workQueue->push(Cell(x, y+1));
	}
	
	delete workQueue;
	isSuccess = true;
	return gradientMap;
}

inline float binaryEntropy(const GridMap::Value v)
{
	const float vf(v);
	const float pf(1/(1+expf(-vf/32768.f)));
	return -(pf * logf(pf) + (1-pf)*logf(1-pf));
}

float GridMap::getInformationQuantity() const
{
	float i = 0;
	const float hPrior(binaryEntropy(0));
	for (Values::const_iterator it(values.begin()); it != values.end(); ++it)
		i += hPrior - binaryEntropy(*it);
	return i;
}

void GridMap::toPGMFile(const std::string& fileName, const int divisorToPGM) const
{
	std::ofstream file(fileName.c_str());
	if (!file.good())
	{
		std::cerr << "Cannot open file " << fileName << " for writing." << std::endl;
		return;
	}
	
	file << "P2\n";
	file << width << " " << height << "\n255\n";
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
			file << (int(atInternalCoord(x,y)) / divisorToPGM + 128) << " ";
		file << "\n";
	}
}
