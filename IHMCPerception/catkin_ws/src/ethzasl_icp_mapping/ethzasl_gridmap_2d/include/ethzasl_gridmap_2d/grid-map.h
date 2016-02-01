#ifndef __GRID_MAP_H
#define __GRID_MAP_H

#include <vector>
#include <set>
#include <cassert>
#include <stdexcept>
#include <boost/tuple/tuple.hpp>
#include <Eigen/Eigen>
#include "nav_msgs/OccupancyGrid.h"

class GridMap
{
public:
	typedef signed short Value;
	typedef Eigen::Vector2f Vector;
	typedef std::set<GridMap*> Group;
	
	struct MapGroupEmpty:public std::runtime_error
	{
		MapGroupEmpty():std::runtime_error("Map group empty, use constructor providing at least resolution and defaultValue.") {}
	};
	struct MapGroupNotEmpty:public std::runtime_error
	{
		MapGroupNotEmpty():std::runtime_error("Map group not empty, use constructor taking only gridMapGroup and defaultValue as argument.") {}
	};
	struct WrongKnownMap:public std::runtime_error
	{
		WrongKnownMap():std::runtime_error("Known map for OccupancyGrid conversion is not in the same map group as the probabilistic map or map group missing.") {}
	};

protected:
	typedef std::vector<Value> Values;
	
	float resolution; // resolution, in unit per grid cell
	int startX, startY; // start of map, in grid cell
	int width, height; // size of the map, in grid cell
	Value defaultValue; // default filling value
	Values values; // map data
	Group* mapGroup; // resize group, when one particle of the group is resized, all are
	mutable unsigned rayCount; // number of rays cast on this grid

public:
	GridMap(Group* gridMapGroup, const Value defaultValue);
	GridMap(const float resolution, const Value defaultValue, Group* gridMapGroup = 0);
	GridMap(const float resolution, const float startX, const float startY, const float width, const float height, const Value defaultValue, Group* gridMapGroup = 0);
	GridMap(const std::string &pgmFileName, const float resolution, const Value defaultValue, Group* gridMapGroup = 0);
	
	GridMap(const GridMap& that);
	GridMap& operator=(const GridMap& that);
	
	~GridMap();

public:
	// coordinate transform and bound check
	Value& atInternalCoord(const int x, const int y);
	Value atInternalCoord(const int x, const int y) const ;
	Vector fromInternalCoord(const int x, const int y) const;
	Vector fromInternalCoord(const float x, const float y) const;
	void toInternalCoord(const Vector& pos, int& internalX, int& internalY) const;
	void toInternalCoordSuperSampled(const Vector& pos, const int superSampleRes, int& internalX, int& internalY) const;
	static inline Value saturatedValueFromInt(int v) { if (v < -32768) return -32768; else if (v > 32767) return 32767; else return Value(v); }
	
	// getters
	// internal
	inline int getInternalStartX() const { return startX; }
	inline int getInternalStartY() const { return startY; }
	inline int getInternalWidth() const { return width; }
	inline int getInternalHeight() const { return height; }
	inline float getResolution() const { return resolution; }
	// external
	inline Vector getMinCoord() const { return fromInternalCoord(0, 0); }
	inline Vector getMaxCoord() const { return fromInternalCoord(width-1, height-1); }
	inline Vector getSize() const { return getMaxCoord() - getMinCoord(); }
	
	// values and bound accessors
	//! Return value at pos, sampled without any interpolation
	Value getValueNearest(const Vector& pos) const;
	//! Return value at pos, with bi-linear interpolation
	Value getValue(const Vector& pos) const;
	//! Return slope at pos, with bi-linear interpolated
	Vector getSlope(const Vector& pos, const float limit = 65536.f) const;
	bool isWithinBoundsInternal(const int x, const int y) const;
	bool isWithinBounds(const Vector& pos) const;
	//! Set value at pos, sampled without any interpolation
	void setNearestValue(const Vector& pos, const Value value);
	//! Add delta ta value at pos, saturated but sampled without any interpolation
	void addNearestValueSaturated(const Vector& pos, const int delta);
	//! Extend to fit position
	void extendToFit(const Vector& pos);
	
	// conversion to ROS
	//! Return a ROS nav_msgs/OccupancyGrid message
	nav_msgs::OccupancyGrid toOccupancyGrid(const std::string& frame_id, const GridMap* knownMap = 0) const;
	
	// local operations on map; safe, extends map when required
	template <typename F>
	void lineScan(const Vector& start, const Vector& stop, F& functor, const Value& value = 0);
	
	template <typename F>
	void lineScan(const Vector& start, const Vector& stop, F& functor, const Value texture[], const unsigned textureLength);
	
	// global operations on map
	//! Invert the values of every pixel, if the value is -32768, map it to +32767
	void invert();
	//! Apply threshold to every value of the map; if it is below threshold, set to lowValue, else set to highValue
	void threshold(const Value threshold, const Value lowValue, const Value highValue);
	//! Dilate the map of amount pixels, taking into account the four N,S,W,E pixels around each pixel
	void dilate4(const unsigned amount, const unsigned delta = 0);
	//! Dilate the map of amount pixels, taking into account all the eight pixels around each pixel
	void dilate8(const unsigned amount, const unsigned delta = 0);
	//! Erode the map of amount pixels, taking into account the four N,S,W,E pixels around each pixel
	void erode4(const unsigned amount, const unsigned delta = 0);
	//! Erode the map of amount pixels, taking into account all the eight pixels around each pixel
	void erode8(const unsigned amount, const unsigned delta = 0);
	//! A label has a center of mass, and an area
	typedef boost::tuple<unsigned, Vector, unsigned> Label;
	//! All found labels
	typedef std::vector<Label> Labels;
	//! Label this map, if value of a pixel is bigger then threshold, take into consideration for label. Return the vector of labels center of mass
	Labels labelize(const Value threshold);
	//! A pair of vectors
	typedef std::pair<Vector, Vector> VectorPair;
	//! Return approximately the closest points between two areas
	VectorPair closestPoints(const Label& area0, const Label& area1, const unsigned monteCarloSteps = 100) const;
	//! Build a gradient to a goal, and avoid obstacles (see Philippsen TR E* 06)
	/*! Assume that this contains the speed map, where 0 corresponds to obstacles and 32767 maximum speed.
		Gradients have 64 subsampling resolution. */
	GridMap buildGradient(const Vector& goal, bool &isSuccess) const;
	//! Map to map generic unary operation
	template<typename Op>
	void unaryOperation(Op& op)
	{
		for (Values::iterator it = values.begin(); it != values.end(); ++it)
			op(*it);
	}
	//! Map to map generic unary operation, overload for const op
	template<typename Op>
	void unaryOperation(const Op& op)
	{
		for (Values::iterator it = values.begin(); it != values.end(); ++it)
			op(*it);
	}
	//! Map to map generic binary operation
	template<typename Op>
	void binaryOperation(const GridMap& that, Op& op)
	{
		assert(that.width == width);
		assert(that.height == height);
		for (size_t i = 0; i < values.size(); ++i)
			values[i] = op(values[i], that.values[i]);
	}
	//! Map to map generic binary operation, overload for const op
	template<typename Op>
	void binaryOperation(const GridMap& that, const Op& op)
	{
		assert(that.width == width);
		assert(that.height == height);
		for (size_t i = 0; i < values.size(); ++i)
			values[i] = op(values[i], that.values[i]);
	}
	
	// performance measurement and stats
	inline void rayCountReset() { rayCount = 0; }
	inline unsigned getRayCount() const { return rayCount; }
	float getInformationQuantity() const;
	void toPGMFile(const std::string& fileName, const int divisorToPGM = 256) const;

protected:
	void initiateMapGroup();
	bool extendMap(int xMin, int yMin, int xMax, int yMax);
	void extendMapInternal(int deltaStartX, int deltaStartY, int newWidth, int newHeight);
	void dilateN(const unsigned amount, const int lookup[][2], const size_t lookupSize, const unsigned delta);
	void erodeN(const unsigned amount, const int lookup[][2], const size_t lookupSize, const unsigned delta);
};

#endif // __GRID_MAP_H
