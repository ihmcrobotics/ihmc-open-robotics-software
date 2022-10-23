//
// Created by quantum on 4/24/22.
//

#ifndef MAP_SENSE_LINE2D_H
#define MAP_SENSE_LINE2D_H

#include "Eigen/Core"


/*
 * Standard Form: ax + by + c = 0
 * Data Stores: {a, b, c}
 * */

class Line2D
{
   public:
      Line2D(const Eigen::Vector3f& params) : _data(params) {}
      Line2D(const Eigen::Vector2f& point1, const Eigen::Vector2f& point2);
      Line2D(const Eigen::Vector2f& point, float slope);
      Line2D(float slope, float intercept);

      Eigen::Vector2f IntersectWith(const Line2D& line);

      Eigen::Vector3f GetData() const { return _data;}

      float& operator()(int i) { return _data(i);}
      float operator()(int i) const { return _data(i);}

   private:
      Eigen::Vector3f _data;
};

#endif //MAP_SENSE_LINE2D_H
