#pragma once

#include "line_2d.h"

class LineTools
{
   public:
      static Eigen::Vector2f Intersect(const Line2D& line1, const Line2D& line2);

      static Eigen::Vector3f GetLineFromTwoPoints2D(const Eigen::Vector2f& start, const Eigen::Vector2f& end);

      static float GetDistanceFromLine2D(const Eigen::Vector3f& line, const Eigen::Vector2f& point);
};

