//
// Created by quantum on 4/24/22.
//

#include "line_2d.h"

Line2D::Line2D(const Eigen::Vector2f& point, float slope)
{
   float c = point.y() - slope * point.x();
   _data(0) = -slope;
   _data(1) = 1;
   _data(2) = c;
}

Line2D::Line2D(float slope, float intercept)
{
   _data(0) = -slope;
   _data(1) = 1;
   _data(2) = intercept;
}

Line2D::Line2D(const Eigen::Vector2f& point1, const Eigen::Vector2f& point2)
{
   float slope = (point1.y() - point2.y())/(point1.x() - point2.x());
   float c = point1.y() - slope * point1.x();
   _data(0) = -slope;
   _data(1) = 1;
   _data(2) = c;
}


