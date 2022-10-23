#include "line_tools.h"


float LineTools::GetDistanceFromLine2D(const Eigen::Vector3f& line, const Eigen::Vector2f& point)
{
   float dist = abs(line.head(2).dot(point) + line.z()) / line.head(2).norm();
   return dist;
}

Eigen::Vector3f LineTools::GetLineFromTwoPoints2D(const Eigen::Vector2f& start, const Eigen::Vector2f& end)
{
   Eigen::Vector2f normal = end - start;
   float x = normal.x();
   normal.x() = -normal.y();
   normal.y() = x;
   float c = -normal.dot(start);
   Eigen::Vector3f line;
   line << normal, c;
   return line;
}

Eigen::Vector2f LineTools::Intersect(const Line2D& l1, const Line2D& l2)
{
   /* x = (b1c2 - b2c1)/(a1b2 - a2b1)
    * y = (a1c2 - a2c1)/(a2b1 - a1b2)
    * */

   Eigen::Vector2f intersection;
   if((l1(0) * l2(1) - l2(0) * l1(1)) != 0)
   {
      intersection.x() = (l1(1) * l2(2) - l2(1) * l1(2)) / (l1(0) * l2(1) - l2(0) * l1(1));
      intersection.y() = (l1(0) * l2(2) - l2(0) * l1(2)) / (l2(0) * l1(1) - l1(0) * l2(1));
   }
   else
   {
      intersection.x() = NAN;
      intersection.y() = NAN;
   }

   printf("Result: %.2lf, %.2lf\n", intersection.x(), intersection.y());

   return intersection;
}


