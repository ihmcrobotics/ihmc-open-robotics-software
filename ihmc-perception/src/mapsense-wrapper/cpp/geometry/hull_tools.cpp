#include "hull_tools.h"
#include "Eigen/Dense"

std::vector<Eigen::Vector2f> HullTools::CalculateIntersection(const std::vector<Eigen::Vector2f>& points1, const std::vector<Eigen::Vector2f>& points2)
{
   std::vector<Eigen::Vector2f> intersectionHull;
   bool previousIsInside = false;
   bool currentIsInside = false;
   for(int i = 1; i<points2.size(); i++)
   {
      auto currentPoint = points2[i-1];
      auto previousPoint = points2[i];

      if(!previousIsInside && currentIsInside)

         //      currentState = (windingNumber > 0.0);
         if (previousIsInside == false && currentIsInside == true)
         {

         }
   }

   return intersectionHull;

}

std::vector<Eigen::Vector2f> HullTools::CalculateUnion(const std::vector<Eigen::Vector2f>& hull1, const std::vector<Eigen::Vector2f>& hull2)
{
   std::vector<Eigen::Vector2f> unionHull;
   for(int i = 0; i < hull2.size(); i++)
   {
      if (!CheckPointInside(hull1, hull2[i]))
         unionHull.emplace_back(hull2[i]);
   }

   for(int i = 0; i < hull1.size(); i++)
   {
      if (!CheckPointInside(hull2, hull1[i]))
         unionHull.emplace_back(hull1[i]);
   }

   return unionHull;
}

float HullTools::ComputeBoundingBoxIoU(const BoundingBox& box1, const BoundingBox& box2)
{
   float intersectionMinX = fmax(box1.GetMinX(), box2.GetMinX());
   float intersectionMinY = fmax(box1.GetMinY(), box2.GetMinY());
   float intersectionMaxX = fmin(box1.GetMaxX(), box2.GetMaxX());
   float intersectionMaxY = fmin(box1.GetMaxY(), box2.GetMaxY());
   float intersectionArea = 0;
   if(intersectionMinX < intersectionMaxX && intersectionMinY < intersectionMaxY)
   {
      intersectionArea = (intersectionMaxX - intersectionMinX) * (intersectionMaxY - intersectionMinY);
   }
   float unionArea = box1.GetArea() + box2.GetArea() - intersectionArea;
   return intersectionArea / unionArea;
}

bool HullTools::CheckPointInside(const std::vector<Eigen::Vector2f>& hull, const Eigen::Vector2f& point)
{
   return (fabs(ComputeWindingNumber(hull, point)) > 2.0f);
}

void printHull(std::stack<int> convexHull, std::vector<Eigen::Vector2f> points)
{
   while (!convexHull.empty())
   {
      Eigen::Vector2f p = points[convexHull.top()];
      std::cout << "(" << p.x() << ", " << p.y() << ")";
      convexHull.pop();
   }
   std::cout << std::endl;
}

std::vector<Eigen::Vector2f> GetConvexHullPoints(std::stack<int> indices, std::vector<Eigen::Vector2f> points)
{
   std::vector<Eigen::Vector2f> convexHull;
   while (!indices.empty())
   {
      convexHull.emplace_back(points[indices.top()]);
      indices.pop();
   }
   return convexHull;
}

int nextToTop(std::stack<int> S)
{
   int top = S.top();
   S.pop();
   int res = S.top();
   S.push(top);
   return res;
}

int orientation(Eigen::Vector2f p, Eigen::Vector2f q, Eigen::Vector2f r)
{
   Eigen::Vector3f pq;
   Eigen::Vector3f qr;
   pq << (q - p), 0;
   qr << (r - q), 0;
   float val = pq.cross(qr).z();
   float normalizedInnerProduct = pq.normalized().dot(qr.normalized());
   if (val == 0)
      return 0;  // colinear
   return (val > 0 || (normalizedInnerProduct > 1)) ? 1 : 2;
}

void swap(Eigen::Vector2f& a, Eigen::Vector2f& b)
{
   Eigen::Vector2f temp = a;
   a = b;
   b = temp;
}

void printCanvas(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos)
{
   for (int i = 0; i < canvas.rows(); i++)
   {
      for (int j = 0; j < canvas.cols(); j++)
      {
         if (canvas(i, j) == 1)
            printf("o ");
         else if (windowPos.x() != -1 && windowPos.y() != -1 && i > windowPos.x() - 3 && i < windowPos.x() + 3 && j > windowPos.y() - 3 &&
                  j < windowPos.y() + 3)
            printf("X ");
         else
            printf(". ");
      }
      printf("\n");
   }
}

int getVisitedCount(BoolDynamicMatrix& visited, int x, int y)
{
   int count = 0;
   for (int i = -3; i < 3; i++)
   {
      for (int j = -3; j < 3; j++)
      {
         if (x + i < visited.rows() - 1 && x + i > 1 && y + j < visited.cols() - 1 && y + j > 1)
         {
            if (visited(x + i, y + j) == 1)
            {
               count += 1;
            }
         }
      }
   }
   //   printf("Visited:(%d)\n", count);
   return count;
}

std::vector<Eigen::Vector2f> HullTools::GrahamScanConvexHull(std::vector<Eigen::Vector2f> points)
{
   Eigen::Vector2f minY(10000, 10000);
   int minYIndex = 0;
   for (int i = 0; i < points.size(); i++)
      if (points[i].y() < minY.y())
      {
         minY = points[i];
         minYIndex = i;
      }

   swap(points[0], points[minYIndex]);
   minY = points[0];
   sort(points.begin() + 1, points.end(), [=](Eigen::Vector2f a, Eigen::Vector2f b)
   {
      return atan2(a.x() - minY.x(), a.y() - minY.y()) > atan2(b.x() - minY.x(), b.y() - minY.y());
   });

   std::stack<int> convexHull;
   convexHull.push(0);
   convexHull.push(1);
   convexHull.push(2);

   std::vector<int> popels;
   for (int i = 3; i < points.size(); i++)
   {
      while (convexHull.size() > 1 && orientation(points[nextToTop(convexHull)], points[convexHull.top()], points[i]) != 1)
      {
         convexHull.pop();
      }
      convexHull.push(i);
   }

   return GetConvexHullPoints(convexHull, points);
}


bool loopComplete(Eigen::Vector2f current, Eigen::Vector2f start, int concaveHullSize)
{
   return (current - start).norm() < 0.1 && concaveHullSize > 10;
}

float CheckConcavity(std::vector<Eigen::Vector2f> concaveHull, Eigen::Vector2f node)
{
   Eigen::Vector2f candidate(node.x(), node.y());
   Eigen::Vector2f current = concaveHull.rbegin()[0];
   Eigen::Vector2f previous = concaveHull.rbegin()[1];
   return ((current - previous).normalized()).dot((candidate - current).normalized());
}

void HullTools::CanvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, std::vector<Eigen::Vector2f>& concaveHull, Eigen::Vector2f start,
                                  float scale)
{
   if (visited(x, y) || loopComplete(concaveHull.rbegin()[0], Eigen::Vector2f((start.x() - canvas.rows()/2) / scale, (start.y() - canvas.cols()/2) / scale), concaveHull.size()))
      return;

   visited(x, y) = 1;

   if(concaveHull.size() < 2 ||
      CheckConcavity(concaveHull, Eigen::Vector2f(((float) x - canvas.rows() / 2) / (float) scale, ((float) y - canvas.cols() / 2) / (float) scale)) > -0.9)
   {
      concaveHull.emplace_back(Eigen::Vector2f(((float) x - canvas.rows()/2) / (float) scale  , ((float)y - canvas.cols()/2) / (float)scale));
   }


   for (int i = -3; i < 3; i++)
   {
      for (int j = -3; j < 3; j++)
      {
         if (x + i < canvas.rows() - 1 && x + i > 1 && y + j < canvas.cols() - 1 && y + j > 1)
         {
            if (canvas(x + i, y + j) == 1)
            {
               if (getVisitedCount(visited, x + i, y + j) > 24)
                  continue;
               CanvasBoundaryDFS(x + i, y + j, canvas, visited, concaveHull, start,scale);
            }
            visited(x + i, y + j) = 1;
         }
      }
   }
}

/* Novel algorithm for approximating concave hull by drawing a list of 2D points
 * on a canvas, and traversing the hull with a moving window. */
std::vector<Eigen::Vector2f> HullTools::CanvasApproximateConcaveHull(std::vector<Eigen::Vector2f> points, uint16_t windowHeight, uint16_t windowWidth)
{
   int r = 120;
   int c = 120;
   BoolDynamicMatrix canvas(r, c);
   canvas.setZero();
   BoolDynamicMatrix visited(r, c);
   visited.setZero();


   /* Draw points on canvas using origin and bounding box dimensions. */
   int scale = 45;
   for (int i = 0; i < points.size(); i++)
   {
      canvas((int) (r / 2 + points[i].x() * scale), (int) (c / 2 + points[i].y() * scale)) = 1;
   }

   std::vector<Eigen::Vector2f> concaveHull;
   concaveHull.emplace_back(Eigen::Vector2f(points[0].x(), points[0].y()));

   /* Traverse through the boundary and extract lower vertex-count ordered concave hull. */
   CanvasBoundaryDFS((uint16_t) (r / 2 + points[0].x() * scale), (uint16_t) (c / 2 + points[0].y() * scale), canvas, visited, concaveHull,
                     Eigen::Vector2f((r / 2 + points[0].x() * scale), (c / 2 + points[0].y() * scale)), scale);


   std::vector<Eigen::Vector2f> finalConcaveHull;
   for(int i = 0; i<concaveHull.size(); i++)
      if(i % 2 == 0)
         finalConcaveHull.emplace_back(concaveHull[i]);


   printf("Original Point Set Size: %d\n", points.size());
   printf("Final Concave Hull Size: %d\n", finalConcaveHull.size());

   return finalConcaveHull;
}

float HullTools::ComputeWindingNumber(const std::vector<Eigen::Vector2f>& hull, const Eigen::Vector2f& point)
{
   float totalAngle = 0;
   for(int i = 0; i<hull.size() - 1; i++)
   {
      Eigen::Vector3f v1;
      v1 << hull[i] - point, 0;
      Eigen::Vector3f v2;
      v2 << hull[i+1] - point, 0;

      Eigen::Vector3f cross = v1.cross(v2);
      float cosim = v1.dot(v2) / (v1.norm() * v2.norm());
      float angle = fabs(acos(cosim)) * cross[2] / fabs(cross[2]);
      totalAngle += angle;
   }
   return totalAngle;
}


void HullTools::GetParametricCurve(std::vector<Eigen::Vector2f> points, uint8_t m, Eigen::MatrixXf& params)
{
   Eigen::MatrixXf A(points.size(), m + 1);
   for (int i = 0; i < points.size(); i++)
   {
      double t = (double) i / (double) 200.0;
      for (int j = 0; j < m + 1; j++)
      {
         A(i, j) = pow(t, m - j);
      }
   }
   Eigen::VectorXf b(points.size());

   /* Get parameters for X regression. */
   for (int i = 0; i < points.size(); i++)
   {
      b(i) = points[i].x();
   }
   Eigen::VectorXf params_x(m);
   params_x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

   /* Get parameters for Y regression. */
   for (int i = 0; i < points.size(); i++)
   {
      b(i) = points[i].y();
   }
   Eigen::VectorXf params_y(m);
   params_y = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

   /* Collect params into a std::vector to be returned. */
   params.row(0) = params_x;
   params.row(1) = params_y;
}