#pragma once

#include "vector"
#include "stack"
#include "iostream"
#include "Eigen/Core"

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BoolDynamicMatrix;

struct BoundingBox
{
   public:
      BoundingBox(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
      {
         sizeX = fabs(p2.x() - p1.x());
         sizeY = fabs(p2.y()- p1.y());
         cx = (p1.x() + p2.x()) / 2.0f;
         cy = (p1.y() + p2.y()) / 2.0f;
      }

      BoundingBox(float cx, float cy, float lx, float ly)
      {
         sizeX = lx;
         sizeY = ly;
         this->cx = cx;
         this->cy = cy;
      }

      BoundingBox(const std::vector<Eigen::Vector2f>& points)
      {
         float minX = MAXFLOAT;
         float minY = MAXFLOAT;
         float maxX = -MAXFLOAT;
         float maxY = -MAXFLOAT;

         for (auto point: points)
         {
            if (point.x() < minX) minX = point.x();
            if (point.y() < minY) minY = point.y();
            if (point.x() > maxX) maxX = point.x();
            if (point.y() > maxY) maxY = point.y();
         }

         sizeX = fabs(maxX - minX);
         sizeY = fabs(maxY- minY);
         cx = (minX + maxX) / 2.0f;
         cy = (minY + maxY) / 2.0f;

      }

      void Print() const
      {
         printf("Box: cx:%.2lf, cy:%.2lf, size_x:%.2lf, size_y:%.2lf\n", cx, cy, sizeX, sizeY);
      }

      float GetSizeX() const { return sizeX;}
      float GetSizeY() const { return sizeY;}
      float GetCenterX() const { return cx;}
      float GetCenterY() const { return cy;}
      float GetMinX() const { return fmin(cx - sizeX/2.0f,cx + sizeX/2.0f);}
      float GetMaxX() const { return fmax(cx - sizeX/2.0f,cx + sizeX/2.0f);}
      float GetMinY() const { return fmin(cy - sizeY/2.0f,cy + sizeY/2.0f);}
      float GetMaxY() const { return fmax(cy - sizeY/2.0f,cy + sizeY/2.0f);}
      float GetArea() const { return fabs(sizeX * sizeY);}


   private:
      float cx = 0;
      float cy = 0;
      float sizeX = 0;
      float sizeY = 0;
};

class HullTools
{
   public:
      static void GetParametricCurve(std::vector<Eigen::Vector2f> points, uint8_t m, Eigen::MatrixXf& params);

      static bool CheckPointInside(const std::vector<Eigen::Vector2f>& hull, const Eigen::Vector2f& point);

      static float ComputeWindingNumber(const std::vector<Eigen::Vector2f>& concaveHull, const Eigen::Vector2f& point);


      static void CanvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, std::vector<Eigen::Vector2f>& concaveHull,
                                    Eigen::Vector2f start, float scale);

      static std::vector<Eigen::Vector2f> GrahamScanConvexHull(std::vector<Eigen::Vector2f> points);

      static std::vector<Eigen::Vector2f> CanvasApproximateConcaveHull(std::vector<Eigen::Vector2f> points, uint16_t windowHeight, uint16_t windowWidth);

      static float ComputeBoundingBoxIoU(const BoundingBox& box1, const BoundingBox& box2);

      static std::vector<Eigen::Vector2f> CalculateIntersection(const std::vector<Eigen::Vector2f>& points1, const std::vector<Eigen::Vector2f>& points2);

      static std::vector<Eigen::Vector2f> CalculateUnion(const std::vector<Eigen::Vector2f>& hull1, const std::vector<Eigen::Vector2f>& hull2);
};

