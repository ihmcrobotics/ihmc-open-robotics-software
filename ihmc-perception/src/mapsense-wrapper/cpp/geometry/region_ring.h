#pragma once

#include <Eigen/Dense>
#include "vector"

class RegionRing
{
   public:
      std::vector<Eigen::Vector3f> boundaryVertices;
      std::vector<Eigen::Vector2i> boundaryIndices;
      int id;

      RegionRing(int id);

      void insertBoundaryVertex(Eigen::Vector3f pos);

      void insertBoundaryIndex(Eigen::Vector2i pos);

      int getNumOfVertices() const;

      int getId() const;
};

