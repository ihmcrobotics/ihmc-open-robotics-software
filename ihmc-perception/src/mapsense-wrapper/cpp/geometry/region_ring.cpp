#pragma once

#include "region_ring.h"

RegionRing::RegionRing(int id)
{
   this->id = id;
}

void RegionRing::insertBoundaryVertex(Eigen::Vector3f pos)
{
   this->boundaryVertices.push_back(pos);
}

void RegionRing::insertBoundaryIndex(Eigen::Vector2i pos)
{
   this->boundaryIndices.push_back(pos);
}

int RegionRing::getNumOfVertices() const
{
   return boundaryVertices.size();
}

int RegionRing::getId() const
{
   return id;
}
