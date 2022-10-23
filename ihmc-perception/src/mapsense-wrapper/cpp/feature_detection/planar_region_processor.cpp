#include "planar_region_processor.h"

void PlanarRegionProcessor::extractRealPlanes(std::vector<std::shared_ptr<PlanarRegion>> planarRegionList)
{
   bool exists = false;
   this->planes.clear();
   Eigen::Vector4f regionSupportPlane, uniquePlane;

   for (int i = 0; i < planarRegionList.size(); i++)
   {
      std::shared_ptr<PlanarRegion> region = planarRegionList[i];
      regionSupportPlane << region->GetNormal(), -region->GetNormal().dot(region->GetCenter());
      for (int index : planes)
      {
         std::shared_ptr<PlanarRegion> plane = planarRegionList[index];
         uniquePlane << plane->GetNormal(), -plane->GetNormal().dot(plane->GetCenter());
         if ((regionSupportPlane - uniquePlane).norm() < 0.1 && (plane->GetCenter() - region->GetCenter()).norm() < 0.3)
         {
            exists = true;
         }
      }
      if (!exists)
         planes.emplace_back(i);
   }
   printf("Total Unique Surfaces Found: %d/%d\n", planes.size(), planarRegionList.size());
   for(int planeIndex : planes)
   {
      printf("Unique Surface: %s\n", planarRegionList[planeIndex]->toString().c_str());
   }
}

void PlanarRegionProcessor::filterPlanarRegions(std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList)
{

}