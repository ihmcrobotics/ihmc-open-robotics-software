#pragma once

#include "planar_region.h"

class PlanarRegionProcessor
{
   private:

      std::vector<int> planes;

   public:

      void extractRealPlanes(std::vector<std::shared_ptr<PlanarRegion>> planarRegionList);

      void filterPlanarRegions(std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList);
};

