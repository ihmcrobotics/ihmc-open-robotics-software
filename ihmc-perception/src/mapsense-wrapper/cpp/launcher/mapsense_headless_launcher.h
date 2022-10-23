#pragma once

#include "planar_region_calculator.h"

class MapsenseHeadlessLauncher
{
   public:

      MapsenseHeadlessLauncher(int argc, char **argv);

      void update();

      ApplicationState appState;
      PlanarRegionCalculator *_regionCalculator;
      OpenCLManager *_openCLManager;
};

