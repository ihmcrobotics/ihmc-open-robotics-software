package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface PlanarRegionFilter
{
   boolean isPlanarRegionRelevant(PlanarRegion planarRegion);
}
