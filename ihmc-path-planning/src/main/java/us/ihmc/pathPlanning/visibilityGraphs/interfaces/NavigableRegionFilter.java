package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface NavigableRegionFilter
{
   boolean isPlanarRegionNavigable(PlanarRegion query);
}
