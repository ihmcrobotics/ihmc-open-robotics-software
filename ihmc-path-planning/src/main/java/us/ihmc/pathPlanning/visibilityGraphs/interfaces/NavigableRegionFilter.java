package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import java.util.List;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface NavigableRegionFilter
{
   boolean isPlanarRegionNavigable(PlanarRegion query, List<PlanarRegion> allOtherRegions);
}
