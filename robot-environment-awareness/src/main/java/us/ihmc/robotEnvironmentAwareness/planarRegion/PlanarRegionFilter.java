package us.ihmc.robotEnvironmentAwareness.planarRegion;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface PlanarRegionFilter
{
   boolean isPlanarRegionRelevant(PlanarRegion planarRegion);
}
