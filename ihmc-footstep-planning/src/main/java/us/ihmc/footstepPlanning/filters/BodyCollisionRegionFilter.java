package us.ihmc.footstepPlanning.filters;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface BodyCollisionRegionFilter
{
   boolean isPlanarRegionCollidable(PlanarRegion query, double groundHeight, double minHeight, double maxHeight);
}
