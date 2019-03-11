package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.filters;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface SteppableRegionFilter
{
   boolean isPlanarRegionSteppable(PlanarRegion query);
}
