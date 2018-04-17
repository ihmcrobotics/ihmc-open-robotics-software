package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.robotics.geometry.PlanarRegion;

public interface ObstacleRegionFilter
{
   /**
    * Whether the {@code query} is an actual obstacle when navigating the {@code navigableRegion}.
    * 
    * @param query the potential obstacle region.
    * @param navigableRegion the region in which the {@code query} might be an obstacle.
    * @return {@code true} if the {@code query} is a legit obstacle, {@code false} if it can be
    *         ignored.
    */
   boolean isRegionValidObstacle(PlanarRegion query, PlanarRegion navigableRegion);
}
