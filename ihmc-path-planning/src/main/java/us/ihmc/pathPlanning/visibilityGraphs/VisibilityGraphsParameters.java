package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools.ExtrusionDistanceCalculator;

public interface VisibilityGraphsParameters
{
   public int getNumberOfForcedConnections();

   public double getMinimumConnectionDistanceForRegions();

   public double getNormalZThresholdForAccessibleRegions();

   public double getNormalZThresholdForPolygonObstacles();

   public double getExtrusionDistance();

   public double getExtrusionDistanceIfNotTooHighToStep();

   public double getTooHighToStepDistance();

   public double getClusterResolution();

   default double getExplorationDistanceFromStartGoal()
   {
      return Double.POSITIVE_INFINITY;
   }

   default double getPlanarRegionMinArea()
   {
      return 0.0;
   }
   
   default int getPlanarRegionMinSize()
   {
      return 0;
   }

   default double getMaxDistanceToProjectStartGoalToClosestRegion()
   {
      return 0.15;
   }

   default ExtrusionDistanceCalculator getExtrusionDistanceCalculator()
   {
      return (c, p, h) -> getExtrusionDistance() + c.getAdditionalExtrusionDistance();
   }
}
