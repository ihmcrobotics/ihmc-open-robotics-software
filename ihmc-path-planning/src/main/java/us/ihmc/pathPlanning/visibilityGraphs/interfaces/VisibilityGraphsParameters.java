package us.ihmc.pathPlanning.visibilityGraphs.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;

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
      return new ExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Cluster clusterInExtrusion, Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            if (obstacleHeight < 0.0)
               return 0.0;
            if (obstacleHeight <= getTooHighToStepDistance())
            {
               double alpha = obstacleHeight / getTooHighToStepDistance();
               return EuclidCoreTools.interpolate(getExtrusionDistanceIfNotTooHighToStep(), getExtrusionDistance(), alpha);
            }
            return getExtrusionDistance();
         }
      };
   }
}
