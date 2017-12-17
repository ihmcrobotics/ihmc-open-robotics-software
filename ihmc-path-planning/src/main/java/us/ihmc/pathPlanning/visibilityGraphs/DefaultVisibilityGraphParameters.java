package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools.ExtrusionDistanceCalculator;

public class DefaultVisibilityGraphParameters implements VisibilityGraphsParameters
{
   @Override
   public int getNumberOfForcedConnections()
   {
      return 5;
   }

   @Override
   public double getMinimumConnectionDistanceForRegions()
   {
      return 0.55;
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return 0.8;
   }

   @Override
   public double getNormalZThresholdForPolygonObstacles()
   {
      return 0.8;
   }

   @Override
   public double getExtrusionDistance()
   {
      return 0.8;
   }

   @Override
   public double getExtrusionDistanceIfNotTooHighToStep()
   {
      return 0.1;
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return 0.6;
   }

   @Override
   public double getClusterResolution()
   {
      return 0.2;
   }

   @Override
   public double getPlanarRegionMinArea()
   {
      return 0.05;
   }

   @Override
   public int getPlanarRegionMinSize()
   {
      return 2;
   }

   @Override
   public ExtrusionDistanceCalculator getExtrusionDistanceCalculator()
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
