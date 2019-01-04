package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;

public class DefaultVisibilityGraphParameters implements VisibilityGraphsParameters
{
   //TODO: JavaDoc all the methods to make them more clear.

   @Override
   public double getMaxInterRegionConnectionLength()
   {
      return 0.55;
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
   {
      return 0.5;
   }

   @Override
   public double getObstacleExtrusionDistance()
   {
      return 0.4;
   }

   @Override
   public double getObstacleExtrusionDistanceIfNotTooHighToStep()
   {
      return 0.05;
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return 0.28;
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

}
