package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;

public class DefaultVisibilityGraphParameters implements VisibilityGraphsParameters
{
   @Override
   public double getMaxInterRegionConnectionLength()
   {
      return 0.55;
   }

   @Override
   public double getNormalZThresholdForAccessibleRegions()
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
      return 0.2;
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
