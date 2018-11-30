package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.robotics.geometry.PlanarRegion;

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
      return 0.75;
   }

   @Override
   public double getExtrusionDistance()
   {
      return 0.4;
   }

   @Override
   public double getExtrusionDistanceIfNotTooHighToStep()
   {
      return 0.05;
   }

   @Override
   public double getTooHighToStepDistance()
   {
      return 0.4;
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
   public NavigableExtrusionDistanceCalculator getNavigableExtrusionDistanceCalculator()
   {
      return new NavigableExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
         {
            return 0.02;
         }
      };
   }
}
