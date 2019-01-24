package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;

public class GenericQuadrupedPointFootSnapperParameters implements PointFootSnapperParameters
{
   @Override
   public double distanceInsidePlanarRegion()
   {
      return 0.07;
   }

   @Override
   public double maximumNormalAngleFromVertical()
   {
      return 1.0;
   }
}
