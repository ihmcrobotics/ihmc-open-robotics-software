package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.GroundProfile3D;

public class DoubleStepGroundProfileTest extends GroundProfileTest
{
   public GroundProfile3D getGroundProfile()
   {
      double yMin = -1.0;
      double yMax = 1.0;
      double initialElevationChangeX = 2.0;
      double finalElevationChangeX = 2.4;
      double initialElevationDifference = 0.1;
      double finalElevationDifference = 0.3;

      return new DoubleStepGroundProfile(yMin, yMax, initialElevationChangeX, finalElevationChangeX, initialElevationDifference, finalElevationDifference);
   }

   public double getMaxPercentageOfAllowableValleyPoints()
   {
      return 0.0;
   }

   public double getMaxPercentageOfAllowablePeakPoints()
   {
      return 0.0;
   }

   public double getMaxPercentageOfAllowableDropOffs()
   {
      return 0.01;
   }

}
