package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.GroundProfile3D;


public class SingleStepGroundProfileTest extends GroundProfileTest
{
   public GroundProfile3D getGroundProfile()
   {
      return new SingleStepGroundProfile(-10.0, 10.0, -5.0, 5.0, 2.0, 0.2);
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
