package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.GroundProfile3D;

public class BumpyGroundProfileTest extends GroundProfileTest
{
   public GroundProfile3D getGroundProfile()
   {
      return new BumpyGroundProfile();
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
      return 0.0;
   }
}
