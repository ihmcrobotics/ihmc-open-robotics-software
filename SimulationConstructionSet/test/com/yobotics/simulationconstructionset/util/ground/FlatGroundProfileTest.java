package com.yobotics.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.GroundProfile3D;

import com.yobotics.simulationconstructionset.ground.GroundProfileTest;

public class FlatGroundProfileTest extends GroundProfileTest
{
   public GroundProfile3D getGroundProfile()
   {
      return new FlatGroundProfile(-0.3);
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
