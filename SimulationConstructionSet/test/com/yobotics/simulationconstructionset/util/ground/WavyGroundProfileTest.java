package com.yobotics.simulationconstructionset.util.ground;

import us.ihmc.graphics3DAdapter.GroundProfile3D;

import com.yobotics.simulationconstructionset.util.ground.WavyGroundProfile;

public class WavyGroundProfileTest extends GroundProfileTest
{

   public GroundProfile3D getGroundProfile()
   {
      return new WavyGroundProfile();
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
      //TODO: 30 percent is too high. Need to actually compute the surface normal properly...
      return 0.3;
   }



}
