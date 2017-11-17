package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;

public class ZeroConeRotationCalculator implements FrictionConeRotationCalculator
{

   @Override
   public double computeConeRotation(YoPlaneContactState yoPlaneContactState, int contactPointIndex)
   {
      return 0.0;
   }

}
