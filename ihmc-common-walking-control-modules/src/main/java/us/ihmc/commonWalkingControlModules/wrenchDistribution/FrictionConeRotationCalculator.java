package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;

public interface FrictionConeRotationCalculator
{

   public abstract double computeConeRotation(YoPlaneContactState yoPlaneContactState, int contactPointIndex);

}
