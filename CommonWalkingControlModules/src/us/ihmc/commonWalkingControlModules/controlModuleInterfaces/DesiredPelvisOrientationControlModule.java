package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.utilities.math.geometry.Orientation;

public interface DesiredPelvisOrientationControlModule
{
   public abstract Orientation getDesiredPelvisOrientationSingleSupport(RobotSide robotSide);

   public abstract Orientation getDesiredPelvisOrientationDoubleSupport();
}
