package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;

public interface DesiredCenterOfPressureFilter
{
   public abstract FramePoint2d filter(FramePoint2d desiredCenterOfPressure, RobotSide supportLeg);
}
