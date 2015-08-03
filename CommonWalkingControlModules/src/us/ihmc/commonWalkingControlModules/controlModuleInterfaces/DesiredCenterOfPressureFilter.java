package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredCenterOfPressureFilter
{
   public abstract FramePoint2d filter(FramePoint2d desiredCenterOfPressure, RobotSide supportLeg);
}
