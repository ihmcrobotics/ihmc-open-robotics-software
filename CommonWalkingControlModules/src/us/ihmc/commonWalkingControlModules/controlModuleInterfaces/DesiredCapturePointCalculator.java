package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface DesiredCapturePointCalculator
{
   public abstract FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint);

   public abstract FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity);
}
