package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredCapturePointCalculator
{
   
   public abstract FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition);

   public abstract FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, OldBipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity);
}
