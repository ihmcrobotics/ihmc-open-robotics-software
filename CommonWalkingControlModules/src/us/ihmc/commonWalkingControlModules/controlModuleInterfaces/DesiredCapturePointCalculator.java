package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.robotSide.RobotSide;

public interface DesiredCapturePointCalculator
{
   
   public abstract FramePoint2d computeDesiredCapturePointSingleSupport(RobotSide supportLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity, SingleSupportCondition singleSupportCondition);

   public abstract FramePoint2d computeDesiredCapturePointDoubleSupport(RobotSide loadingLeg, BipedSupportPolygons bipedSupportPolygons, FrameVector2d desiredVelocity);
}
