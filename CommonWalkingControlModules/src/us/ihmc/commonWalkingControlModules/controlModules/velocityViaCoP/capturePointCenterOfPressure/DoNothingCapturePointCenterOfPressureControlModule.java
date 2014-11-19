package us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.capturePointCenterOfPressure;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.CapturePointCenterOfPressureControlModule;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;

public class DoNothingCapturePointCenterOfPressureControlModule implements CapturePointCenterOfPressureControlModule
{
   private ReferenceFrame midFeetZUpFrame;

   public DoNothingCapturePointCenterOfPressureControlModule(ReferenceFrame midFeetZUpFrame)
   {
      this.midFeetZUpFrame = midFeetZUpFrame;
   }
   
   public void controlDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint currentCapturePoint, FramePoint desiredCapturePoint,
                                    FramePoint centerOfMassPositionInWorldFrame, FrameVector2d desiredVelocity, FrameVector2d currentVelocity)
   {
   }

   public void controlSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons supportPolygons, FramePoint currentCapturePoint, FrameVector2d desiredVelocity,
                                    FrameLineSegment2d guideLine, FramePoint desiredCapturePoint, FramePoint centerOfMassPositionInZUpFrame,
                                    FrameVector2d currentVelocity)
   {
   }

   public void packDesiredCenterOfPressure(FramePoint desiredCenterOfPressureToPack)
   {
      desiredCenterOfPressureToPack.setToZero(midFeetZUpFrame);
   }

}
