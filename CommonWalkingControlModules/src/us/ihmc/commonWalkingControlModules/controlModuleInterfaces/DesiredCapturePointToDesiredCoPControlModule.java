package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface DesiredCapturePointToDesiredCoPControlModule
{
   /**
    * Computes the desired CoP based on a desired capture point
    *
    * @param supportLeg null means double support
    * @param bipedSupportPolygons
    * @param capturePoint
    * @param desiredVelocity
    * @param desiredCapturePoint
    * @param desiredCapturePointVelocity 
    */
   public abstract FramePoint2d computeDesiredCoPSingleSupport(RobotSide supportLeg, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
           FrameVector2d desiredVelocity, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity);

   public abstract FramePoint2d computeDesiredCoPDoubleSupport(OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d capturePoint,
           FrameVector2d desiredVelocity, FramePoint2d desiredCapturePoint, FrameVector2d desiredCapturePointVelocity);
}
