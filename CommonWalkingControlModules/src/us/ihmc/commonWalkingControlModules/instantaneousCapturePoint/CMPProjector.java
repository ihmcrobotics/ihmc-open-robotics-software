package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;

public abstract class CMPProjector
{
   public abstract void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2d finalDesiredCapturePoint, FramePoint2d desiredCMP);

   public abstract boolean getWasCMPProjected();

   public void projectCMPIntoSupportPolygonUsingFinalDesired(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2d finalDesiredCapturePoint, FramePoint2d desiredCMP)
   {
   }
}
