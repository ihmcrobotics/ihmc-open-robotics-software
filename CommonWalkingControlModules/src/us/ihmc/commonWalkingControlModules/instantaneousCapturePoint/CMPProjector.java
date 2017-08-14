package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2D;

public abstract class CMPProjector
{
   public abstract void projectCMPIntoSupportPolygonIfOutside(FramePoint2D capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2D finalDesiredCapturePoint, FramePoint2D desiredCMP);

   public abstract boolean getWasCMPProjected();
}
