package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public abstract class CMPProjector
{
   public abstract void projectCMPIntoSupportPolygonIfOutside(FramePoint2D capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2D finalDesiredCapturePoint, FramePoint2D desiredCMP);

   public abstract boolean getWasCMPProjected();
}
