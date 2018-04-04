package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;

public abstract class CMPProjector
{
   public abstract void projectCMPIntoSupportPolygonIfOutside(FramePoint2D capturePoint, FrameConvexPolygon2D supportPolygon,
         FramePoint2D finalDesiredCapturePoint, FramePoint2D desiredCMP);

   public abstract boolean getWasCMPProjected();
}
