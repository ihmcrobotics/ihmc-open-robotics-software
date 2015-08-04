package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CapturePointCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   public static void computeCapturePoint(FramePoint2d capturePointToPack, FramePoint2d centerOfMassInWorld, FrameVector2d centerOfMassVelocityInWorld, double omega0)
   {
      centerOfMassInWorld.checkReferenceFrameMatch(worldFrame);
      centerOfMassVelocityInWorld.checkReferenceFrameMatch(worldFrame);
      
      capturePointToPack.setToZero(worldFrame);
      capturePointToPack.set(centerOfMassVelocityInWorld);
      capturePointToPack.scale(1.0 / omega0);
      capturePointToPack.add(centerOfMassInWorld);
   }
   
   public static double computeOmega0ConstantHeight(double g, double z)
   {
      return Math.sqrt(g / z);
   }
}
