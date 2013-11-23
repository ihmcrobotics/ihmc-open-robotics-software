package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CapturePointCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   public static FramePoint2d computeCapturePoint(FramePoint centerOfMass, FrameVector centerOfMassVelocity, double omega0)
   {
      centerOfMass.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);
      
      FramePoint2d capturePoint = new FramePoint2d(worldFrame);
      FramePoint2d centerOfMass2d = centerOfMass.toFramePoint2d();
      FrameVector2d centerOfMassVelocity2d = centerOfMassVelocity.toFrameVector2d();
      
      computeCapturePoint(capturePoint, centerOfMass2d, centerOfMassVelocity2d, omega0);

      return capturePoint;
   }

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
