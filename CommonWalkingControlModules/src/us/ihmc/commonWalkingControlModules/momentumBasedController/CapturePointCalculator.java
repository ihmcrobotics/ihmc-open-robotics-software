package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CapturePointCalculator
{
   public static FramePoint2d computeCapturePoint(FramePoint centerOfMass, FrameVector centerOfMassVelocity, double omega0)
   {
      centerOfMass.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassVelocity.changeFrame(ReferenceFrame.getWorldFrame());

      FramePoint2d ret = centerOfMass.toFramePoint2d();
      FrameVector2d velocityPart = centerOfMassVelocity.toFrameVector2d();
      velocityPart.scale(1.0 / omega0);
      ret.add(velocityPart);

      return ret;
   }
   
   public static double computeOmega0ConstantHeight(double g, double z)
   {
      return Math.sqrt(g / z);
   }
}
