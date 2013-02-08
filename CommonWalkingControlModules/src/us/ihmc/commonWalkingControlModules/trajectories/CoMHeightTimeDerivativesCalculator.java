package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CoMHeightTimeDerivativesCalculator
{   
   private final FramePoint2d comXYPosition = new FramePoint2d(ReferenceFrame.getWorldFrame());
   private final FrameVector2d comXYVelocity = new FrameVector2d(ReferenceFrame.getWorldFrame());
   private final FrameVector2d comXYAcceleration = new FrameVector2d(ReferenceFrame.getWorldFrame());

   public void computeCoMHeightTimeDerivatives(CoMHeightTimeDerivativesData comHeightDataToPack, CoMXYTimeDerivativesData xyVelocityAndAcceleration, CoMHeightPartialDerivativesData comPartialDerivatives)
   {
      xyVelocityAndAcceleration.getCoMXYPosition(comXYPosition);
      xyVelocityAndAcceleration.getCoMXYVelocity(comXYVelocity);
      xyVelocityAndAcceleration.getCoMXYAcceleration(comXYAcceleration);
      
      double comHeight = comPartialDerivatives.getCoMHeight();
      double dzDx = comPartialDerivatives.getPartialDzDx();
      double dzDy = comPartialDerivatives.getPartialDzDy();
      double d2zDx2 = comPartialDerivatives.getPartialD2zDx2();
      double d2zDy2 = comPartialDerivatives.getPartialD2zDy2();
      double d2zDxDy = comPartialDerivatives.getPartialD2zDxDy();
      
      double xDot = comXYVelocity.getX();
      double yDot = comXYVelocity.getX();
      
      double xDDot = comXYAcceleration.getX();
      double yDDot = comXYAcceleration.getX();
      
      double comHeightVelocity = dzDx * xDot + dzDy * yDot;
      double comHeightAcceleration = d2zDx2 * xDot * xDot + dzDx * xDDot+ d2zDy2 * yDot * yDot + dzDy * yDDot;
      
      comHeightDataToPack.setComHeight(comHeight);
      comHeightDataToPack.setComHeightVelocity(comHeightVelocity);
      comHeightDataToPack.setComHeightAcceleration(comHeightAcceleration);
   }
}
