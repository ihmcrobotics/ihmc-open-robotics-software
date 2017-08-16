package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CoMHeightTimeDerivativesCalculator
{
   private final FramePoint2D comXYPosition = new FramePoint2D(ReferenceFrame.getWorldFrame());
   private final FrameVector2D comXYVelocity = new FrameVector2D(ReferenceFrame.getWorldFrame());
   private final FrameVector2D comXYAcceleration = new FrameVector2D(ReferenceFrame.getWorldFrame());
   private final FramePoint3D centerOfMassHeightPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

   public void computeCoMHeightTimeDerivatives(CoMHeightTimeDerivativesData comHeightDataToPack, CoMXYTimeDerivativesData comXYTimeDerivatives,
         CoMHeightPartialDerivativesData comPartialDerivatives)
   {
      comXYTimeDerivatives.getCoMXYPosition(comXYPosition);
      comXYTimeDerivatives.getCoMXYVelocity(comXYVelocity);
      comXYTimeDerivatives.getCoMXYAcceleration(comXYAcceleration);

      comPartialDerivatives.getCoMHeight(centerOfMassHeightPoint);
      double dzDx = comPartialDerivatives.getPartialDzDx();
      double dzDy = comPartialDerivatives.getPartialDzDy();
      double d2zDx2 = comPartialDerivatives.getPartialD2zDx2();
      double d2zDy2 = comPartialDerivatives.getPartialD2zDy2();
      double d2zDxDy = comPartialDerivatives.getPartialD2zDxDy();

      double xDot = comXYVelocity.getX();
      double yDot = comXYVelocity.getY();

      double xDDot = comXYAcceleration.getX();
      double yDDot = comXYAcceleration.getY();

      double comHeightVelocity = dzDx * xDot + dzDy * yDot;
      double comHeightAcceleration = d2zDx2 * xDot * xDot + dzDx * xDDot + d2zDy2 * yDot * yDot + dzDy * yDDot;

      comHeightDataToPack.setComHeight(centerOfMassHeightPoint.getReferenceFrame(), centerOfMassHeightPoint.getZ());
      comHeightDataToPack.setComHeightVelocity(comHeightVelocity);
      comHeightDataToPack.setComHeightAcceleration(comHeightAcceleration);
   }
}
