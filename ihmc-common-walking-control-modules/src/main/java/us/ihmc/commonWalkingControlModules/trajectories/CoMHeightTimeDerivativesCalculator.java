package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class CoMHeightTimeDerivativesCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public void computeCoMHeightTimeDerivatives(CoMHeightTimeDerivativesData comHeightDataToPack,
                                               CoMXYTimeDerivativesData comXYTimeDerivatives,
                                               CoMHeightPartialDerivativesData comPartialDerivatives)
   {
      FrameVector3DReadOnly comXYVelocity = comXYTimeDerivatives.getCoMVelocity();
      FrameVector3DReadOnly comXYAcceleration = comXYTimeDerivatives.getCoMAcceleration();
      comXYVelocity.checkReferenceFrameMatch(worldFrame);
      comXYAcceleration.checkReferenceFrameMatch(worldFrame);

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

      comHeightDataToPack.setComHeight(comPartialDerivatives.getFrameOfCoMHeight(), comPartialDerivatives.getComHeight());
      comHeightDataToPack.setComHeightVelocity(comHeightVelocity);
      comHeightDataToPack.setComHeightAcceleration(comHeightAcceleration);
   }
}
