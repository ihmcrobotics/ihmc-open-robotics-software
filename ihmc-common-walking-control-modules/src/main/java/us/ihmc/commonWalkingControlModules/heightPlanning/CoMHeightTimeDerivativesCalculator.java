package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class CoMHeightTimeDerivativesCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static void computeCoMHeightTimeDerivatives(CoMHeightTimeDerivativesData comHeightDataToPack,
                                                      CoMXYTimeDerivativesData comXYTimeDerivatives,
                                                      CoMHeightPartialDerivativesDataReadOnly comPartialDerivatives)
   {
      computeCoMHeightTimeDerivatives(comHeightDataToPack,
                                      comXYTimeDerivatives.getCoMVelocity(),
                                      comXYTimeDerivatives.getCoMAcceleration(),
                                      comPartialDerivatives);
   }

   public static void computeCoMHeightTimeDerivatives(CoMHeightTimeDerivativesData comHeightDataToPack,
                                                      FrameVector3DReadOnly comVelocity,
                                                      FrameVector3DReadOnly comAcceleration,
                                                      CoMHeightPartialDerivativesDataReadOnly comPartialDerivatives)
   {
      comVelocity.checkReferenceFrameMatch(worldFrame);
      comAcceleration.checkReferenceFrameMatch(worldFrame);

      double dzDx = comPartialDerivatives.getPartialDzDx();
      double dzDy = comPartialDerivatives.getPartialDzDy();
      double d2zDx2 = comPartialDerivatives.getPartialD2zDx2();
      double d2zDy2 = comPartialDerivatives.getPartialD2zDy2();
      double d2zDxDy = comPartialDerivatives.getPartialD2zDxDy();

      double xDot = comVelocity.getX();
      double yDot = comVelocity.getY();

      double xDDot = comAcceleration.getX();
      double yDDot = comAcceleration.getY();

      double comHeightVelocity = dzDx * xDot + dzDy * yDot;
      double comHeightAcceleration = d2zDx2 * xDot * xDot + dzDx * xDDot + d2zDy2 * yDot * yDot + dzDy * yDDot;

      comHeightDataToPack.setComHeight(comPartialDerivatives.getFrameOfCoMHeight(), comPartialDerivatives.getComHeight());
      comHeightDataToPack.setComHeightVelocity(comHeightVelocity);
      comHeightDataToPack.setComHeightAcceleration(comHeightAcceleration);
   }
}
