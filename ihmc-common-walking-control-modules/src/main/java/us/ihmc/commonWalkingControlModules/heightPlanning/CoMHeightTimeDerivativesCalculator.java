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
      double d3zDx3 = 0.0;
      double d3zDy3 = 0.0;
      double d3zDx2Dy = 0.0;
      double d3zDxDy2 = 0.0;

      double xDot = comVelocity.getX();
      double yDot = comVelocity.getY();

      double xDDot = comAcceleration.getX();
      double yDDot = comAcceleration.getY();

      double xDDDot = 0.0;
      double yDDDot = 0.0;

      double comHeightVelocity = dzDx * xDot + dzDy * yDot;
      double comHeightAcceleration = d2zDx2 * xDot * xDot + dzDx * xDDot + d2zDy2 * yDot * yDot + dzDy * yDDot + 2.0 * d2zDxDy * xDot * yDot;
      double comHeightJerk =
            d3zDx3 * (xDot * xDot * xDot) + d3zDy3 * (yDot * yDot * yDot) + dzDx * xDDDot + dzDy * yDDDot + 3.0 * d3zDx2Dy * (xDot * xDot * yDot)
            + 3.0 * d3zDxDy2 * (xDot * yDot * yDot) + 3.0 * d2zDx2 * xDot * xDDot + 3.0 * d2zDy2 * yDot * yDDot + 3.0 * d2zDxDy * xDDot * yDot + xDot * yDDot;

      comHeightDataToPack.setComHeight(comPartialDerivatives.getFrameOfCoMHeight(), comPartialDerivatives.getComHeight());
      comHeightDataToPack.setComHeightVelocity(comHeightVelocity);
      comHeightDataToPack.setComHeightAcceleration(comHeightAcceleration);
      comHeightDataToPack.setComHeightJerk(comHeightJerk);
   }
}
