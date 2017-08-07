package us.ihmc.steppr.controlParameters;

import us.ihmc.steppr.parameters.BonoPhysicalProperties;
import us.ihmc.wholeBodyController.UIParameters;

public class BonoUIParameters implements UIParameters
{
   /** @inheritDoc */
   @Override
   public double getAnkleHeight()
   {
      return BonoPhysicalProperties.ankleHeight;
   }

   /** @inheritDoc */
   @Override
   public double getSpineYawLimit()
   {
      return 0.0;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchUpperLimit()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double getSpineRollLimit()
   {
      return Math.PI / 4.0;
   }

   /** @inheritDoc */
   @Override
   public boolean isSpinePitchReversed()
   {
      return false;
   }

   /** @inheritDoc */
   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return 0;
   }

   /** @inheritDoc */
   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0;
   }

}
