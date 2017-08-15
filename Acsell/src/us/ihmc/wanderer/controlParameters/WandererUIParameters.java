package us.ihmc.wanderer.controlParameters;

import us.ihmc.wanderer.parameters.WandererPhysicalProperties;
import us.ihmc.wholeBodyController.UIParameters;

public class WandererUIParameters implements UIParameters
{

   @Override
   public double getAnkleHeight()
   {
      return WandererPhysicalProperties.ankleHeight;
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

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return 0;
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0;
   }
}
