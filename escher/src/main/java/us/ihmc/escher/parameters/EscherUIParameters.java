package us.ihmc.escher.parameters;

import us.ihmc.wholeBodyController.UIParameters;

public class EscherUIParameters implements UIParameters
{
   // Limits // FIXME: 11/19/16
   private final double spineYawLimit = Math.PI / 4.0;
   private final double spinePitchUpperLimit = 0.4;
   private final double spinePitchLowerLimit = -0.1;    // -math.pi / 6.0;
   private final double spineRollLimit = Math.PI / 4.0;

   @Override
   public double getAnkleHeight()
   {
      return EscherPhysicalProperties.ankleHeight;
   }

   /** @inheritDoc */
   @Override
   public double getSpineYawLimit()
   {
      return spineYawLimit;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchUpperLimit()
   {
      return spinePitchUpperLimit;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return spinePitchLowerLimit;
   }

   /** @inheritDoc */
   @Override
   public double getSpineRollLimit()
   {
      return spineRollLimit;
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
      return (1 + 0.3) * 2 * Math.sqrt(EscherPhysicalProperties.footForwardForControl * EscherPhysicalProperties.footForwardForControl + 0.25 * EscherPhysicalProperties.footWidthForControl * EscherPhysicalProperties.footWidthForControl);
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.623;
   }
}
