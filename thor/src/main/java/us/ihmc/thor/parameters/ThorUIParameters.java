package us.ihmc.thor.parameters;

import us.ihmc.wholeBodyController.UIParameters;

public class ThorUIParameters implements UIParameters
{
   // Limits // FIXME: 11/19/16
   private final double spineYawLimit = Math.PI / 4.0;
   private final double spinePitchUpperLimit = 0.4;
   private final double spinePitchLowerLimit = -0.1;    // -math.pi / 6.0;
   private final double spineRollLimit = Math.PI / 4.0;

   @Override
   public double getAnkleHeight()
   {
      return ThorPhysicalProperties.ankleHeight;
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
      return (1 + 0.3) * 2 * Math.sqrt(ThorPhysicalProperties.footForwardForControl * ThorPhysicalProperties.footForwardForControl + 0.25 * ThorPhysicalProperties.footWidthForControl * ThorPhysicalProperties.footWidthForControl);
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.623;
   }
}
