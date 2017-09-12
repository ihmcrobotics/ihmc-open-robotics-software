package us.ihmc.valkyrie.parameters;

import us.ihmc.wholeBodyController.UIParameters;

public class ValkyrieUIParameters implements UIParameters
{

   @Override
   public double getAnkleHeight()
   {
      return ValkyriePhysicalProperties.ankleHeight;
   }

   /** @inheritDoc */
   @Override
   public double getSpineYawLimit()
   {
      return Math.PI / 4.0;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchUpperLimit()
   {
      return -0.13;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return 0.666;
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
      return true;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(ValkyriePhysicalProperties.footForward * ValkyriePhysicalProperties.footForward + 0.25 * ValkyriePhysicalProperties.footWidth * ValkyriePhysicalProperties.footWidth);
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.8157;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultTrajectoryTime()
   {
      return 2.0;
   }
}
