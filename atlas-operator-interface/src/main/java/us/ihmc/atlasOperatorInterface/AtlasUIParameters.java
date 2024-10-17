package us.ihmc.atlasOperatorInterface;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.humanoidOperatorInterface.UIParameters;
import us.ihmc.humanoidOperatorInterface.footstep.footstepGenerator.UIFootstepGeneratorParameters;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasUIParameters implements UIParameters
{
   private final double spineYawLimit = Math.PI / 4.0;
   private final double spinePitchUpperLimit = 0.4;
   private final double spinePitchLowerLimit = -0.1; // -math.pi / 6.0;
   private final double spineRollLimit = Math.PI / 4.0;

   private final AtlasPhysicalProperties physicalProperties;
   private final AtlasRobotVersion selectedVersion;

   public AtlasUIParameters(AtlasRobotVersion selectedVersion, AtlasPhysicalProperties physicalProperties)
   {
      this.selectedVersion = selectedVersion;
      this.physicalProperties = physicalProperties;
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.623;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(physicalProperties.getFootForwardForControl() * physicalProperties.getFootForwardForControl()
            + 0.25 * physicalProperties.getFootWidthForControl() * physicalProperties.getFootWidthForControl());
   }

   @Override
   public double getSpineYawLimit()
   {
      return spineYawLimit;
   }

   @Override
   public double getSpinePitchUpperLimit()
   {
      return spinePitchUpperLimit;
   }

   @Override
   public double getSpinePitchLowerLimit()
   {
      return spinePitchLowerLimit;
   }

   @Override
   public double getSpineRollLimit()
   {
      return spineRollLimit;
   }

   @Override
   public boolean isSpinePitchReversed()
   {
      return false;
   }

   @Override
   public double getAnkleHeight()
   {
      return Math.abs(physicalProperties.getSoleToAnkleFrameTransforms().get(RobotSide.LEFT).getTranslationZ());
   }

   @Override
   public UIFootstepGeneratorParameters getUIFootstepGeneratorParameters()
   {
      return new AtlasUIFootstepGeneratorParameters();
   }
}
