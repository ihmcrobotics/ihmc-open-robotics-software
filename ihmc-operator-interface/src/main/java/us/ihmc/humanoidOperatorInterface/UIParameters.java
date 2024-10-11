package us.ihmc.humanoidOperatorInterface;

import us.ihmc.humanoidOperatorInterface.footstep.footstepGenerator.UIFootstepGeneratorParameters;
import us.ihmc.robotics.physics.RobotCollisionModel;

public interface UIParameters
{
   public abstract double getAnkleHeight();

   public abstract double pelvisToAnkleThresholdForWalking();

   public abstract double getSpineYawLimit();

   public abstract double getSpinePitchUpperLimit();

   public abstract double getSpinePitchLowerLimit();

   public abstract double getSpineRollLimit();

   public abstract boolean isSpinePitchReversed();

   public abstract double getSideLengthOfBoundingBoxForFootstepHeight();

   public default double getDefaultTrajectoryTime()
   {
      return 3.0;
   }

   public default RobotCollisionModel getSelectionModel()
   {
      return null;
   }

   /**
    * Returns the parameters used to create Footstep Plans.
    */
   default public UIFootstepGeneratorParameters getUIFootstepGeneratorParameters()
   {
      return new UIFootstepGeneratorParameters();
   }
}
