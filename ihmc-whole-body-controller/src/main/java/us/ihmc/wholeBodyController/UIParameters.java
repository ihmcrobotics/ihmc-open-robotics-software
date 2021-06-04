package us.ihmc.wholeBodyController;

import com.jme3.math.Transform;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.UIFootstepGeneratorParameters;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;

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

   public abstract Transform getJmeTransformWristToHand(RobotSide side);

   public default RobotCollisionModel getSelectionModel()
   {
      return null;
   }

   public default RigidBodyTransform getHandControlFramePose(RobotSide side)
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
