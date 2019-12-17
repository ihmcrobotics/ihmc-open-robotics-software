package us.ihmc.wholeBodyController;

import com.jme3.math.Transform;

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

   public Transform getJmeTransformWristToHand(RobotSide side);
}
