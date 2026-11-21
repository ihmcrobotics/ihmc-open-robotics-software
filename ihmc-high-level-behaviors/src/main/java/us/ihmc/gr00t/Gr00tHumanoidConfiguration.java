package us.ihmc.gr00t;

import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;

/** Robot execution semantics and safety limits for a humanoid GR00T embodiment. */
public record Gr00tHumanoidConfiguration(RobotSide armSide,
                                         boolean controlLeftHand,
                                         boolean controlRightHand,
                                         NeckJointName neckYawJoint,
                                         NeckJointName neckPitchJoint,
                                         double policyActionPeriod,
                                         double initialTrajectoryTime,
                                         double trajectoryTime,
                                         double maxLinearVelocity,
                                         double maxAngularVelocity,
                                         double stageSettleTime,
                                         double endpointConvergenceGraceTime,
                                         double endpointPositionThreshold,
                                         double endpointOrientationThreshold)
{
   public Gr00tHumanoidConfiguration
   {
      if (armSide == null)
         throw new IllegalArgumentException("armSide must not be null");
      requirePositive(policyActionPeriod, "policyActionPeriod");
      requirePositive(initialTrajectoryTime, "initialTrajectoryTime");
      requirePositive(trajectoryTime, "trajectoryTime");
      requirePositive(maxLinearVelocity, "maxLinearVelocity");
      requirePositive(maxAngularVelocity, "maxAngularVelocity");
      requireNonNegative(stageSettleTime, "stageSettleTime");
      requireNonNegative(endpointConvergenceGraceTime, "endpointConvergenceGraceTime");
      requirePositive(endpointPositionThreshold, "endpointPositionThreshold");
      requirePositive(endpointOrientationThreshold, "endpointOrientationThreshold");
   }

   public boolean controlsHand(RobotSide side)
   {
      return side == RobotSide.LEFT ? controlLeftHand : controlRightHand;
   }

   public boolean controlsNeck()
   {
      return neckYawJoint != null || neckPitchJoint != null;
   }

   private static void requirePositive(double value, String name)
   {
      if (!Double.isFinite(value) || value <= 0.0)
         throw new IllegalArgumentException(name + " must be finite and positive");
   }

   private static void requireNonNegative(double value, String name)
   {
      if (!Double.isFinite(value) || value < 0.0)
         throw new IllegalArgumentException(name + " must be finite and non-negative");
   }
}
