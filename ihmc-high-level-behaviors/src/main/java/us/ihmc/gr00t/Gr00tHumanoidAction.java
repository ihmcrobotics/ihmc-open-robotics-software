package us.ihmc.gr00t;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/** Immutable semantic action used by the reusable humanoid GR00T executor. */
public final class Gr00tHumanoidAction
{
   private final Pose3D leftWristPose;
   private final Pose3D rightWristPose;
   private final double neckPitch;
   private final double neckYaw;
   private final double[] leftHandTargets;
   private final double[] rightHandTargets;

   public Gr00tHumanoidAction(Pose3DReadOnly leftWristPose,
                              Pose3DReadOnly rightWristPose,
                              double neckPitch,
                              double neckYaw,
                              double[] leftHandTargets,
                              double[] rightHandTargets)
   {
      this.leftWristPose = new Pose3D(leftWristPose);
      this.rightWristPose = new Pose3D(rightWristPose);
      this.neckPitch = neckPitch;
      this.neckYaw = neckYaw;
      this.leftHandTargets = leftHandTargets.clone();
      this.rightHandTargets = rightHandTargets.clone();
   }

   public Pose3D getWristPose(RobotSide side)
   {
      return new Pose3D(side == RobotSide.LEFT ? leftWristPose : rightWristPose);
   }

   Pose3DReadOnly getWristPoseReadOnly(RobotSide side)
   {
      return side == RobotSide.LEFT ? leftWristPose : rightWristPose;
   }

   public double getNeckPitch()
   {
      return neckPitch;
   }

   public double getNeckYaw()
   {
      return neckYaw;
   }

   public double[] getHandTargets(RobotSide side)
   {
      return (side == RobotSide.LEFT ? leftHandTargets : rightHandTargets).clone();
   }
}
