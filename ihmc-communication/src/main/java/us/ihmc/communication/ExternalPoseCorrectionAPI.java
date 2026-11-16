package us.ihmc.communication;

import ihmc_common_msgs.ExternalPoseSourceStatus;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.jros2.ROS2Topic;

/**
 * Topics around an external pose source, such as visual-inertial odometry, that corrects the state estimator's pelvis pose.
 * <p>
 * The pelvis poses themselves go to the state estimator on the standard {@code StateEstimatorAPI} topic for
 * {@link StampedPosePacket}. These topics carry what surrounds them: resetting the source, its status, the sensor pose it
 * measured, and the corrected pelvis pose. Every topic is scoped by robot name.
 */
public final class ExternalPoseCorrectionAPI
{
   public static final String MODULE_NAME = "external_pose_correction";

   private static final HumanoidROS2Topic<?> BASE_TOPIC = HumanoidROS2Topic.IHMC_ROOT.withModule(MODULE_NAME);

   public static HumanoidROS2Topic<?> getInputTopic(String robotName)
   {
      return BASE_TOPIC.withRobot(robotName).withInput();
   }

   public static HumanoidROS2Topic<?> getOutputTopic(String robotName)
   {
      return BASE_TOPIC.withRobot(robotName).withOutput();
   }

   /** Asks the pose source to reset its world frame and start a new session. */
   public static ROS2Topic<std_msgs.Empty> getResetSourceTopic(String robotName)
   {
      return getInputTopic(robotName).withType(std_msgs.Empty.class).withSuffix("reset_source");
   }

   /** The pose source's session, state and acceptance counts. */
   public static ROS2Topic<ExternalPoseSourceStatus> getSourceStatusTopic(String robotName)
   {
      return getOutputTopic(robotName).withTypeName(ExternalPoseSourceStatus.class);
   }

   /**
    * The pose of the sensor the source measures with, for example a camera, as the source measured it, in the robot's world.
    * The timestamp is the measurement time and the frame ID is the source's session.
    */
   public static ROS2Topic<StampedPosePacket> getSensorPoseTopic(String robotName)
   {
      return getOutputTopic(robotName).withType(StampedPosePacket.class).withSuffix("sensor_pose");
   }

   /** The pelvis pose after a pelvis pose correction, published whether or not the correction is applied to the estimator. */
   public static ROS2Topic<StampedPosePacket> getCorrectedPelvisPoseTopic(String robotName)
   {
      return getOutputTopic(robotName).withType(StampedPosePacket.class).withSuffix("corrected_pelvis_pose");
   }
}
