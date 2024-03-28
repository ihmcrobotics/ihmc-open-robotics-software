package us.ihmc.communication;

import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.ros2.ROS2Topic;

public class HumanoidControllerAPI
{
   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";

   public static final ROS2Topic<?> HUMANOID_CONTROLLER = ROS2Tools.IHMC_ROOT.withModule(HUMANOID_CONTROL_MODULE_NAME);

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withInput();
   }

   public static ROS2Topic<StampedPosePacket> getPoseCorrectionTopic(String robotName)
   {
      return getInputTopic(robotName).withTypeName(StampedPosePacket.class);
   }
}
