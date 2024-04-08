package us.ihmc.communication;

import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.ros2.ROS2Topic;

public final class HumanoidControllerAPI
{
   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";

   public static final ROS2Topic<?> HUMANOID_CONTROLLER = ROS2Tools.IHMC_ROOT.withModule(HUMANOID_CONTROL_MODULE_NAME);
   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = ROS2Tools.IHMC_ROOT.withTypeName(TextToSpeechPacket.class);

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withInput();
   }

   /** Applies only for the humanoid controller. */
   public static <T> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      return ControllerAPI.getTopic(ControllerAPI.getBaseTopic(HUMANOID_CONTROL_MODULE_NAME, robotName), messageClass);
   }
}
