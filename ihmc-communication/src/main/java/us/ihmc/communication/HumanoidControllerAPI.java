package us.ihmc.communication;

import ihmc_common_msgs.TextToSpeechPacket;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.jros2.ROS2Topic;

public final class HumanoidControllerAPI
{
   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";

   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = ROS2Tools.IHMC_ROOT.withType(TextToSpeechPacket.class);

   public static ROS2Topic<?> getBaseTopic(String robotName)
   {
      return ControllerAPI.getBaseTopic(HUMANOID_CONTROL_MODULE_NAME, robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return getBaseTopic(robotName).appendedWith("output");
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return getBaseTopic(robotName).withInput();
   }

   /** Applies only for the humanoid controller. */
   public static <T> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      return ControllerAPI.getTopic(getBaseTopic(robotName), messageClass);
   }

   public static <T> ROS2Topic<T> getLowFrequencyTopic(Class<T> messageClass, String robotName)
   {
      return ControllerAPI.getLowFrequencyTopic(getBaseTopic(robotName), messageClass);
   }
}
