package us.ihmc.communication;

import ihmc_common_msgs.TextToSpeechPacket;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;

public final class HumanoidControllerAPI
{
   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";

   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = HumanoidROS2Topic.IHMC_ROOT.withTypeName(TextToSpeechPacket.class)
                                                                                               .withQoS(ROS2QoSProfile.RELIABLE);

   public static ROS2Topic<?> getBaseTopic(String robotName)
   {
      return ControllerAPI.getBaseTopic(HUMANOID_CONTROL_MODULE_NAME, robotName);
   }

   public static HumanoidROS2Topic<?> getOutputTopic(String robotName)
   {
      return toHumanoid(getBaseTopic(robotName)).withOutput();
   }

   public static HumanoidROS2Topic<?> getInputTopic(String robotName)
   {
      return toHumanoid(getBaseTopic(robotName)).withInput();
   }

   private static HumanoidROS2Topic<?> toHumanoid(ROS2Topic<?> topic)
   {
      return (HumanoidROS2Topic<?>) topic;
   }

   /** Applies only for the humanoid controller. */
   public static <T extends ROS2Message<T>> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      return ControllerAPI.getTopic(getBaseTopic(robotName), messageClass);
   }

   public static <T extends ROS2Message<T>> ROS2Topic<T> getLowFrequencyTopic(Class<T> messageClass, String robotName)
   {
      return ControllerAPI.getLowFrequencyTopic(getBaseTopic(robotName), messageClass);
   }
}
