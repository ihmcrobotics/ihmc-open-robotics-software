package us.ihmc.communication;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.MessageCollection;
import ihmc_common_msgs.msg.dds.MessageCollectionNotification;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import toolbox_msgs.msg.dds.*;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

import java.util.*;

public final class HumanoidControllerAPI
{
   public static final String HUMANOID_CONTROLLER_NODE_NAME = "ihmc_controller";
   public static final String HUMANOID_KINEMATICS_CONTROLLER_NODE_NAME = "kinematics_ihmc_controller";
   public static final String HUMANOID_CONTROL_MODULE_NAME = "humanoid_control";

   private static final ROS2Topic<?> HUMANOID_CONTROLLER = ROS2Tools.IHMC_ROOT.withModule(HUMANOID_CONTROL_MODULE_NAME);
   public static final ROS2Topic<TextToSpeechPacket> TEXT_STATUS = ROS2Tools.IHMC_ROOT.withTypeName(TextToSpeechPacket.class);


   /**
    * Gets the base topic name for status messages that are coming from the robot.
    * @param robotName
    * @return
    */
   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withOutput();
   }

   /**
    * Gets the base topic name for commands that are going to the robot.
    * @param robotName
    * @return
    */
   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return HUMANOID_CONTROLLER.withRobot(robotName).withInput();
   }
}
