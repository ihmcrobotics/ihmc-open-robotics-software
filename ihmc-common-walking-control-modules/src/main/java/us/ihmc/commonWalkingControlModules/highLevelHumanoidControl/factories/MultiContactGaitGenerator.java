package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TerrainMapCommand;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

public class MultiContactGaitGenerator
{
   private static final List<Class<? extends Command<?, ?>>> supportedCommands;
   private static final List<Class<? extends Settable<?>>> supportedStatusMessages;
   private static final HashSet<Class<?>> inputMessageClasses = new HashSet<>();
   private static final HashSet<Class<?>> outputMessageClasses = new HashSet<>();

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(PlanarRegionsListCommand.class);
      commands.add(TerrainMapCommand.class);

      supportedCommands = Collections.unmodifiableList(commands);
      supportedCommands.forEach(command -> inputMessageClasses.add(ROS2TopicNameTools.newMessageInstance(command).getMessageClass()));

      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(ContinuousStepGeneratorStatusMessage.class);

      supportedStatusMessages = Collections.unmodifiableList(statusMessages);
      outputMessageClasses.addAll(supportedStatusMessages);
   }

   public static List<Class<? extends Command<?, ?>>> getSupportedCommands()
   {
      return supportedCommands;
   }

   public static HashSet<Class<?>> getROS2CommandMessageTypes()
   {
      return inputMessageClasses;
   }

   public static HashSet<Class<?>> getROS2StatusMessageTypes()
   {
      return outputMessageClasses;
   }

   public static List<Class<? extends Settable<?>>> getSupportedStatusMessages()
   {
      return supportedStatusMessages;
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return HumanoidControllerAPI.getInputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return HumanoidControllerAPI.getOutputTopic(robotName);
   }

   public static <T> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      if (inputMessageClasses.contains(messageClass))
      {
         return getInputTopic(robotName).withTypeName(messageClass);
      }
      if (outputMessageClasses.contains(messageClass))
      {
         return getOutputTopic(robotName).withTypeName(messageClass);
      }

      throw new RuntimeException("Topic does not exist: " + messageClass);
   }
}
