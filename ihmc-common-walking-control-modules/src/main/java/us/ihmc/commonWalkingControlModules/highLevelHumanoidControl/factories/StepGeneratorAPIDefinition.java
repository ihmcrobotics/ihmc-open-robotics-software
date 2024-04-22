package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorInputCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorParametersCommand;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

public class StepGeneratorAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> stepGeneratorSupportedCommands;
   private static final List<Class<? extends Settable<?>>> stepGeneratorSupportedStatusMessages;
   private static final HashSet<Class<?>> inputMessageClasses = new HashSet<>();
   private static final HashSet<Class<?>> outputMessageClasses = new HashSet<>();

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();

      commands.add(ContinuousStepGeneratorParametersCommand.class);
      commands.add(ContinuousStepGeneratorInputCommand.class);
      commands.add(PlanarRegionsListCommand.class);

      stepGeneratorSupportedCommands = Collections.unmodifiableList(commands);
      stepGeneratorSupportedCommands.forEach(command -> inputMessageClasses.add(ROS2TopicNameTools.newMessageInstance(command).getMessageClass()));

      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();


      stepGeneratorSupportedStatusMessages = Collections.unmodifiableList(statusMessages);
      outputMessageClasses.addAll(stepGeneratorSupportedStatusMessages);
   }

   public static List<Class<? extends Command<?, ?>>> getStepGeneratorSupportedCommands()
   {
      return stepGeneratorSupportedCommands;
   }

   public static HashSet<Class<?>> getROS2CommandMessageTypes()
   {
      return inputMessageClasses;
   }

   public static HashSet<Class<?>> getROS2StatusMessageTypes()
   {
      return inputMessageClasses;
   }

   public static List<Class<? extends Settable<?>>> getStepGeneratorSupportedStatusMessages()
   {
      return stepGeneratorSupportedStatusMessages;
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
