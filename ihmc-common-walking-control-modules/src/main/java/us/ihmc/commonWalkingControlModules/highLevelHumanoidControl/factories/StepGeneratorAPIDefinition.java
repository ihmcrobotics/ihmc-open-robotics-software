package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import controller_msgs.ContinuousStepGeneratorStatusMessage;
import controller_msgs.ControllerWalkToGoalStatusMessage;
import controller_msgs.ControllerWaypointChangeStatusMessage;
import controller_msgs.ControllerWaypointListStatusMessage;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorInputCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorParametersCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerReleaseGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalListCommand;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.AbortWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;

public class StepGeneratorAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> stepGeneratorSupportedCommands;
   private static final List<Class<? extends ROS2Message<?>>> stepGeneratorSupportedStatusMessages;
   private static final HashSet<Class<?>> inputMessageClasses = new HashSet<>();
   private static final HashSet<Class<?>> outputMessageClasses = new HashSet<>();

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();

      commands.add(ContinuousStepGeneratorParametersCommand.class);
      commands.add(ContinuousStepGeneratorInputCommand.class);
      commands.add(ControllerWaypointGoalCommand.class);
      commands.add(ControllerWaypointGoalListCommand.class);
      commands.add(ControllerReleaseGoalCommand.class);
      commands.add(PlanarRegionsListCommand.class);
      commands.add(HeightMapCommand.class);
      commands.add(AbortWalkingCommand.class);
      commands.add(PauseWalkingCommand.class);

      stepGeneratorSupportedCommands = Collections.unmodifiableList(commands);
      for (Class<? extends Command<?, ?>> commandClass : stepGeneratorSupportedCommands)
      {
         try
         {
            Command<?, ?> command = commandClass.getDeclaredConstructor().newInstance();
            inputMessageClasses.add(command.getMessageClass());
         }
         catch (InstantiationException | IllegalAccessException | NoSuchMethodException | InvocationTargetException e)
         {
            throw new RuntimeException(e);
         }
      }

      List<Class<? extends ROS2Message<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(ContinuousStepGeneratorStatusMessage.class);
      statusMessages.add(ControllerWalkToGoalStatusMessage.class);
      statusMessages.add(ControllerWaypointChangeStatusMessage.class);
      statusMessages.add(ControllerWaypointListStatusMessage.class);

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
      return outputMessageClasses;
   }

   public static List<Class<? extends ROS2Message<?>>> getStepGeneratorSupportedStatusMessages()
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

   public static <T extends ROS2Message<T>> ROS2Topic<T> getTopic(Class<T> messageClass, String robotName)
   {
      if (inputMessageClasses.contains(messageClass) || outputMessageClasses.contains(messageClass))
      {
         return HumanoidControllerAPI.getTopic(messageClass, robotName);
      }

      throw new RuntimeException("Topic does not exist: " + messageClass);
   }
}
