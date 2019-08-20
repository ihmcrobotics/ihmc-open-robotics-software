package us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class PawPlannerCommunicationProperties
{
   private static final List<Class<? extends Command<?, ?>>> toolboxSupportedCommands;
   private static final List<Class<? extends Settable<?>>> toolboxSupportedStatusMessages;

   static
   {
      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(FootstepPlanningToolboxOutputStatus.class);
      statusMessages.add(FootstepPlannerStatusMessage.class);
      statusMessages.add(BodyPathPlanMessage.class);
      statusMessages.add(BodyPathPlanStatisticsMessage.class);
      statusMessages.add(FootstepNodeDataListMessage.class);
      statusMessages.add(FootstepPlannerOccupancyMapMessage.class);

      toolboxSupportedStatusMessages = Collections.unmodifiableList(statusMessages);

      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();

      toolboxSupportedCommands = Collections.unmodifiableList(commands);
   }

   public static List<Class<? extends Settable<?>>> getSupportedStatusMessages()
   {
      return toolboxSupportedStatusMessages;
   }

   public static List<Class<? extends Command<?, ?>>> getSupportedCommands()
   {
      return toolboxSupportedCommands;
   }

   public static MessageTopicNameGenerator publisherTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   public static MessageTopicNameGenerator subscriberTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX, ROS2TopicQualifier.INPUT);
   }
}
