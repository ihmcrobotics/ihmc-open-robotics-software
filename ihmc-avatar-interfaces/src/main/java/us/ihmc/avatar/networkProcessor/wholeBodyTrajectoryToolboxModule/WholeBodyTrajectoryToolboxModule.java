package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.ReachingManifoldCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.RigidBodyExplorationConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WaypointBasedTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxConfigurationCommand;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.RealtimeRos2Node;

public class WholeBodyTrajectoryToolboxModule extends ToolboxModule
{
   private final WholeBodyTrajectoryToolboxController wholeBodyTrajectoryToolboxController;

   public WholeBodyTrajectoryToolboxModule(DRCRobotModel drcRobotModel, boolean startYoVariableServer) throws IOException
   {
      this(drcRobotModel, startYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public WholeBodyTrajectoryToolboxModule(DRCRobotModel drcRobotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
         throws IOException
   {
      super(drcRobotModel.getSimpleRobotName(),
            drcRobotModel.createFullRobotModel(),
            drcRobotModel.getLogModelProvider(),
            startYoVariableServer,
            pubSubImplementation);

      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);

      wholeBodyTrajectoryToolboxController = new WholeBodyTrajectoryToolboxController(drcRobotModel, fullRobotModel, commandInputManager, statusOutputManager,
                                                                                      registry, yoGraphicsListRegistry, startYoVariableServer);
      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryToolboxMessage.class, 10,
                                                                       MessageUnpackingTools.createWholeBodyTrajectoryToolboxMessageUnpacker());
      commandInputManager.registerConversionHelper(new WholeBodyTrajectoryToolboxCommandConverter(fullRobotModel));
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return wholeBodyTrajectoryToolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(WaypointBasedTrajectoryCommand.class);
      commands.add(RigidBodyExplorationConfigurationCommand.class);
      commands.add(ReachingManifoldCommand.class);
      commands.add(WholeBodyTrajectoryToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(WholeBodyTrajectoryToolboxOutputStatus.class);
      return status;
   }

   @Override
   public Set<Class<? extends Command<?, ?>>> silentCommands()
   {
      Set<Class<? extends Command<?, ?>>> commands = new HashSet<>();
      commands.add(WaypointBasedTrajectoryCommand.class);
      commands.add(RigidBodyExplorationConfigurationCommand.class);
      commands.add(ReachingManifoldCommand.class);
      commands.add(WholeBodyTrajectoryToolboxConfigurationCommand.class);
      return commands;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WHOLE_BODY_TRAJECTORY_TOOLBOX, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.WHOLE_BODY_TRAJECTORY_TOOLBOX, ROS2TopicQualifier.INPUT);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator, s -> {
         if (wholeBodyTrajectoryToolboxController != null)
            wholeBodyTrajectoryToolboxController.updateRobotConfigurationData(s.takeNextData());
      });
   }
}