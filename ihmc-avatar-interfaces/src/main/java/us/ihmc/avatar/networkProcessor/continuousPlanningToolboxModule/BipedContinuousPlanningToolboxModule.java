package us.ihmc.avatar.networkProcessor.continuousPlanningToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2TopicName;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.List;

public class BipedContinuousPlanningToolboxModule extends ToolboxModule
{
   private final BipedContinuousPlanningToolboxController toolboxController;

   private IHMCRealtimeROS2Publisher<ToolboxStateMessage> footstepPlanningToolboxStatePublisher;
   private IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;

   public BipedContinuousPlanningToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer)
   {
      this(drcRobotModel, modelProvider, startYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public BipedContinuousPlanningToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
                                               PubSubImplementation pubSubImplementation)
   {
      super(drcRobotModel.getSimpleRobotName(), drcRobotModel.createFullRobotModel(), modelProvider, startYoVariableServer, pubSubImplementation);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);

      toolboxController = new BipedContinuousPlanningToolboxController(statusOutputManager,
                                                                       footstepPlanningRequestPublisher,
                                                                       footstepPlanningToolboxStatePublisher,
                                                                       drcRobotModel.getFootstepPlannerParameters(),
                                                                       registry);

      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2TopicName controllerPubGenerator = ROS2Tools.getControllerOutputTopicName(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepStatusMessage.class, controllerPubGenerator,
                                           s -> processFootstepStatusMessage(s.takeNextData()));

      // status messages from the planner
      ROS2TopicName plannerPubGenerator = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                                             .withOutput();
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPlanningToolboxOutputStatus.class, plannerPubGenerator,
                                           s -> processFootstepPlannerOutput(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, BipedContinuousPlanningRequestPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> processContinuousPlanningRequest(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator,
                                           s -> processPlanarRegionsListMessage(s.takeNextData()));

      ROS2TopicName plannerSubGenerator = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                                             .withInput();
      footstepPlanningRequestPublisher = ROS2Tools.createPublisher(realtimeRos2Node, FootstepPlanningRequestPacket.class, plannerSubGenerator);
      footstepPlanningToolboxStatePublisher = ROS2Tools.createPublisher(realtimeRos2Node, ToolboxStateMessage.class, plannerSubGenerator);
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return toolboxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> messages = new ArrayList<>();

      messages.add(FootstepPlanningToolboxOutputStatus.class);
      messages.add(BodyPathPlanMessage.class);
      messages.add(FootstepDataListMessage.class);

      return messages;
   }

   @Override
   public ROS2TopicName getPublisherTopicNameGenerator()
   {
      return ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2TopicName getSubscriberTopicNameGenerator()
   {
      return ROS2Tools.CONTINUOUS_PLANNING_TOOLBOX.withRobot(robotName).withInput();
   }

   @Override
   public void sleep()
   {
      super.sleep();

      footstepPlanningToolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   private void processFootstepPlannerOutput(FootstepPlanningToolboxOutputStatus footstepPlannerOutput)
   {
      if (toolboxController != null)
         toolboxController.processFootstepPlannerOutput(footstepPlannerOutput);
   }

   private void processContinuousPlanningRequest(BipedContinuousPlanningRequestPacket planningRequestPacket)
   {
      if (toolboxController != null)
      {
         toolboxController.processContinuousPlanningRequest(planningRequestPacket);
         wakeUp();
      }
   }

   private void processFootstepStatusMessage(FootstepStatusMessage footstepStatusMessage)
   {
      if (toolboxController != null)
         toolboxController.processFootstepStatusMessage(footstepStatusMessage);
   }

   private void processPlanarRegionsListMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      if (toolboxController != null)
         toolboxController.processPlanarRegionListMessage(planarRegionsListMessage);
   }
}
