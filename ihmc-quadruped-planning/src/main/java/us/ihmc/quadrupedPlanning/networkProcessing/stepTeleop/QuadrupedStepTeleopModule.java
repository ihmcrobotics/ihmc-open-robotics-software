package us.ihmc.quadrupedPlanning.networkProcessing.stepTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedStepTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 50;

   private final QuadrupedStepTeleopController stepTeleopController;

   public QuadrupedStepTeleopModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                    PointFootSnapperParameters pointFootSnapperParameters, LogModelProvider modelProvider, boolean startYoVariableServer,
                                    DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer, updatePeriodMilliseconds,
            pubSubImplementation);


      stepTeleopController = new QuadrupedStepTeleopController(defaultXGaitSettings, pointFootSnapperParameters, outputManager, robotDataReceiver, registry,
                                                               yoGraphicsListRegistry, updatePeriodMilliseconds);
      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
                                           s -> stepTeleopController.processTimestamp(s.takeNextData().getTimestamp()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> stepTeleopController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processFootstepStatusMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processSteppingStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedGroundPlaneMessage.class, controllerPubGenerator,
                                           s -> stepTeleopController.processGroundPlaneMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedBodyPathPlanMessage.class, getSubscriberTopicNameGenerator(),
                                           s -> stepTeleopController.processBodyPathPlanMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> stepTeleopController.processXGaitSettingsPacket(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedTeleopDesiredVelocity.class, getSubscriberTopicNameGenerator(),
                                           s -> stepTeleopController.processTeleopDesiredVelocity(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, PlanarRegionsListMessage.class, getSubscriberTopicNameGenerator(),
                                           s -> stepTeleopController.processPlanarRegionsListMessage(s.takeNextData()));
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return stepTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public Map<Class<? extends Settable<?>>, MessageTopicNameGenerator> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, MessageTopicNameGenerator> messages = new HashMap<>();

      ROS2Tools.MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      messages.put(QuadrupedTimedStepListMessage.class, controllerSubGenerator);
      messages.put(QuadrupedBodyOrientationMessage.class, controllerSubGenerator);

      return messages;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   @Override
   public void sleep()
   {
      stepTeleopController.setPaused(true);

      super.sleep();
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      stepTeleopController.setShiftPlanBasedOnStepAdjustment(shift);
   }


}
