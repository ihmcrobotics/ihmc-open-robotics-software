package us.ihmc.quadrupedPlanning.networkProcessing.xBox;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedXBoxModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 10;

   private final QuadrupedXBoxController xBoxController;

   public QuadrupedXBoxModule(FullQuadrupedRobotModelFactory modelFactory, QuadrupedXGaitSettings defaultXGaitSettings, double nominalBodyHeight,
                              LogModelProvider modelProvider, DomainFactory.PubSubImplementation pubSubImplementation)
         throws IOException
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, false, updatePeriodMilliseconds,
            pubSubImplementation);

      xBoxController = new QuadrupedXBoxController(defaultXGaitSettings, nominalBodyHeight, outputManager, registry, updatePeriodMilliseconds);
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools
            .createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> xBoxController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> xBoxController.processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> xBoxController.processSteppingStateChangeMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedXGaitSettingsPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> xBoxController.processXGaitSettingsPacket(s.takeNextData()));
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return xBoxController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return null;
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Tools.MessageTopicNameGenerator> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Tools.MessageTopicNameGenerator> messages = new HashMap<>();

      messages.put(QuadrupedTeleopDesiredHeight.class, getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));
      messages.put(QuadrupedTeleopDesiredPose.class, getTopicNameGenerator(robotName, ROS2Tools.BODY_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));
      messages.put(QuadrupedTeleopDesiredVelocity.class, getTopicNameGenerator(robotName, ROS2Tools.STEP_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT));

      return messages;
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.XBOX_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.XBOX_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   @Override
   public void sleep()
   {
      super.sleep();
   }
}
