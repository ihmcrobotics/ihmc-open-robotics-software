package us.ihmc.quadrupedCommunication.networkProcessing.heightTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;
import static us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor.bodyHeightPort;

public class QuadrupedBodyHeightTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 100;

   private final QuadrupedBodyHeightTeleopController heightTeleopController;

   public QuadrupedBodyHeightTeleopModule(FullQuadrupedRobotModelFactory modelFactory, double nominalHeight, LogModelProvider modelProvider,
                                          boolean startYoVariableServer, boolean logYoVariables, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, startYoVariableServer,
            new DataServerSettings(logYoVariables, true, bodyHeightPort, "BodyHeightTeleopModule"), updatePeriodMilliseconds, pubSubImplementation);

      heightTeleopController = new QuadrupedBodyHeightTeleopController(nominalHeight, outputManager, robotDataReceiver, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      startYoVariableServer(getClass());
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> processHighLevelStateChangeMessage(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> processSteppingStateChangeMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedTeleopDesiredHeight.class, getSubscriberTopicNameGenerator(),
                                           s -> setDesiredBodyHeight(s.takeNextData().getDesiredHeight()));
   }

   private void setPaused(boolean paused)
   {
      if (heightTeleopController != null)
         heightTeleopController.setPaused(paused);
   }

   private void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      if (heightTeleopController != null)
         heightTeleopController.processHighLevelStateChangeMessage(message);
   }

   private void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      if (heightTeleopController != null)
         heightTeleopController.processSteppingStateChangeMessage(message);
   }

   private void setDesiredBodyHeight(double desiredBodyHeight)
   {
      if (heightTeleopController != null)
         heightTeleopController.setDesiredBodyHeight(desiredBodyHeight);
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return heightTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public Map<Class<? extends Settable<?>>, ROS2Tools.MessageTopicNameGenerator> createMapOfSupportedOutputMessages()
   {
      Map<Class<? extends Settable<?>>, ROS2Tools.MessageTopicNameGenerator> messages = new HashMap<>();

      ROS2Tools.MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      messages.put(QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      return messages;
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public ROS2Tools.MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getTopicNameGenerator(robotName, ROS2Tools.HEIGHT_TELEOP_TOOLBOX, ROS2Tools.ROS2TopicQualifier.INPUT);
   }

   @Override
   public void sleep()
   {
      heightTeleopController.setPaused(true);

      super.sleep();
   }
}
