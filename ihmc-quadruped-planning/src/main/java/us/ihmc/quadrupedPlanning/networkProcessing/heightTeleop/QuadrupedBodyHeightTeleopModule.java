package us.ihmc.quadrupedPlanning.networkProcessing.heightTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
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

public class QuadrupedBodyHeightTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 10;

   private final QuadrupedBodyHeightTeleopController heightTeleopController;

   public QuadrupedBodyHeightTeleopModule(FullQuadrupedRobotModelFactory modelFactory, double nominalHeight, LogModelProvider modelProvider,
                                          DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, false, updatePeriodMilliseconds,
            pubSubImplementation);

      heightTeleopController = new QuadrupedBodyHeightTeleopController(nominalHeight, outputManager, robotDataReceiver, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   @Override
   public void registerExtraSubscribers(RealtimeRos2Node realtimeRos2Node)
   {
      // status messages from the controller
      ROS2Tools.MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> heightTeleopController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> heightTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));

      // inputs to this module
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, QuadrupedTeleopDesiredHeight.class, getSubscriberTopicNameGenerator(),
                                           s -> heightTeleopController.setDesiredBodyHeight(s.takeNextData().getDesiredHeight()));
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
