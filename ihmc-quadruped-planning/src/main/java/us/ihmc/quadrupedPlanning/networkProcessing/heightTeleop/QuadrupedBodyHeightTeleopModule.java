package us.ihmc.quadrupedPlanning.networkProcessing.heightTeleop;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedRobotModelProviderNode;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedPlanning.networkProcessing.QuadrupedToolboxModule;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class QuadrupedBodyHeightTeleopModule extends QuadrupedToolboxModule
{
   private static final int updatePeriodMilliseconds = 1;

   private final QuadrupedBodyHeightTeleopController heightTeleopController;

   public QuadrupedBodyHeightTeleopModule(FullQuadrupedRobotModelFactory modelFactory, double nominalHeight, LogModelProvider modelProvider,
                                          DomainFactory.PubSubImplementation pubSubImplementation) throws IOException
   {
      super(modelFactory.getRobotDescription().getName(), modelFactory.createFullRobotModel(), modelProvider, false, updatePeriodMilliseconds,
            pubSubImplementation);

      QuadrupedRobotModelProviderNode robotModelProvider = new QuadrupedRobotModelProviderNode(robotName, realtimeRos2Node, modelFactory);

      heightTeleopController = new QuadrupedBodyHeightTeleopController(nominalHeight, commandInputManager, statusOutputManager, robotModelProvider, registry);
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateMessage.class, getPublisherTopicNameGenerator(),
                                           s -> heightTeleopController.setPaused(true));
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, getPublisherTopicNameGenerator(),
                                           s -> heightTeleopController.processHighLevelStateChangeMessage(s.takeNextData()));
   }

   @Override
   public QuadrupedToolboxController getToolboxController()
   {
      return heightTeleopController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return null;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(HighLevelStateMessage.class);
      statusMessages.add(HighLevelStateChangeStatusMessage.class);
      statusMessages.add(RobotConfigurationData.class);

      return statusMessages;
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

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      heightTeleopController.setDesiredBodyHeight(desiredBodyHeight);
   }
}
