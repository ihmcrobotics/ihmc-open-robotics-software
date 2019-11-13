package us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.RealtimeRos2Node;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.communication.ROS2Tools.getTopicNameGenerator;

public class FootstepPlanPostProcessingToolboxModule extends ToolboxModule
{
   public static final String moduleName = ROS2Tools.FOOTSTEP_POSTPROCESSING_TOOLBOX;

   private final FootstepPlanPostProcessingToolboxController toolboxController;

   public FootstepPlanPostProcessingToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer)
   {
      this(drcRobotModel, modelProvider, startYoVariableServer, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   public FootstepPlanPostProcessingToolboxModule(DRCRobotModel drcRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
                                                  DomainFactory.PubSubImplementation pubSubImplementation)
   {
      super(drcRobotModel.getSimpleRobotName(), drcRobotModel.createFullRobotModel(), modelProvider, startYoVariableServer, pubSubImplementation);
      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      toolboxController = new FootstepPlanPostProcessingToolboxController(new DefaultFootstepPostProcessingParameters(),
                                                                          drcRobotModel.getWalkingControllerParameters(),
                                                                          drcRobotModel.getContactPointParameters(),
                                                                          drcRobotModel.getCapturePointPlannerParameters(),
                                                                          statusOutputManager, registry,
                                                                          yoGraphicsListRegistry);
      startYoVariableServer();
   }

   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPostProcessingPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> {
                                              toolboxController.processPostProcessingPacket(s.takeNextData());
                                              wakeUp();
                                           });
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, FootstepPostProcessingParametersPacket.class, getSubscriberTopicNameGenerator(),
                                           s -> toolboxController.processFootstepPostProcessingParameters(s.takeNextData()));
   }

   @Override
   public void sleep()
   {
      toolboxController.finishUp();
      super.sleep();
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
      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();
      statusMessages.add(FootstepPostProcessingPacket.class);

      return statusMessages;
   }

   @Override
   public MessageTopicNameGenerator getPublisherTopicNameGenerator()
   {
      return getPublisherTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, moduleName, ROS2TopicQualifier.OUTPUT);
   }

   @Override
   public MessageTopicNameGenerator getSubscriberTopicNameGenerator()
   {
      return getSubscriberTopicNameGenerator(robotName);
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return getTopicNameGenerator(robotName, moduleName, ROS2TopicQualifier.INPUT);
   }
}
