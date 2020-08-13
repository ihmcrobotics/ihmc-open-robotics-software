package us.ihmc.avatar.networkProcessor.objectDetectorToolBox;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.DetectedFiducialPacket;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeRos2Node;

public class ObjectDetectorToolboxModule extends ToolboxModule
{
   private final ObjectDetectorToolboxController controller;

   public ObjectDetectorToolboxModule(String robotName, FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider,
                                      PubSubImplementation pubSubImplementation)
   {
      super(robotName, desiredFullRobotModel, modelProvider, false, 250, pubSubImplementation);
      controller = new ObjectDetectorToolboxController(fullRobotModel, statusOutputManager, registry);
      setTimeWithoutInputsBeforeGoingToSleep(1.2e+6);
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   //TODO check this
   @Override
   public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node)
   {

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeRos2Node,
                                                    DetectedFiducialPacket.class,
                                                    FiducialDetectorToolboxModule.getOutputTopic(robotName),
                                                    s ->
                                                    {
                                                       if (controller != null)
                                                       {

                                                          controller.receivedPacket(s.takeNextData());
                                                          receivedInput.set(true);

                                                       }
                                                    });
   }

   //TODO check this   
   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      return commands;
   }

   //TODO check this
   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return Collections.singletonList(DoorLocationPacket.class);
   }

   @Override
   public ROS2Topic<DoorLocationPacket> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<DoorLocationPacket> getOutputTopic(String robotName)
   {
      return ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput().withType(DoorLocationPacket.class);
   }

   @Override
   public ROS2Topic<ToolboxStateMessage> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<ToolboxStateMessage> getInputTopic(String robotName)
   {
      return ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withInput().withType(ToolboxStateMessage.class);
   }
}
