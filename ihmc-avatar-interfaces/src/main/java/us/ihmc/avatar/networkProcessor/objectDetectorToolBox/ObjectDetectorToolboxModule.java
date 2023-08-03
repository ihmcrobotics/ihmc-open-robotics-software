package us.ihmc.avatar.networkProcessor.objectDetectorToolBox;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import perception_msgs.msg.dds.DetectedFiducialPacket;
import perception_msgs.msg.dds.DoorLocationPacket;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

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
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
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
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return PerceptionAPI.OBJECT_DETECTOR_TOOLBOX_OUTPUT.withRobot(robotName);
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return PerceptionAPI.OBJECT_DETECTOR_TOOLBOX_INPUT.withRobot(robotName);
   }
}
