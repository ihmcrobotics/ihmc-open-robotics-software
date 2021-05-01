package us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.DetectedFiducialPacket;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public class FiducialDetectorToolboxModule extends ToolboxModule
{
   private final FiducialDetectorToolboxController controller;

   public FiducialDetectorToolboxModule(String robotName,
                                        RobotTarget target,
                                        FullHumanoidRobotModel desiredFullRobotModel,
                                        LogModelProvider modelProvider,
                                        PubSubImplementation pubSubImplementation)
   {
      super(robotName, desiredFullRobotModel, modelProvider, false, 250, pubSubImplementation);
      controller = new FiducialDetectorToolboxController(fullRobotModel, target, statusOutputManager, registry);
      setTimeWithoutInputsBeforeGoingToSleep(1.2e+6);
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public void registerExtraPuSubs(RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT, s ->
      {
         if (controller != null)
         {
            controller.receivedPacket(s.takeNextData());
            receivedInput.set(true);
         }
      });
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();

      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return Collections.singletonList(DetectedFiducialPacket.class);
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ROS2Tools.FIDUCIAL_DETECTOR_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ROS2Tools.FIDUCIAL_DETECTOR_TOOLBOX.withRobot(robotName).withInput();
   }
}
