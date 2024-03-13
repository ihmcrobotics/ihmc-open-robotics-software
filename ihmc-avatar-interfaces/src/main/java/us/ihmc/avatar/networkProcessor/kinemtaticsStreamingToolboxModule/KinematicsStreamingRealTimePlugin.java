package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * This class is meant to wrap an instance of the {@link KinematicsStreamingToolboxController} into a real-time plugin that can be used on the control computer.
 */
public class KinematicsStreamingRealTimePlugin
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final KinematicsStreamingToolboxController kinematicsStreamingToolboxController;

   public KinematicsStreamingRealTimePlugin(String robotName,
                                            ROS2NodeInterface ros2Node,
                                            CommandInputManager walkingInputManager,
                                            StatusMessageOutputManager walkingOutputManager,
                                            FullHumanoidRobotModelFactory fullRobotModelFactory,
                                            KinematicsStreamingToolboxParameters parameters)
   {
      FullHumanoidRobotModel desiredFullRobotModel = fullRobotModelFactory.createFullRobotModel(false);
      this.commandInputManager = new CommandInputManager(KinematicsStreamingToolboxModule.supportedCommands());
      this.statusOutputManager = new StatusMessageOutputManager(KinematicsStreamingToolboxModule.supportedStatus());
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(KinematicsStreamingToolboxModule.getInputTopic(robotName),
                                                                                                commandInputManager,
                                                                                                KinematicsStreamingToolboxModule.getOutputTopic(robotName),
                                                                                                statusOutputManager,
                                                                                                ros2Node);

      this.kinematicsStreamingToolboxController = new KinematicsStreamingToolboxController(commandInputManager,
                                                                                           statusOutputManager,
                                                                                           parameters,
                                                                                           desiredFullRobotModel,
                                                                                           fullRobotModelFactory,
                                                                                           yoGraphicsListRegistry,
                                                                                           registry);
      kinematicsStreamingToolboxController.setStreamingMessagePublisher(walkingInputManager::submitMessage);
      kinematicsStreamingToolboxController.setTrajectoryMessagePublisher(walkingInputManager::submitMessage);
      walkingOutputManager.attachStatusMessageListener(RobotConfigurationData.class, kinematicsStreamingToolboxController::updateRobotConfigurationData);
      walkingOutputManager.attachStatusMessageListener(CapturabilityBasedStatus.class, kinematicsStreamingToolboxController::updateCapturabilityBasedStatus);
   }
}
