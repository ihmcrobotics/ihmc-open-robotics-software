package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider.QuicksterFootstepProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public class HumanoidSteppingManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoEnum<HighLevelControllerName> latestHighLevelControllerStatus = new YoEnum<>("LatestHighLevelControllerStatePlugin", registry, HighLevelControllerName.class);

   private final ContinuousStepGenerator stepGenerator;

   private final StepGeneratorCommandInputManager commandInputManager = new StepGeneratorCommandInputManager();
   private final StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(StepGeneratorAPIDefinition.getStepGeneratorSupportedStatusMessages());

   private final List<Updatable> updatables = new ArrayList<>();

   public HumanoidSteppingManager(FullHumanoidRobotModel robotModel,
                                  CommonHumanoidReferenceFrames referenceFrames,
                                  DoubleProvider updateDT,
                                  WalkingControllerParameters walkingControllerParameters,
                                  StatusMessageOutputManager controllerStatusMessageOutputManager,
                                  CommandInputManager controllerCommandInputManager,
                                  SideDependentList<? extends ContactableBody> contactableFeet,
                                  DoubleProvider timeProvider)
   {
      registry.addChild(commandInputManager.getRegistry());

      // Configure the inputs to the modules from the command input manager
      DesiredVelocityProvider desiredVelocityProvider = commandInputManager.createDesiredVelocityProvider();
      DesiredTurningVelocityProvider desiredTurningVelocityProvider = commandInputManager.createDesiredTurningVelocityProvider();
      BooleanProvider walkingInputProvider = commandInputManager.createWalkInputProvider();

      // Configure the outputs from the modules;
      StopWalkingMessenger stopWalkingMessenger = createStopWalkingMessenger(controllerCommandInputManager);
      StartWalkingMessenger startWalkingMessenger = createStartWalkingMessenger(controllerCommandInputManager);
      FootstepMessenger footstepMessenger = controllerCommandInputManager::submitMessage;

      // Set up listeners for the status messages, and pass them into the step generator command input manager.
      controllerStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, commandInputManager::setHighLevelStateChangeStatusMessage);
      controllerStatusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, commandInputManager::setWalkingStatus);
      controllerStatusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, commandInputManager::consumeFootstepStatus);

      // Set up the step generator.
      stepGenerator = new ContinuousStepGenerator(statusMessageOutputManager, registry);
      stepGenerator.setQuicksterFootstepProvider(new QuicksterFootstepProvider(robotModel, referenceFrames, updateDT, registry, null, timeProvider));
      stepGenerator.setSupportFootBasedFootstepAdjustment(false);
      stepGenerator.setFootstepStatusListener(controllerStatusMessageOutputManager);
      stepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      stepGenerator.configureWith(walkingControllerParameters);

      // Set the inputs to the step generator.
      stepGenerator.setDesiredVelocityProvider(desiredVelocityProvider);
      stepGenerator.setDesiredTurningVelocityProvider(desiredTurningVelocityProvider);
      stepGenerator.setWalkInputProvider(walkingInputProvider);
      // Set the outputs from the step generator.
      stepGenerator.setStopWalkingMessenger(stopWalkingMessenger);
      stepGenerator.setStartWalkingMessenger(startWalkingMessenger);
      stepGenerator.setFootstepMessenger(footstepMessenger);

      if (contactableFeet != null)
         stepGenerator.setupVisualization(contactableFeet);

      // FIXME move towards some kind of consumer. this is probably not the way the class was intended to be modified.
      commandInputManager.setCSG(stepGenerator);
      stepGenerator.setYoComponentProviders();

      controllerStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, this::consumeHighLevelStateChangeStatus);
   }

   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      return commandInputManager;
   }

   public void createStepGeneratorNetworkSubscriber(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> baseTopic = ControllerAPI.getBaseTopic(HumanoidControllerAPI.HUMANOID_CONTROL_MODULE_NAME, robotName);
      StepGeneratorNetworkSubscriber stepGeneratorNetworkSubscriber = new StepGeneratorNetworkSubscriber(baseTopic,
                                                                                                         commandInputManager.getCommandInputManager(),
                                                                                                         statusMessageOutputManager,
                                                                                                         realtimeROS2Node);

      stepGeneratorNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public void update(double time)
   {
      commandInputManager.update(time);

      for (int i = 0; i < updatables.size(); i++)
         updatables.get(i).update(time);

      if (latestHighLevelControllerStatus.getValue() == HighLevelControllerName.WALKING)
         stepGenerator.update(time);
   }

   public void setFootstepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      stepGenerator.setFootstepAdjustment(footstepAdjustment);
   }

   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      stepGenerator.addFootstepValidityIndicator(footstepValidityIndicator);
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return stepGenerator.getSCS2YoGraphics();
   }

   private void consumeHighLevelStateChangeStatus(HighLevelStateChangeStatusMessage statusMessage)
   {
      latestHighLevelControllerStatus.set(HighLevelControllerName.fromByte(statusMessage.getEndHighLevelControllerName()));
   }

   private static StopWalkingMessenger createStopWalkingMessenger(CommandInputManager walkingCommandInputManager)
   {
      return new StopWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(true);

         @Override
         public void submitStopWalkingRequest()
         {
            message.setClearRemainingFootstepQueue(true);
            walkingCommandInputManager.submitMessage(message);
         }
      };
   }

   private static StartWalkingMessenger createStartWalkingMessenger(CommandInputManager controllerCommandInputManager)
   {
      return new StartWalkingMessenger()
      {
         private final PauseWalkingMessage message = HumanoidMessageTools.createPauseWalkingMessage(false);

         @Override
         public void submitStartWalkingRequest()
         {
            controllerCommandInputManager.submitMessage(message);
         }
      };
   }
}
