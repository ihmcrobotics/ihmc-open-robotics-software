package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.*;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider.QuicksterFootstepProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class HumanoidSteppingPluginFactory
{
   private final StepGeneratorCommandInputManager commandInputManager = new StepGeneratorCommandInputManager();
   private final StatusMessageOutputManager statusMessageOutputManager = new StatusMessageOutputManager(StepGeneratorAPIDefinition.getStepGeneratorSupportedStatusMessages());
   private final List<Updatable> updatables = new ArrayList<>();
   private FootstepAdjustment footstepAdjustment;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();

   public HumanoidSteppingPluginFactory()
   {
   }

   public void setFootStepAdjustment(FootstepAdjustment footstepAdjustment)
   {
      this.footstepAdjustment = footstepAdjustment;
   }

   public void addFootstepValidityIndicator(FootstepValidityIndicator footstepValidityIndicator)
   {
      this.footstepValidityIndicators.add(footstepValidityIndicator);
   }

   public void addUpdatable(Updatable updatable)
   {
      this.updatables.add(updatable);
   }

   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStepGeneratorStatusMessageOutputManager()
   {
      return statusMessageOutputManager;
   }

   public void createStepGeneratorNetworkSubscriber(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> baseTopic = ControllerAPI.getBaseTopic(HumanoidControllerAPI.HUMANOID_CONTROL_MODULE_NAME, robotName);
      StepGeneratorNetworkSubscriber stepGeneratorNetworkSubscriber = new StepGeneratorNetworkSubscriber(baseTopic,
                                                                                                         getStepGeneratorCommandInputManager().getCommandInputManager(),
                                                                                                         getStepGeneratorStatusMessageOutputManager(),
                                                                                                         realtimeROS2Node);

      stepGeneratorNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
   }

   public HumanoidSteppingPlugin buildPlugin(FullHumanoidRobotModel robotModel,
                                             CommonHumanoidReferenceFrames referenceFrames,
                                             DoubleProvider updateDT,
                                             WalkingControllerParameters walkingControllerParameters,
                                             StatusMessageOutputManager controllerStatusMessageOutputManager,
                                             CommandInputManager controllerCommandInputManager,
                                             SideDependentList<? extends ContactableBody> contactableFeet,
                                             DoubleProvider timeProvider)
   {
      YoRegistry registry = new YoRegistry(HumanoidSteppingPlugin.class.getSimpleName());

      // Configure the inputs to the modules from the command input manager
      DesiredVelocityProvider desiredVelocityProvider = commandInputManager.createDesiredVelocityProvider();
      DesiredTurningVelocityProvider desiredTurningVelocityProvider = commandInputManager.createDesiredTurningVelocityProvider();
      BooleanProvider walkingInputProvider = commandInputManager.createWalkInputProvider();

      // Configure the outputs from the modules;
      StopWalkingMessenger stopWalkingMessenger = createStopWalkingMessenger(controllerCommandInputManager);
      StartWalkingMessenger startWalkingMessenger = createStartWalkingMessenger(controllerCommandInputManager);
      FootstepMessenger footstepMessenger = controllerCommandInputManager::submitMessage;
      DirectionalControlMessenger directionalControlMessenger = createDirectionalControlMessenger(controllerCommandInputManager);


      // Set up listeners for the status messages, and pass them into the step generator command input manager.
      controllerStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                       commandInputManager::setHighLevelStateChangeStatusMessage);
      controllerStatusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, commandInputManager::setWalkingStatus);
      controllerStatusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, commandInputManager::consumeFootstepStatus);

      // Set up the step generator.
      ContinuousStepGenerator continuousStepGenerator = new ContinuousStepGenerator(statusMessageOutputManager, registry);
      continuousStepGenerator.setQuicksterFootstepProvider(new QuicksterFootstepProvider(robotModel,
                                                                                         referenceFrames,
                                                                                         updateDT,
                                                                                         registry,
                                                                                         null,
                                                                                         timeProvider));
      continuousStepGenerator.setSupportFootBasedFootstepAdjustment(false);
      if (footstepAdjustment != null)
         continuousStepGenerator.setFootstepAdjustment(footstepAdjustment);
      for (FootstepValidityIndicator footstepValidityIndicator : footstepValidityIndicators)
         continuousStepGenerator.addFootstepValidityIndicator(footstepValidityIndicator);
      continuousStepGenerator.setFootstepStatusListener(controllerStatusMessageOutputManager);
      continuousStepGenerator.setFrameBasedFootPoseProvider(referenceFrames.getSoleZUpFrames());
      continuousStepGenerator.configureWith(walkingControllerParameters);

      // Set the inputs to the step generator.
      continuousStepGenerator.setDesiredVelocityProvider(desiredVelocityProvider);
      continuousStepGenerator.setDesiredTurningVelocityProvider(desiredTurningVelocityProvider);
      continuousStepGenerator.setWalkInputProvider(walkingInputProvider);
      // Set the outputs from the step generator.
      continuousStepGenerator.setStopWalkingMessenger(stopWalkingMessenger);
      continuousStepGenerator.setStartWalkingMessenger(startWalkingMessenger);
      continuousStepGenerator.setFootstepMessenger(footstepMessenger);

      if (contactableFeet != null)
         continuousStepGenerator.setupVisualization(contactableFeet);


      // FIXME move towards some kind of consumer. this is probably not the way the class was intended to be modified.
      commandInputManager.setCSG(continuousStepGenerator);
      continuousStepGenerator.setYoComponentProviders();

      VelocityBasedSteppingPlugin velocityBasedSteppingPlugin = new VelocityBasedSteppingPlugin();

      // Set the inputs to the velocity plugin.
      velocityBasedSteppingPlugin.setDesiredVelocityProvider(desiredVelocityProvider);
      velocityBasedSteppingPlugin.setDesiredTurningVelocityProvider(desiredTurningVelocityProvider);
      velocityBasedSteppingPlugin.setWalkInputProvider(walkingInputProvider);
      velocityBasedSteppingPlugin.setSwingHeightInputProvider(commandInputManager.createSwingHeightProvider());

      // Set the outputs from the velocity plugin.
      velocityBasedSteppingPlugin.setDirectionalControlMessenger(directionalControlMessenger);
      velocityBasedSteppingPlugin.setStopWalkingMessenger(stopWalkingMessenger);
      velocityBasedSteppingPlugin.setStartWalkingMessenger(startWalkingMessenger);

      updatables.add(commandInputManager);

      HumanoidSteppingPlugin joystickBasedSteppingPlugin = new HumanoidSteppingPlugin(continuousStepGenerator, velocityBasedSteppingPlugin, updatables);
      joystickBasedSteppingPlugin.setHighLevelStateChangeStatusListener(controllerStatusMessageOutputManager);

      return joystickBasedSteppingPlugin;
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

   private static DirectionalControlMessenger createDirectionalControlMessenger(CommandInputManager controllerCommandInputManager)
   {
      return new DirectionalControlMessenger()
      {
         private final DirectionalControlInputMessage message = new DirectionalControlInputMessage();
         private final FastWalkingGaitParametersMessage gaitParameters = new FastWalkingGaitParametersMessage();

         @Override
         public void submitDirectionalControlRequest(double desiredXVelocity, double desiredYVelocity, double desiredTurningSpeed)
         {
            message.setForward(desiredXVelocity);
            message.setRight(-desiredYVelocity);
            message.setClockwise(-desiredTurningSpeed);

            controllerCommandInputManager.submitMessage(message);
         }

         @Override
         public void submitGaitParameters(double swingHeight, double swingDuration, double doubleSupportFraction)
         {
            gaitParameters.setSwingHeight(swingHeight);
            gaitParameters.setSwingDuration(swingDuration);
            gaitParameters.setDoubleSupportFraction(doubleSupportFraction);

            controllerCommandInputManager.submitMessage(gaitParameters);
         }
      };
   }
}
