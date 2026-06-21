package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.VelocityBasedWalkingInputMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.PDVelocityBasedGoalReacher;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.StartWalkingMessenger;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.StopWalkingMessenger;
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
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public class HumanoidSteppingManager implements Updatable, SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoEnum<HighLevelControllerName> latestHighLevelControllerStatus = new YoEnum<>("LatestHighLevelControllerState",
                                                                                                registry,
                                                                                                HighLevelControllerName.class);

   private final ContinuousStepGenerator stepGenerator;
   private final PDVelocityBasedGoalReacher goalReacher;

   private final CommandInputManager controllerCommandInputManager;

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
      this.controllerCommandInputManager = controllerCommandInputManager;
      registry.addChild(commandInputManager.getRegistry());

      // Set up listeners for the status messages, and pass them into the step generator command input manager.
      controllerStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                       commandInputManager::setHighLevelStateChangeStatusMessage);
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
      stepGenerator.setDesiredVelocityProvider(commandInputManager.createDesiredVelocityProvider());
      stepGenerator.setDesiredTurningVelocityProvider(commandInputManager.createDesiredTurningVelocityProvider());
      stepGenerator.setWalkInputProvider(commandInputManager.createWalkInputProvider());
      // Set the outputs from the step generator.
      stepGenerator.setStopWalkingMessenger(createStopWalkingMessenger(controllerCommandInputManager));
      stepGenerator.setStartWalkingMessenger(createStartWalkingMessenger(controllerCommandInputManager));
      stepGenerator.setFootstepMessenger(controllerCommandInputManager::submitMessage);

      if (contactableFeet != null)
         stepGenerator.setupVisualization(contactableFeet);

      commandInputManager.addContinuousStepGeneratorParametersCommandConsumer(command ->
                                                                              {
                                                                                 ContinuousStepGeneratorParameters parameters = command.getParameters();
                                                                                 stepGenerator.setFootstepTiming(parameters.getSwingDuration(),
                                                                                                                 parameters.getTransferDuration());
                                                                                 stepGenerator.setSwingHeight(parameters.getSwingHeight());
                                                                                 stepGenerator.setFootstepsAreAdjustable(parameters.getStepsAreAdjustable());
                                                                                 stepGenerator.setStepWidths(parameters.getDefaultStepWidth(),
                                                                                                             parameters.getMinStepWidth(),
                                                                                                             parameters.getMaxStepWidth());
                                                                                 stepGenerator.seIsSnappingToHeightmap(parameters.getRequestSnapToHeightmap());
                                                                                 stepGenerator.setMaxStepLengthForwards(parameters.getMaxStepLengthForwards());
                                                                                 stepGenerator.setMaxStepLengthBackwards(parameters.getMaxStepLengthBackwards());
                                                                                 stepGenerator.getCSGParameters()
                                                                                              .setAccountForGroundDrift(parameters.getAccountForGroundDrift());
                                                                              });

      controllerStatusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class,
                                                                       (statusMessage) -> latestHighLevelControllerStatus.set(HighLevelControllerName.fromByte(
                                                                             statusMessage.getEndHighLevelControllerName())));

      goalReacher = new PDVelocityBasedGoalReacher(referenceFrames.getPelvisZUpFrame(), statusMessageOutputManager, registry);
      commandInputManager.addControllerWaypointGoalCommandConsumer(goalReacher::consumeNewWaypoint);
      commandInputManager.addControllerWaypointGoalListCommandConsumer(goalReacher::consumeNewWaypointList);
      commandInputManager.addControllerReleaseGoalCommand(goalReacher::consumeReleaseGoalCommand);
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

   public void createStepGeneratorNetworkSubscriber(String robotName, RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> baseTopic = ControllerAPI.getBaseTopic(HumanoidControllerAPI.HUMANOID_CONTROL_MODULE_NAME, robotName);
      StepGeneratorNetworkSubscriber stepGeneratorNetworkSubscriber = new StepGeneratorNetworkSubscriber(baseTopic,
                                                                                                         commandInputManager.getCommandInputManager(),
                                                                                                         statusMessageOutputManager,
                                                                                                         realtimeROS2Node);

      stepGeneratorNetworkSubscriber.addMessageValidator(ControllerAPIDefinition.createDefaultMessageValidation());
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(goalReacher.getSCS2YoGraphics());
      group.addChild(stepGenerator.getSCS2YoGraphics());

      return group;
   }

   public StepGeneratorCommandInputManager getStepGeneratorCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStatusMessageOutputManager()
   {
      return statusMessageOutputManager;
   }

   @Override
   public void update(double time)
   {
      commandInputManager.update(time);

      for (int i = 0; i < updatables.size(); i++)
         updatables.get(i).update(time);

      if (latestHighLevelControllerStatus.getValue() == HighLevelControllerName.WALKING
          || latestHighLevelControllerStatus.getValue() == HighLevelControllerName.MPC_CONTROL)
         stepGenerator.update(time);

      if (latestHighLevelControllerStatus.getValue() == HighLevelControllerName.RL_CONTROL)
      {
         goalReacher.update(time);
         VelocityBasedWalkingInputMessage message = goalReacher.getOutputMessage();
         if (message != null)
            controllerCommandInputManager.submitMessage(goalReacher.getOutputMessage());
      }
      else
      {
         goalReacher.clear();
      }
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
