package us.ihmc.avatar.continuousLocomotion;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorInputCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.ConcurrentMessageInputBuffer;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import java.util.ArrayList;
import java.util.List;

public class AvatarWalkingModeManager implements Updatable
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble rcValueThreshold = new YoDouble("rcWalkingModeValueThreshold", registry);
   private final YoBoolean requestedTransitionInProgress = new YoBoolean("requestedTransitionInProgress", registry);
   private final YoBoolean ignoreRCWalkingModeSelection = new YoBoolean("ignoreRCWalkingModeSelection", registry);
   private final YoBoolean ignoreAutoWalkingModeSwitch = new YoBoolean("ignoreAutoWalkingModeSwitch", registry);

   private enum JoystickRequestedWalkingMode {WALKING, AUTO, FAST_WALKING}
   private final YoEnum<JoystickRequestedWalkingMode> yoJoystickRequestedWalkingMode = new YoEnum<>("joystickRequestedWalkingMode", registry, JoystickRequestedWalkingMode.class, false);
   private final YoEnum<HighLevelControllerName> locomotionManagerRequestedWalkingState = new YoEnum<>("locomotionManagerRequestedWalkingState", registry, HighLevelControllerName.class, false);

   private final YoEnum<WalkingStatus> currentWalkingStatus = new YoEnum<>("currentWalkingStatus", registry, WalkingStatus.class, false);
   private final YoEnum<WalkingStatus> walkingStatusBeforeTransition = new YoEnum<>("walkingStatusBeforeTransition", registry, WalkingStatus.class, false);

   private final AvatarFlatGroundDetector terrainIdentifier;
   private final ConcurrentMessageInputBuffer messageListener;

   private final CommandInputManager walkingCommandInputManager;
   private final StepGeneratorCommandInputManager stepGeneratorCommandInputManager;

   private final HighLevelControllerStateCommand controllerStateCommand = new HighLevelControllerStateCommand();
   private final PauseWalkingCommand pauseWalkingCommand = new PauseWalkingCommand();
   private final ContinuousStepGeneratorInputCommand stepGeneratorInputCommand = new ContinuousStepGeneratorInputCommand();

   private HighLevelControllerName currentController = HighLevelControllerName.DO_NOTHING_BEHAVIOR;

   public AvatarWalkingModeManager(DRCRobotModel robotModel,
                                   FullHumanoidRobotModel fullRobotModel,
                                   StatusMessageOutputManager statusMessageOutputManager,
                                   CommandInputManager controllerCommandInputManager,
                                   StepGeneratorCommandInputManager stepGeneratorCommandInputManager,
                                   HumanoidReferenceFrames referenceFrames,
                                   HumanoidRobotContextData humanoidRobotContextData,
                                   YoRegistry parentRegistry)
   {
      this.walkingCommandInputManager = controllerCommandInputManager;
      this.stepGeneratorCommandInputManager = stepGeneratorCommandInputManager;

      rcValueThreshold.set(1.0);
      ignoreAutoWalkingModeSwitch.set(false);

      currentWalkingStatus.set(WalkingStatus.PAUSED);
      walkingStatusBeforeTransition.set(WalkingStatus.PAUSED);

      yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.AUTO);
      yoJoystickRequestedWalkingMode.addListener(change -> updateRequestedWalkingState());

      locomotionManagerRequestedWalkingState.set(HighLevelControllerName.WALKING);
      locomotionManagerRequestedWalkingState.addListener(change ->
                                                         {
                                                            if (locomotionManagerRequestedWalkingState.getEnumValue() != currentController)
                                                            {
                                                               recordWalkingStatusBeforeTransition();
                                                               //pauseWalking();
                                                               requestedTransitionInProgress.set(true);
                                                            }
                                                            else
                                                            {
                                                               requestedTransitionInProgress.set(false);
                                                            }
                                                         });


      List<Class<? extends Settable<?>>> messagesToRegister = new ArrayList<>();
      messagesToRegister.add(CapturabilityBasedStatus.class);
      messagesToRegister.add(HighLevelStateChangeStatusMessage.class);
      messagesToRegister.add(FootstepStatusMessage.class);
      messagesToRegister.add(WalkingStatusMessage.class);
      messagesToRegister.add(PauseWalkingMessage.class);

      messageListener = new ConcurrentMessageInputBuffer(null, messagesToRegister, 35);
      terrainIdentifier = new AvatarFlatGroundDetector(robotModel, fullRobotModel, referenceFrames, messageListener, registry);

      stepGeneratorCommandInputManager.addPlanarRegionsListCommandConsumer(terrainIdentifier::acceptPlanarRegionsListCommand);
      stepGeneratorCommandInputManager.addContinuousStepGeneratorInputCommandConsumer(this::acceptContinuousStepGeneratorInputCommand);

      statusMessageOutputManager.attachStatusMessageListener(CapturabilityBasedStatus.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, messageListener::submitMessage);

      parentRegistry.addChild(registry);
   }

   private void pauseWalking()
   {
      if (isReadyForPause())
      {
         stepGeneratorInputCommand.setWalk(false);
      }
      else
      {
         stepGeneratorInputCommand.setForwardVelocity(0.0);
         stepGeneratorInputCommand.setLateralVelocity(0.0);
         stepGeneratorInputCommand.setTurnVelocity(0.0);
      }
   }

   private boolean isWalkingPaused()
   {
      return currentWalkingStatus.getEnumValue() == WalkingStatus.PAUSED || currentWalkingStatus.getEnumValue() == WalkingStatus.COMPLETED || currentWalkingStatus.getEnumValue() == WalkingStatus.ABORT_REQUESTED;
   }

   private boolean isReadyForPause()
   {
      return terrainIdentifier.areFeetSquared();
      //return currentWalkingStatus.getEnumValue() == WalkingStatus.PAUSED || currentWalkingStatus.getEnumValue() == WalkingStatus.COMPLETED;
   }

   @Override
   public void update(double time)
   {
      consumeMessages();

      if (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.AUTO)
         terrainIdentifier.update();

      if (!ignoreRCWalkingModeSelection.getBooleanValue() && requestedTransitionInProgress.getBooleanValue())
      {
         if (stepGeneratorCommandInputManager.getCommandInputManager().isNewCommandAvailable(ContinuousStepGeneratorInputCommand.class))
         {
            stepGeneratorInputCommand.set(stepGeneratorCommandInputManager.getCommandInputManager().pollNewestCommand(ContinuousStepGeneratorInputCommand.class));
            stepGeneratorCommandInputManager.getCommandInputManager().clearCommands(ContinuousStepGeneratorInputCommand.class);
         }

         // If walking is paused, submit high level controller state change command
         // Once high level state has successfully changed, set walking status to whatever it was before state change was requested
         if (locomotionManagerRequestedWalkingState.getEnumValue() != currentController)
         {
            if (isWalkingPaused())
            {
               stepGeneratorInputCommand.setWalk(false);
               controllerStateCommand.setHighLevelControllerName(locomotionManagerRequestedWalkingState.getEnumValue());
               walkingCommandInputManager.submitCommand(controllerStateCommand);
            }
            else
            {
               pauseWalking();
            }
         }
         else
         {
            stepGeneratorInputCommand.setWalk(walkingStatusBeforeTransition.getEnumValue() != WalkingStatus.PAUSED || walkingStatusBeforeTransition.getEnumValue() != WalkingStatus.COMPLETED);
            requestedTransitionInProgress.set(false);
         }

         stepGeneratorCommandInputManager.getCommandInputManager().submitCommand(stepGeneratorInputCommand);
      }
   }

   private void consumeMessages()
   {
      if (messageListener.isNewMessageAvailable(HighLevelStateChangeStatusMessage.class))
      {
         currentController = HighLevelControllerName.fromByte(messageListener.pollNewestMessage(HighLevelStateChangeStatusMessage.class).getEndHighLevelControllerName());
         messageListener.clearMessages(HighLevelStateChangeStatusMessage.class);
      }

      if (messageListener.isNewMessageAvailable(WalkingStatusMessage.class))
      {
         WalkingStatusMessage walkingStatusMessage = messageListener.pollNewestMessage(WalkingStatusMessage.class);

//         if (walkingStatusMessage.getWalkingStatus() == -1)
//            currentWalkingStatus.set(WalkingStatus.PAUSED);
//         else
            currentWalkingStatus.set(WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus()));

         messageListener.clearMessages(WalkingStatusMessage.class);
      }

      terrainIdentifier.consumeMessages();
   }

   private void updateRequestedWalkingState()
   {
      if (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.FAST_WALKING || (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.AUTO && !ignoreAutoWalkingModeSwitch.getBooleanValue() && terrainIdentifier.flatGroundDetected()))
         locomotionManagerRequestedWalkingState.set(HighLevelControllerName.CUSTOM1);

      else if (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.WALKING || (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.AUTO && !ignoreAutoWalkingModeSwitch.getBooleanValue() && !terrainIdentifier.flatGroundDetected()))
         locomotionManagerRequestedWalkingState.set(HighLevelControllerName.WALKING);
   }

   private void recordWalkingStatusBeforeTransition()
   {
      walkingStatusBeforeTransition.set(currentWalkingStatus.getEnumValue());
   }

//   private boolean shouldExecuteTransition()
//   {
//      if (locomotionManagerRequestedWalkingState.getEnumValue() != currentController
//          && (walkingStatusMessage.getWalkingStatus() == -1 || WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus()) == WalkingStatus.PAUSED))
//      {
//         return true;
//      }
//
//      else if (locomotionManagerRequestedWalkingState.getEnumValue() != currentController && WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus()) != WalkingStatus.PAUSED)
//      {
//         //walkingCommandInputManager.clearCommands(PauseWalkingCommand.class);
//         pauseWalkingCommand.setPauseRequested(true);
//         walkingCommandInputManager.submitCommand(pauseWalkingCommand);
//         return false;
//      }
//
//      else
//      {
//         return false;
//      }
//   }

   private void acceptContinuousStepGeneratorInputCommand(ContinuousStepGeneratorInputCommand continuousStepGeneratorInputCommand)
   {
      setWalkingModeFromRC(continuousStepGeneratorInputCommand.getWalkingModeSwitchValue());
   }

   private void setWalkingModeFromRC(double rcValue)
   {
      if (rcValue <= -rcValueThreshold.getDoubleValue()/3.0)
         yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.WALKING);
      else if (rcValue >= rcValueThreshold.getDoubleValue()/3.0)
         yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.FAST_WALKING);
      else
         yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.AUTO);
   }
}
