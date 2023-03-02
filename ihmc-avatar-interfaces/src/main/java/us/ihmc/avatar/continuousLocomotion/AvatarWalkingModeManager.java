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
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import java.util.ArrayList;
import java.util.List;

public class AvatarWalkingModeManager implements Updatable
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble rcValueThreshold;
   private final YoBoolean ignoreRCWalkingModeSelection;
   private final YoBoolean ignoreAutoWalkingModeSwitch; //TODO probably rename

   private enum JoystickRequestedWalkingMode {WALKING, AUTO, FAST_WALKING}
   private final YoEnum<JoystickRequestedWalkingMode> yoJoystickRequestedWalkingMode;
   private final YoEnum<HighLevelControllerName> locomotionManagerRequestedWalkingState;

   private final AvatarFlatGroundDetector terrainIdentifier;
   private final ConcurrentMessageInputBuffer messageListener;

   private final CommandInputManager walkingCommandInputManager;

   private final HighLevelControllerStateCommand controllerStateCommand = new HighLevelControllerStateCommand();
   private final PauseWalkingCommand pauseWalkingCommand = new PauseWalkingCommand();

   private final HumanoidRobotContextData humanoidRobotContextData;
   private final HumanoidReferenceFrames referenceFrames;

   //private final AtomicReference<FootstepStatus> latestFootstepStatusReceived = new AtomicReference<>(null);
   private final WalkingStatusMessage walkingStatusMessage = new WalkingStatusMessage();
   private final PauseWalkingMessage walkingMessageBeforeTransition = new PauseWalkingMessage();

   private HighLevelControllerName currentController = HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   private WalkingStatus walkingStatusBeforeTransition = WalkingStatus.PAUSED;

   public AvatarWalkingModeManager(DRCRobotModel robotModel,
                                   FullHumanoidRobotModel fullRobotModel,
                                   StatusMessageOutputManager statusMessageOutputManager,
                                   CommandInputManager walkingCommandInputManager,
                                   StepGeneratorCommandInputManager csgCommandInputManager,
                                   HumanoidReferenceFrames referenceFrames,
                                   HumanoidRobotContextData humanoidRobotContextData,
                                   YoRegistry parentRegistry)
   {
      rcValueThreshold = new YoDouble("rcWalkingModeValueThreshold", registry);
      rcValueThreshold.set(1.0);

      ignoreRCWalkingModeSelection = new YoBoolean("ignoreRCWalkingModeSelection", registry);
      ignoreAutoWalkingModeSwitch = new YoBoolean("ignoreAutoWalkingModeSwitch", registry);
      ignoreAutoWalkingModeSwitch.set(true);

      yoJoystickRequestedWalkingMode = new YoEnum<>("yoJoystickRequestedWalkingMode", registry, JoystickRequestedWalkingMode.class, false);
      yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.WALKING);

      locomotionManagerRequestedWalkingState = new YoEnum<>("locomotionManagerRequestedWalkingState", registry, HighLevelControllerName.class, false);
      locomotionManagerRequestedWalkingState.set(HighLevelControllerName.WALKING);
      locomotionManagerRequestedWalkingState.addListener(change -> recordWalkingStatusBeforeTransition());

      this.walkingCommandInputManager = walkingCommandInputManager;

      this.humanoidRobotContextData = humanoidRobotContextData;
      this.referenceFrames = referenceFrames;

      List<Class<? extends Settable<?>>> messagesToRegister = new ArrayList<>();
      messagesToRegister.add(CapturabilityBasedStatus.class);
      messagesToRegister.add(HighLevelStateChangeStatusMessage.class);
      messagesToRegister.add(FootstepStatusMessage.class);
      messagesToRegister.add(WalkingStatusMessage.class);
      messagesToRegister.add(PauseWalkingMessage.class);

      messageListener = new ConcurrentMessageInputBuffer(messagesToRegister);
      terrainIdentifier = new AvatarFlatGroundDetector(robotModel, fullRobotModel, referenceFrames, messageListener, registry);

      csgCommandInputManager.addPlanarRegionsListCommandConsumer(terrainIdentifier::acceptPlanarRegionsListCommand);
      csgCommandInputManager.addContinuousStepGeneratorInputCommandConsumer(this::acceptContinuousStepGeneratorInputCommand);

      statusMessageOutputManager.attachStatusMessageListener(CapturabilityBasedStatus.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(HighLevelStateChangeStatusMessage.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, messageListener::submitMessage);
      statusMessageOutputManager.attachStatusMessageListener(PauseWalkingMessage.class, messageListener::submitMessage);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update(double time)
   {
      consumeMessages();
      updateRequestedWalkingState();

      if (shouldExecuteTransition())
      {
         walkingCommandInputManager.clearCommands(HighLevelControllerStateCommand.class);
         controllerStateCommand.setHighLevelControllerName(locomotionManagerRequestedWalkingState.getEnumValue());
         walkingCommandInputManager.submitCommand(controllerStateCommand);

         walkingCommandInputManager.clearCommands(PauseWalkingCommand.class);
         pauseWalkingCommand.setFromMessage(walkingMessageBeforeTransition);//walkingStatusBeforeTransition == WalkingStatus.PAUSED || walkingStatusBeforeTransition == WalkingStatus.ABORT_REQUESTED);
         walkingCommandInputManager.submitCommand(pauseWalkingCommand);
      }
   }

   private void consumeMessages()
   {
      if (messageListener.isNewMessageAvailable(HighLevelStateChangeStatusMessage.class))
      {
         currentController = HighLevelControllerName.fromByte(messageListener.pollNewestMessage(HighLevelStateChangeStatusMessage.class).getEndHighLevelControllerName());
      }

      if (messageListener.isNewMessageAvailable(WalkingStatusMessage.class))
      {
         walkingStatusMessage.set(messageListener.pollNewestMessage(WalkingStatusMessage.class));
      }
   }

   private void updateRequestedWalkingState()
   {
      if (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.FAST_WALKING || (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.AUTO && terrainIdentifier.flatGroundDetected() && !ignoreAutoWalkingModeSwitch.getBooleanValue()))
         locomotionManagerRequestedWalkingState.set(HighLevelControllerName.CUSTOM1);

      else if (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.WALKING || (yoJoystickRequestedWalkingMode.getEnumValue() == JoystickRequestedWalkingMode.AUTO && !terrainIdentifier.flatGroundDetected() && !ignoreAutoWalkingModeSwitch.getBooleanValue()))
         locomotionManagerRequestedWalkingState.set(HighLevelControllerName.WALKING);
   }

   private void recordWalkingStatusBeforeTransition()
   {
      walkingStatusBeforeTransition = WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus());

      if (messageListener.isNewMessageAvailable(PauseWalkingMessage.class))
      {
         walkingMessageBeforeTransition.set(messageListener.pollNewestMessage(PauseWalkingMessage.class));
      }
   }

   private boolean shouldExecuteTransition()
   {
      if (locomotionManagerRequestedWalkingState.getEnumValue() != currentController && WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus()) == WalkingStatus.PAUSED)
      {
         return true;
      }

      else if (locomotionManagerRequestedWalkingState.getEnumValue() != currentController && WalkingStatus.fromByte(walkingStatusMessage.getWalkingStatus()) != WalkingStatus.PAUSED)
      {
         walkingCommandInputManager.clearCommands(PauseWalkingCommand.class);
         pauseWalkingCommand.setPauseRequested(true);
         walkingCommandInputManager.submitCommand(pauseWalkingCommand);
         return false;
      }

      else
      {
         return false;
      }
   }

   private void acceptContinuousStepGeneratorInputCommand(ContinuousStepGeneratorInputCommand continuousStepGeneratorInputCommand)
   {
      setWalkingModeFromRC(continuousStepGeneratorInputCommand.getWalkingModeSwitchValue());
   }

   private void setWalkingModeFromRC(double rcValue)
   {
      if (!ignoreRCWalkingModeSelection.getBooleanValue())
      {
         if (rcValue <= -rcValueThreshold.getDoubleValue()/3.0)
            yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.WALKING);
         else if (rcValue >= rcValueThreshold.getDoubleValue()/3.0)
            yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.FAST_WALKING);
         else
            yoJoystickRequestedWalkingMode.set(JoystickRequestedWalkingMode.AUTO);
      }
   }
}
