package us.ihmc.quadrupedRobotics.controller;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DoNothingControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.FreezeControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandReadyControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.ControllerFailedTransition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.QuadrupedFeetLoadedToWalkingStandTransition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedExitWalkingControllerState;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedSitDownControllerState;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedWalkingControllerState;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.output.JointIntegratorComponent;
import us.ihmc.quadrupedRobotics.output.OutputProcessorBuilder;
import us.ihmc.quadrupedRobotics.output.StateChangeSmootherComponent;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSitDownParameters;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorModeSubscriber;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal
 * finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedControllerManager implements RobotController, CloseableAndDisposable
{
   public static final HighLevelControllerName sitDownStateName = HighLevelControllerName.CUSTOM1;

   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEnum<HighLevelControllerName> requestedControllerState;
   private final AtomicReference<HighLevelControllerName> requestedControllerStateReference = new AtomicReference<>();
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<HighLevelControllerName, HighLevelControllerState> stateMachine;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;

   private final OutputProcessor outputProcessor;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;
   private final JointDesiredOutputList lowLevelControllerOutput;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final HighLevelStateChangeStatusMessage stateChangeMessage = new HighLevelStateChangeStatusMessage();
   private final WalkingControllerFailureStatusMessage walkingControllerFailureStatusMessage = new WalkingControllerFailureStatusMessage();

   private final BooleanProvider trustFootSwitchesInSwing;
   private final BooleanProvider trustFootSwitchesInSupport;

   private StateEstimatorModeSubscriber stateEstimatorModeSubscriber;

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                     HighLevelControllerName initialControllerState, HighLevelControllerState calibrationState)
   {
      this.controllerToolbox = new QuadrupedControllerToolbox(runtimeEnvironment, physicalProperties, registry, runtimeEnvironment.getGraphicsListRegistry());
      this.runtimeEnvironment = runtimeEnvironment;
      this.lowLevelControllerOutput = runtimeEnvironment.getJointDesiredOutputList();
      this.yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(
            runtimeEnvironment.getFullRobotModel().getControllableOneDoFJoints(), registry);

      // Initialize control modules
      this.controlManagerFactory = new QuadrupedControlManagerFactory(controllerToolbox, physicalProperties, runtimeEnvironment.getGraphicsListRegistry(),
                                                                      registry);

      HighLevelControllerName.setName(sitDownStateName, QuadrupedSitDownControllerState.name);
      requestedControllerState = new YoEnum<>("requestedControllerState", registry, HighLevelControllerName.class, true);
      trustFootSwitchesInSwing = new BooleanParameter("trustFootSwitchesInSwing", registry, physicalProperties.trustFootSwitchesInSwing());
      trustFootSwitchesInSupport = new BooleanParameter("trustFootSwitchesInSupport", registry, physicalProperties.trustFootSwitchesInSupport());

      commandInputManager = new CommandInputManager(QuadrupedControllerAPIDefinition.getQuadrupedSupportedCommands());
      try
      {
         commandInputManager.registerConversionHelper(new ClearDelayQueueConverter(QuadrupedControllerAPIDefinition.getQuadrupedSupportedCommands()));
      }
      catch (InstantiationException | IllegalAccessException e)
      {
         e.printStackTrace();
      }
      statusMessageOutputManager = new StatusMessageOutputManager(QuadrupedControllerAPIDefinition.getQuadrupedSupportedStatusMessages());

      controlManagerFactory.getOrCreateFeetManager();
      controlManagerFactory.getOrCreateBodyOrientationManager();
      controlManagerFactory.getOrCreateBalanceManager();
      controlManagerFactory.getOrCreateJointSpaceManager();

      // Initialize output processor
      StateChangeSmootherComponent stateChangeSmootherComponent = new StateChangeSmootherComponent(runtimeEnvironment, registry);
      JointIntegratorComponent jointControlComponent = new JointIntegratorComponent(runtimeEnvironment, registry);
      controlManagerFactory.getOrCreateFeetManager().attachStateChangedListener(stateChangeSmootherComponent.createFiniteStateMachineStateChangedListener());
      OutputProcessorBuilder outputProcessorBuilder = new OutputProcessorBuilder(runtimeEnvironment.getFullRobotModel());
      outputProcessorBuilder.addComponent(stateChangeSmootherComponent);
      outputProcessorBuilder.addComponent(jointControlComponent);
      outputProcessor = outputProcessorBuilder.build();

      requestedControllerState.set(null);
      requestedControllerState.addVariableChangedListener(v -> {
         HighLevelControllerName currentRequestedState = requestedControllerState.getEnumValue();
         if (currentRequestedState != null)
         {
            requestedControllerStateReference.set(currentRequestedState);
            requestedControllerState.set(null);
         }
      });

      if (initialControllerState == null)
      {
         initialControllerState = runtimeEnvironment.getHighLevelControllerParameters().getDefaultInitialControllerState();
      }

      this.stateMachine = buildStateMachine(runtimeEnvironment, initialControllerState, calibrationState);
   }

   public void registerHighLevelStateChangedListener(StateChangedListener<HighLevelControllerName> listener)
   {
      stateMachine.addStateChangedListener(listener);
   }

   public State getState(HighLevelControllerName state)
   {
      return stateMachine.getState(state);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      if (commandInputManager.isNewCommandAvailable(HighLevelControllerStateCommand.class))
      {
         requestedControllerState.set(commandInputManager.pollNewestCommand(HighLevelControllerStateCommand.class).getHighLevelControllerName());
      }

      // update controller state machine
      try
      {
         stateMachine.doActionAndTransition();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         statusMessageOutputManager.reportStatusMessage(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.CONTROLLER_RUN,
                                                                                                             e.getMessage()));
      }

      // update contact state used for state estimation
      switch (stateMachine.getCurrentStateKey())
      {
      case DO_NOTHING_BEHAVIOR:
      case STAND_PREP_STATE:
      case FREEZE_STATE:
      case CUSTOM1:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitchInSwing(false);
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitchInSupport(false);
         }

         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_RIGHT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_RIGHT).setFootContactState(true);
         if (stateEstimatorModeSubscriber != null)
            stateEstimatorModeSubscriber.requestStateEstimatorMode(StateEstimatorMode.FROZEN);
         break;

      case STAND_READY:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitchInSwing(trustFootSwitchesInSwing.getValue());
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitchInSupport(trustFootSwitchesInSupport.getValue());
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(true);
         }
         if (stateEstimatorModeSubscriber != null)
            stateEstimatorModeSubscriber.requestStateEstimatorMode(StateEstimatorMode.NORMAL);
         break;
      case STAND_TRANSITION_STATE:
      case WALKING:
      default:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitchInSwing(trustFootSwitchesInSwing.getValue());
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitchInSupport(trustFootSwitchesInSupport.getValue());
            boolean inContact = controllerToolbox.getContactState(robotQuadrant) == ContactState.IN_CONTACT;
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(inContact);
         }
         break;
      }

      // update fall detector
      controllerToolbox.getFallDetector().detect();

      if (controllerToolbox.getControllerFailedBoolean().getBooleanValue())
      {
         walkingControllerFailureStatusMessage.falling_direction_.set(runtimeEnvironment.getFullRobotModel().getRootJoint().getJointTwist().getLinearPart());
         statusMessageOutputManager.reportStatusMessage(walkingControllerFailureStatusMessage);
      }

      // update output processor
      outputProcessor.update();

      copyJointDesiredsToJoints();

      requestedControllerStateReference.set(null);
   }

   public YoBoolean getControllerFailedBoolean()
   {
      return controllerToolbox.getControllerFailedBoolean();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return "A proxy controller for switching between multiple subcontrollers";
   }

   public RobotMotionStatusHolder getMotionStatusHolder()
   {
      return motionStatusHolder;
   }

   private StateMachine<HighLevelControllerName, HighLevelControllerState> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
                                                                                             HighLevelControllerName initialControllerState,
                                                                                             HighLevelControllerState calibrationState)
   {
      OneDoFJointBasics[] controlledJoints = runtimeEnvironment.getFullRobotModel().getControllableOneDoFJoints();
      HighLevelControllerParameters highLevelControllerParameters = runtimeEnvironment.getHighLevelControllerParameters();
      QuadrupedSitDownParameters sitDownParameters = runtimeEnvironment.getSitDownParameters();
      JointDesiredOutputList jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();

      DoNothingControllerState doNothingState = new DoNothingControllerState(controlledJoints, highLevelControllerParameters);
      StandPrepControllerState standPrepState = new StandPrepControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      StandReadyControllerState standReadyState = new StandReadyControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      QuadrupedWalkingControllerState walkingState = new QuadrupedWalkingControllerState(runtimeEnvironment, controllerToolbox, commandInputManager,
                                                                                         statusMessageOutputManager, controlManagerFactory);
      FreezeControllerState freezeState = new FreezeControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      SmoothTransitionControllerState standTransitionState = new SmoothTransitionControllerState("toWalking", HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                                                 standReadyState, walkingState, controlledJoints,
                                                                                                 highLevelControllerParameters);
      SmoothTransitionControllerState exitWalkingState = new QuadrupedExitWalkingControllerState(walkingState, freezeState, controlledJoints,
                                                                                                 highLevelControllerParameters);

      QuadrupedSitDownControllerState sitDownState = new QuadrupedSitDownControllerState(sitDownStateName, controlledJoints, highLevelControllerParameters,
                                                                                         sitDownParameters, jointDesiredOutputList);

      StateMachineFactory<HighLevelControllerName, HighLevelControllerState> factory = new StateMachineFactory<>(HighLevelControllerName.class);
      factory.setNamePrefix("controller").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());

      factory.addState(HighLevelControllerName.DO_NOTHING_BEHAVIOR, doNothingState);
      factory.addState(HighLevelControllerName.STAND_PREP_STATE, standPrepState);
      factory.addState(HighLevelControllerName.STAND_READY, standReadyState);
      factory.addState(HighLevelControllerName.FREEZE_STATE, freezeState);
      factory.addState(HighLevelControllerName.WALKING, walkingState);
      factory.addState(HighLevelControllerName.STAND_TRANSITION_STATE, standTransitionState);
      factory.addState(HighLevelControllerName.EXIT_WALKING, exitWalkingState);
      factory.addState(sitDownStateName, sitDownState);

      // Manually triggered events to transition to main controllers.
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.STAND_TRANSITION_STATE,
                            createRequestedTransition(HighLevelControllerName.WALKING));
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.STAND_TRANSITION_STATE,
                            createRequestedTransition(HighLevelControllerName.STAND_TRANSITION_STATE));
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.STAND_PREP_STATE,
                            createRequestedTransition(HighLevelControllerName.STAND_PREP_STATE));
      factory.addTransition(HighLevelControllerName.STAND_READY, sitDownStateName, createRequestedTransition(sitDownStateName));
      factory.addTransition(HighLevelControllerName.FREEZE_STATE, sitDownStateName, createRequestedTransition(sitDownStateName));
      factory.addTransition(HighLevelControllerName.FREEZE_STATE, HighLevelControllerName.STAND_PREP_STATE,
                            createRequestedTransition(HighLevelControllerName.STAND_PREP_STATE));
      factory.addTransition(HighLevelControllerName.STAND_PREP_STATE, HighLevelControllerName.FREEZE_STATE,
                            createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
      factory.addTransition(HighLevelControllerName.WALKING, HighLevelControllerName.STAND_READY,
                            createRequestedTransition(HighLevelControllerName.STAND_READY));
      factory.addTransition(HighLevelControllerName.WALKING, HighLevelControllerName.EXIT_WALKING,
                            createRequestedTransition(HighLevelControllerName.EXIT_WALKING));

      factory.addTransition(sitDownStateName, HighLevelControllerName.FREEZE_STATE, createRequestedTransition(HighLevelControllerName.FREEZE_STATE));

      factory.addTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR, HighLevelControllerName.FREEZE_STATE,
                            createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
      factory.addTransition(HighLevelControllerName.WALKING, HighLevelControllerName.FREEZE_STATE,
                            createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.FREEZE_STATE,
                            createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
      factory.addTransition(HighLevelControllerName.STAND_TRANSITION_STATE, HighLevelControllerName.FREEZE_STATE,
                            createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
      factory.addTransition(HighLevelControllerName.EXIT_WALKING, HighLevelControllerName.FREEZE_STATE,
                            createRequestedTransition(HighLevelControllerName.FREEZE_STATE));

      // Trigger do nothing
      for (HighLevelControllerName state : HighLevelControllerName.values)
      {
         factory.addTransition(state, HighLevelControllerName.DO_NOTHING_BEHAVIOR, createRequestedTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR));
      }

      if (calibrationState != null)
      {
         factory.addState(HighLevelControllerName.CALIBRATION, calibrationState);
         factory.addTransition(HighLevelControllerName.FREEZE_STATE, HighLevelControllerName.CALIBRATION, createRequestedTransition(HighLevelControllerName.CALIBRATION));
         factory.addTransition(HighLevelControllerName.CALIBRATION, HighLevelControllerName.FREEZE_STATE, createRequestedTransition(HighLevelControllerName.FREEZE_STATE));
         factory.addDoneTransition(HighLevelControllerName.CALIBRATION, HighLevelControllerName.FREEZE_STATE);
      }

      // Set up standard operating transitions
      factory.addDoneTransition(HighLevelControllerName.STAND_PREP_STATE, HighLevelControllerName.STAND_READY);
      factory.addDoneTransition(HighLevelControllerName.STAND_TRANSITION_STATE, HighLevelControllerName.WALKING);
      factory.addDoneTransition(sitDownStateName, HighLevelControllerName.FREEZE_STATE);
      factory.addDoneTransition(HighLevelControllerName.EXIT_WALKING, sitDownStateName);

      // Set up walking controller failure transition
      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      factory.addTransition(HighLevelControllerName.WALKING, fallbackControllerState, new ControllerFailedTransition(controllerToolbox.getControllerFailedBoolean()));
      factory.addTransition(fallbackControllerState, HighLevelControllerName.STAND_PREP_STATE,
                            createRequestedTransition(HighLevelControllerName.STAND_PREP_STATE));

      // Set up foot loaded transition
      StateTransitionCondition footLoadedTransition = new QuadrupedFeetLoadedToWalkingStandTransition(HighLevelControllerName.STAND_TRANSITION_STATE, requestedControllerState,
                                                                                                      runtimeEnvironment.getFootSwitches(),
                                                                                                      runtimeEnvironment.getControlDT(),
                                                                                                      runtimeEnvironment.getFullRobotModel().getTotalMass(),
                                                                                                      runtimeEnvironment.getGravity(),
                                                                                                      runtimeEnvironment.getHighLevelControllerParameters(),
                                                                                                      registry);
      factory.addTransition(HighLevelControllerName.STAND_READY, new StateTransition<>(HighLevelControllerName.STAND_TRANSITION_STATE, footLoadedTransition));

      factory.addStateChangedListener((from, to) -> {
         byte fromByte = from == null ? -1 : from.toByte();
         byte toByte = to == null ? -1 : to.toByte();
         stateChangeMessage.setInitialHighLevelControllerName(fromByte);
         stateChangeMessage.setEndHighLevelControllerName(toByte);
         statusMessageOutputManager.reportStatusMessage(stateChangeMessage);
      });

      registry.addChild(doNothingState.getYoVariableRegistry());
      registry.addChild(standPrepState.getYoVariableRegistry());
      registry.addChild(standReadyState.getYoVariableRegistry());
      registry.addChild(freezeState.getYoVariableRegistry());
      registry.addChild(walkingState.getYoVariableRegistry());
      registry.addChild(standTransitionState.getYoVariableRegistry());
      registry.addChild(exitWalkingState.getYoVariableRegistry());
      registry.addChild(sitDownState.getYoVariableRegistry());

      return factory.build(initialControllerState);
   }

   private StateTransitionCondition createRequestedTransition(HighLevelControllerName endState)
   {
      return time -> requestedControllerStateReference.get() != null && requestedControllerStateReference.get() == endState;
   }

   private void copyJointDesiredsToJoints()
   {
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         OneDoFJointBasics controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         JointDesiredOutputReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   public void createControllerNetworkSubscriber(String robotName, RealtimeRos2Node realtimeRos2Node)
   {
      ROS2Tools.MessageTopicNameGenerator subscriberTopicNameGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
      ROS2Tools.MessageTopicNameGenerator publisherTopicNameGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(subscriberTopicNameGenerator, commandInputManager,
                                                                                                publisherTopicNameGenerator, statusMessageOutputManager,
                                                                                                realtimeRos2Node);
      controllerNetworkSubscriber.addMessageCollector(QuadrupedControllerAPIDefinition.createDefaultMessageIDExtractor());
      controllerNetworkSubscriber.addMessageValidator(QuadrupedControllerAPIDefinition.createDefaultMessageValidation());
   }

   public void resetSteppingState()
   {
      QuadrupedWalkingControllerState steppingState = (QuadrupedWalkingControllerState) stateMachine.getState(HighLevelControllerName.WALKING);
      steppingState.onEntry();
   }

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
   }

   public void setStateEstimatorModeSubscriber(StateEstimatorModeSubscriber stateEstimatorModeSubscriber)
   {
      this.stateEstimatorModeSubscriber = stateEstimatorModeSubscriber;
   }
}
