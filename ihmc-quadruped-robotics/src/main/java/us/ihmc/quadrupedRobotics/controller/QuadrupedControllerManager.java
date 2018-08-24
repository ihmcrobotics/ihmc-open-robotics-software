package us.ihmc.quadrupedRobotics.controller;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.*;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedControllerStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedWalkingControllerState;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.concurrent.atomic.AtomicReference;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal
 * finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedControllerManager implements RobotController, CloseableAndDisposable
{
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEnum<QuadrupedControllerRequestedEvent> requestedControllerState = new YoEnum<>("requestedControllerState", registry,
                                                                                                   QuadrupedControllerRequestedEvent.class, true);
   private final YoEnum<QuadrupedControllerRequestedEvent> lastEvent = new YoEnum<>("lastEvent", registry, QuadrupedControllerRequestedEvent.class);

   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<HighLevelControllerName, HighLevelControllerState> stateMachine;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final HighLevelStateChangeStatusMessage stateChangeMessage = new HighLevelStateChangeStatusMessage();
   private final WalkingControllerFailureStatusMessage walkingControllerFailureStatusMessage = new WalkingControllerFailureStatusMessage();

   private final AtomicReference<QuadrupedControllerRequestedEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties)
   {
      this.controllerToolbox = new QuadrupedControllerToolbox(runtimeEnvironment, physicalProperties, registry, runtimeEnvironment.getGraphicsListRegistry());
      this.runtimeEnvironment = runtimeEnvironment;

      // Initialize control modules
      this.controlManagerFactory = new QuadrupedControlManagerFactory(controllerToolbox, physicalProperties, runtimeEnvironment.getGraphicsListRegistry(),
                                                                      registry);

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

      requestedControllerState.set(null);
      requestedControllerState.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            QuadrupedControllerRequestedEvent requestedControllerEvent = requestedControllerState.getEnumValue();
            if (requestedControllerEvent != null)
            {
               requestedEvent.set(requestedControllerEvent);
               requestedControllerState.set(null);
            }
         }
      });

      this.stateMachine = buildStateMachine(runtimeEnvironment);
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
      if (commandInputManager.isNewCommandAvailable(QuadrupedRequestedControllerStateCommand.class))
      {
         requestedEvent.set(commandInputManager.pollNewestCommand(QuadrupedRequestedControllerStateCommand.class).getRequestedControllerState());
      }

      // update requested events
      QuadrupedControllerRequestedEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         lastEvent.set(reqEvent);
         requestedControllerState.set(reqEvent);
      }

      // update controller state machine
      stateMachine.doActionAndTransition();

      // update contact state used for state estimation
      switch (stateMachine.getCurrentStateKey())
      {
      case DO_NOTHING_BEHAVIOR:
      case STAND_PREP_STATE:
      case STAND_READY:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitch(false);
         }

         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_RIGHT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_RIGHT).setFootContactState(true);
         break;
      default:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).trustFootSwitch(true);
            if (controllerToolbox.getContactState(robotQuadrant) == ContactState.IN_CONTACT)
            {
               runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(true);
            }
            else
            {
               runtimeEnvironment.getFootSwitches().get(robotQuadrant).setFootContactState(false);
            }
         }
         break;
      }

      // update fall detector
      if (controllerToolbox.getFallDetector().detect())
      {
         requestedControllerState.set(QuadrupedControllerRequestedEvent.REQUEST_FALL);
         walkingControllerFailureStatusMessage.falling_direction_.set(runtimeEnvironment.getFullRobotModel().getRootJoint().getJointTwist().getLinearPart());
         statusMessageOutputManager.reportStatusMessage(walkingControllerFailureStatusMessage);
      }
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

   private StateMachine<HighLevelControllerName, HighLevelControllerState> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment)
   {
      OneDoFJointBasics[] controlledJoints = runtimeEnvironment.getFullRobotModel().getControllableOneDoFJoints();
      HighLevelControllerParameters highLevelControllerParameters = runtimeEnvironment.getHighLevelControllerParameters();
      JointDesiredOutputList jointDesiredOutputList = runtimeEnvironment.getJointDesiredOutputList();

      DoNothingControllerState doNothingState = new DoNothingControllerState(controlledJoints, highLevelControllerParameters);
      StandPrepControllerState standPrepState = new StandPrepControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      StandReadyControllerState standReadyState = new StandReadyControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);
      QuadrupedWalkingControllerState walkingState = new QuadrupedWalkingControllerState(runtimeEnvironment, controllerToolbox, commandInputManager,
                                                                                         statusMessageOutputManager, controlManagerFactory, registry);
      SmoothTransitionControllerState standTransitionState = new SmoothTransitionControllerState("toWalking", HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                                                 standReadyState, walkingState, controlledJoints,
                                                                                                 highLevelControllerParameters);
      SmoothTransitionControllerState exitWalkingState = new SmoothTransitionControllerState("exitWalking", HighLevelControllerName.EXIT_WALKING, walkingState,
                                                                                             standPrepState, controlledJoints, highLevelControllerParameters);
      FreezeControllerState freezeState = new FreezeControllerState(controlledJoints, highLevelControllerParameters, jointDesiredOutputList);

      StateMachineFactory<HighLevelControllerName, HighLevelControllerState> factory = new StateMachineFactory<>(HighLevelControllerName.class);
      factory.setNamePrefix("controller").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());

      factory.addState(HighLevelControllerName.DO_NOTHING_BEHAVIOR, doNothingState);
      factory.addState(HighLevelControllerName.STAND_PREP_STATE, standPrepState);
      factory.addState(HighLevelControllerName.STAND_READY, standReadyState);
      factory.addState(HighLevelControllerName.FREEZE_STATE, freezeState);
      factory.addState(HighLevelControllerName.WALKING, walkingState);
      factory.addState(HighLevelControllerName.STAND_TRANSITION_STATE, standTransitionState);
      factory.addState(HighLevelControllerName.EXIT_WALKING, exitWalkingState);

      // Add automatic transitions that lead into the stand state.
      factory.addDoneTransition(HighLevelControllerName.STAND_PREP_STATE, HighLevelControllerName.STAND_READY);

      // Manually triggered events to transition to main controllers.
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.WALKING, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STEPPING);
      factory.addTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR, HighLevelControllerName.WALKING, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STEPPING);
      factory.addTransition(HighLevelControllerName.FREEZE_STATE, HighLevelControllerName.WALKING, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STEPPING);
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.STAND_PREP_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP);
      factory.addTransition(HighLevelControllerName.FREEZE_STATE, HighLevelControllerName.STAND_PREP_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP);
      factory.addTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR, HighLevelControllerName.FREEZE_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_FREEZE);
      factory.addTransition(HighLevelControllerName.WALKING, HighLevelControllerName.FREEZE_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_FREEZE);
      factory.addTransition(HighLevelControllerName.STAND_PREP_STATE, HighLevelControllerName.FREEZE_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_FREEZE);
      factory.addTransition(HighLevelControllerName.STAND_READY, HighLevelControllerName.FREEZE_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_FREEZE);

      // Trigger do nothing
      for (HighLevelControllerName state : HighLevelControllerName.values)
      {
         factory.addTransition(state, HighLevelControllerName.DO_NOTHING_BEHAVIOR, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_DO_NOTHING);
      }

      // Fall triggered events
      factory.addTransition(HighLevelControllerName.WALKING, HighLevelControllerName.FREEZE_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_FALL);

      // Transitions from controllers back to stand prep.
      factory.addTransition(HighLevelControllerName.DO_NOTHING_BEHAVIOR, HighLevelControllerName.STAND_PREP_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP);
      factory.addTransition(HighLevelControllerName.WALKING, HighLevelControllerName.STAND_PREP_STATE, time -> requestedControllerState.getEnumValue() == QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP);

      factory.addStateChangedListener(new StateChangedListener<HighLevelControllerName>()
      {
         @Override
         public void stateChanged(HighLevelControllerName from, HighLevelControllerName to)
         {
            byte fromByte = from == null ? -1 : from.toByte();
            byte toByte = to == null ? -1 : to.toByte();
            stateChangeMessage.setInitialHighLevelControllerName(fromByte);
            stateChangeMessage.setEndHighLevelControllerName(toByte);
            statusMessageOutputManager.reportStatusMessage(stateChangeMessage);
         }
      });

      return factory.build(HighLevelControllerName.DO_NOTHING_BEHAVIOR);
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
}
