package us.ihmc.quadrupedRobotics.controller;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedControllerStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedDoNothingController;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedFreezeController;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedJointInitializationController;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedPositionBasedCrawlController;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedStandPrepController;
import us.ihmc.quadrupedRobotics.controller.states.QuadrupedSteppingState;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.output.JointIntegratorComponent;
import us.ihmc.quadrupedRobotics.output.OutputProcessorBuilder;
import us.ihmc.quadrupedRobotics.output.StateChangeSmootherComponent;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

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

   private final StateMachine<QuadrupedControllerEnum, QuadrupedController> stateMachine;
   private EventTrigger trigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;
   private final OutputProcessor outputProcessor;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final QuadrupedControllerStateChangeMessage quadrupedControllerStateChangeMessage = new QuadrupedControllerStateChangeMessage();
   private final WalkingControllerFailureStatusMessage walkingControllerFailureStatusMessage = new WalkingControllerFailureStatusMessage();

   private final AtomicReference<QuadrupedControllerRequestedEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                     QuadrupedInitialPositionParameters initialPositionParameters)
   {
      this(runtimeEnvironment, physicalProperties, initialPositionParameters, QuadrupedControllerEnum.JOINT_INITIALIZATION);
   }

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                     QuadrupedInitialPositionParameters initialPositionParameters, QuadrupedControllerEnum initialState)
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

      // Initialize output processor
      StateChangeSmootherComponent stateChangeSmootherComponent = new StateChangeSmootherComponent(runtimeEnvironment, registry);
      JointIntegratorComponent jointControlComponent = new JointIntegratorComponent(runtimeEnvironment, registry);
      controlManagerFactory.getOrCreateFeetManager().attachStateChangedListener(stateChangeSmootherComponent.createFiniteStateMachineStateChangedListener());
      OutputProcessorBuilder outputProcessorBuilder = new OutputProcessorBuilder(runtimeEnvironment.getFullRobotModel());
      outputProcessorBuilder.addComponent(stateChangeSmootherComponent);
      outputProcessorBuilder.addComponent(jointControlComponent);
      outputProcessor = outputProcessorBuilder.build();

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

      this.stateMachine = buildStateMachine(runtimeEnvironment, initialState, initialPositionParameters);
   }

   /**
    * Hack for realtime controllers to run all states a lot of times. This hopefully kicks in the
    * JIT compiler and avoids expensive interpreted code paths
    */
   public void warmup(int iterations)
   {
      YoDouble robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      double robotTimeBeforeWarmUp = robotTimestamp.getDoubleValue();
      for (QuadrupedControllerEnum state : QuadrupedControllerEnum.values)
      {
         QuadrupedController stateImpl = stateMachine.getState(state);

         stateImpl.onEntry();
         for (int i = 0; i < iterations; i++)
         {
            robotTimestamp.add(Conversions.millisecondsToSeconds(1));
            stateImpl.doAction(Double.NaN);
         }
         stateImpl.onExit();

         for (int i = 0; i < iterations; i++)
         {
            robotTimestamp.add(Conversions.millisecondsToSeconds(1));
            stateImpl.onEntry();
            stateImpl.onExit();
         }
      }
      robotTimestamp.set(robotTimeBeforeWarmUp);
   }

   public QuadrupedController getState(QuadrupedControllerEnum state)
   {
      return stateMachine.getState(state);
   }

   @Override
   public void initialize()
   {
      outputProcessor.initialize();
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
         trigger.fireEvent(reqEvent);
      }

      // update controller state machine
      stateMachine.doActionAndTransition();

      // update contact state used for state estimation
      switch (stateMachine.getCurrentStateKey())
      {
      case DO_NOTHING:
      case STAND_PREP:
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
         trigger.fireEvent(QuadrupedControllerRequestedEvent.REQUEST_FALL);
         walkingControllerFailureStatusMessage.falling_direction_.set(runtimeEnvironment.getFullRobotModel().getRootJoint().getLinearVelocityForReading());
         statusMessageOutputManager.reportStatusMessage(walkingControllerFailureStatusMessage);
      }

      // update output processor
      outputProcessor.update();
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

   private StateMachine<QuadrupedControllerEnum, QuadrupedController> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
                                                                                        QuadrupedControllerEnum initialState,
                                                                                        QuadrupedInitialPositionParameters initialPositionParameters)
   {
      // Initialize controllers.
      final QuadrupedController jointInitializationController = new QuadrupedJointInitializationController(controllerToolbox, registry);
      final QuadrupedController doNothingController = new QuadrupedDoNothingController(controllerToolbox, registry);
      final QuadrupedController standPrepController = new QuadrupedStandPrepController(controllerToolbox, initialPositionParameters, registry);
      final QuadrupedController freezeController = new QuadrupedFreezeController(controllerToolbox, controlManagerFactory, registry);

      final QuadrupedController steppingController = new QuadrupedSteppingState(runtimeEnvironment, controllerToolbox, commandInputManager,
                                                                                statusMessageOutputManager, controlManagerFactory, registry);

      EventBasedStateMachineFactory<QuadrupedControllerEnum, QuadrupedController> factory = new EventBasedStateMachineFactory<>(QuadrupedControllerEnum.class);
      factory.setNamePrefix("controller").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());
      factory.buildYoEventTrigger("userTrigger", QuadrupedControllerRequestedEvent.class);
      trigger = factory.buildEventTrigger();

      factory.addState(QuadrupedControllerEnum.JOINT_INITIALIZATION, jointInitializationController);
      factory.addState(QuadrupedControllerEnum.DO_NOTHING, doNothingController);
      factory.addState(QuadrupedControllerEnum.STAND_PREP, standPrepController);
      factory.addState(QuadrupedControllerEnum.STAND_READY, freezeController);
      factory.addState(QuadrupedControllerEnum.FREEZE, freezeController);
      factory.addState(QuadrupedControllerEnum.STEPPING, steppingController);
      factory.addState(QuadrupedControllerEnum.FALL, freezeController);

      // Add automatic transitions that lead into the stand state.
      factory.addTransition(ControllerEvent.DONE, QuadrupedControllerEnum.JOINT_INITIALIZATION, QuadrupedControllerEnum.DO_NOTHING);
      factory.addTransition(ControllerEvent.DONE, QuadrupedControllerEnum.STAND_PREP, QuadrupedControllerEnum.STAND_READY);
      factory.addTransition(ControllerEvent.FAIL, QuadrupedControllerEnum.STEPPING, QuadrupedControllerEnum.FREEZE);

      // Manually triggered events to transition to main controllers.
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STEPPING, QuadrupedControllerEnum.STAND_READY, QuadrupedControllerEnum.STEPPING);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STEPPING, QuadrupedControllerEnum.DO_NOTHING, QuadrupedControllerEnum.STEPPING);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STEPPING, QuadrupedControllerEnum.FREEZE, QuadrupedControllerEnum.STEPPING);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedControllerEnum.STAND_READY, QuadrupedControllerEnum.STAND_PREP);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedControllerEnum.FREEZE, QuadrupedControllerEnum.STAND_PREP);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_FREEZE, QuadrupedControllerEnum.DO_NOTHING, QuadrupedControllerEnum.FREEZE);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_FREEZE, QuadrupedControllerEnum.STEPPING, QuadrupedControllerEnum.FREEZE);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_FREEZE, QuadrupedControllerEnum.STAND_PREP, QuadrupedControllerEnum.FREEZE);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_FREEZE, QuadrupedControllerEnum.STAND_READY, QuadrupedControllerEnum.FREEZE);

      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_DO_NOTHING, QuadrupedControllerEnum.STEPPING, QuadrupedControllerEnum.DO_NOTHING);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_DO_NOTHING, QuadrupedControllerEnum.FREEZE, QuadrupedControllerEnum.DO_NOTHING);

      // Trigger do nothing
      for (QuadrupedControllerEnum state : QuadrupedControllerEnum.values)
      {
         factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_DO_NOTHING, state, QuadrupedControllerEnum.DO_NOTHING);
      }

      // Fall triggered events
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_FALL, QuadrupedControllerEnum.STEPPING, QuadrupedControllerEnum.FALL);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_FALL, QuadrupedControllerEnum.FREEZE, QuadrupedControllerEnum.FALL);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedControllerEnum.FALL, QuadrupedControllerEnum.STAND_PREP);
      factory.addTransition(ControllerEvent.DONE, QuadrupedControllerEnum.FALL, QuadrupedControllerEnum.FREEZE);

      // Transitions from controllers back to stand prep.
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedControllerEnum.DO_NOTHING, QuadrupedControllerEnum.STAND_PREP);
      factory.addTransition(QuadrupedControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedControllerEnum.STEPPING, QuadrupedControllerEnum.STAND_PREP);

      factory.addStateChangedListener(new StateChangedListener<QuadrupedControllerEnum>()
      {
         @Override
         public void stateChanged(QuadrupedControllerEnum from, QuadrupedControllerEnum to)
         {
            byte fromByte = from == null ? -1 : from.toByte();
            byte toByte = to == null ? -1 : to.toByte();
            quadrupedControllerStateChangeMessage.setInitialQuadrupedControllerEnum(fromByte);
            quadrupedControllerStateChangeMessage.setEndQuadrupedControllerEnum(toByte);
            statusMessageOutputManager.reportStatusMessage(quadrupedControllerStateChangeMessage);
         }
      });

      return factory.build(initialState);
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
      QuadrupedSteppingState steppingState = (QuadrupedSteppingState) stateMachine.getState(QuadrupedControllerEnum.STEPPING);
      steppingState.onEntry();
   }

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
   }
}
