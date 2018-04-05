package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;
import us.ihmc.quadrupedRobotics.communication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedControllerStateCommand;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.force.states.*;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.output.OutputProcessorBuilder;
import us.ihmc.quadrupedRobotics.output.StateChangeSmootherComponent;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EventTrigger;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedForceControllerManager implements QuadrupedControllerManager, CloseableAndDisposable
{
   private final CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEnum<QuadrupedForceControllerRequestedEvent> lastEvent = new YoEnum<>("lastEvent", registry, QuadrupedForceControllerRequestedEvent.class);

   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<QuadrupedForceControllerEnum, QuadrupedController> stateMachine;
   private EventTrigger trigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final QuadrupedControlManagerFactory controlManagerFactory;
   private final OutputProcessor outputProcessor;

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final QuadrupedControllerStateChangeMessage quadrupedControllerStateChangeMessage = new QuadrupedControllerStateChangeMessage();

   private final AtomicReference<QuadrupedForceControllerRequestedEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      this(runtimeEnvironment, physicalProperties, QuadrupedForceControllerEnum.JOINT_INITIALIZATION);
   }

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties,
                                          QuadrupedForceControllerEnum initialState) throws IOException
   {
      this.controllerToolbox = new QuadrupedForceControllerToolbox(runtimeEnvironment, physicalProperties, registry,
                                                                   runtimeEnvironment.getGraphicsListRegistry());
      this.runtimeEnvironment = runtimeEnvironment;

      // Initialize control modules
      this.controlManagerFactory = new QuadrupedControlManagerFactory(controllerToolbox, physicalProperties,
                                                                      runtimeEnvironment.getGraphicsListRegistry(), registry);

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
      controlManagerFactory.getOrCreateFeetManager().attachStateChangedListener(stateChangeSmootherComponent.createFiniteStateMachineStateChangedListener());
      OutputProcessorBuilder outputProcessorBuilder = new OutputProcessorBuilder(runtimeEnvironment.getFullRobotModel());
      outputProcessorBuilder.addComponent(stateChangeSmootherComponent);
      outputProcessor = outputProcessorBuilder.build();

      this.stateMachine = buildStateMachine(runtimeEnvironment, initialState);
   }

   /**
    * Hack for realtime controllers to run all states a lot of times. This hopefully kicks in the JIT compiler and avoids expensive interpreted code paths
    */
   public void warmup(int iterations)
   {
      YoDouble robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      double robotTimeBeforeWarmUp = robotTimestamp.getDoubleValue();
      for (QuadrupedForceControllerEnum state : QuadrupedForceControllerEnum.values)
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

   public QuadrupedController getState(QuadrupedForceControllerEnum state)
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

      // update fall detector
      if (controllerToolbox.getFallDetector().detect())
      {
         trigger.fireEvent(QuadrupedForceControllerRequestedEvent.REQUEST_FALL);
      }

      // update requested events
      QuadrupedForceControllerRequestedEvent reqEvent = requestedEvent.getAndSet(null);
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
            runtimeEnvironment.getFootSwitches().get(robotQuadrant).reset();

            if (controllerToolbox.getTaskSpaceController().getContactState(robotQuadrant) == ContactState.IN_CONTACT)
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

   @Override
   public RobotMotionStatusHolder getMotionStatusHolder()
   {
      return motionStatusHolder;
   }

   private StateMachine<QuadrupedForceControllerEnum, QuadrupedController> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
                                                                                             QuadrupedForceControllerEnum initialState)
   {
      // Initialize controllers.
      final QuadrupedController jointInitializationController = new QuadrupedForceBasedJointInitializationController(runtimeEnvironment);
      final QuadrupedController doNothingController = new QuadrupedForceBasedDoNothingController(controlManagerFactory.getOrCreateFeetManager(),
                                                                                                 runtimeEnvironment, registry);
      final QuadrupedController standPrepController = new QuadrupedForceBasedStandPrepController(controllerToolbox, controlManagerFactory, registry);
      final QuadrupedController freezeController = new QuadrupedForceBasedFreezeController(controllerToolbox, controlManagerFactory, registry);
      final QuadrupedSteppingState steppingController = new QuadrupedSteppingState(runtimeEnvironment, controllerToolbox, commandInputManager,
                                                                                   statusMessageOutputManager, controlManagerFactory, registry);
      final QuadrupedController fallController = new QuadrupedForceBasedFallController(controllerToolbox, controlManagerFactory, registry);

      EventBasedStateMachineFactory<QuadrupedForceControllerEnum, QuadrupedController> factory = new EventBasedStateMachineFactory<>(
            QuadrupedForceControllerEnum.class);
      factory.setNamePrefix("forceController").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());
      factory.buildYoEventTrigger("userTrigger", QuadrupedForceControllerRequestedEvent.class);
      trigger = factory.buildEventTrigger();

      factory.addState(QuadrupedForceControllerEnum.JOINT_INITIALIZATION, jointInitializationController);
      factory.addState(QuadrupedForceControllerEnum.DO_NOTHING, doNothingController);
      factory.addState(QuadrupedForceControllerEnum.STAND_PREP, standPrepController);
      factory.addState(QuadrupedForceControllerEnum.STAND_READY, freezeController);
      factory.addState(QuadrupedForceControllerEnum.FREEZE, freezeController);
      factory.addState(QuadrupedForceControllerEnum.STEPPING, steppingController);
      factory.addState(QuadrupedForceControllerEnum.FALL, fallController);

      // Add automatic transitions that lead into the stand state.
      factory.addTransition(ControllerEvent.DONE, QuadrupedForceControllerEnum.JOINT_INITIALIZATION, QuadrupedForceControllerEnum.DO_NOTHING);
      factory.addTransition(ControllerEvent.DONE, QuadrupedForceControllerEnum.STAND_PREP, QuadrupedForceControllerEnum.STAND_READY);
      factory.addTransition(ControllerEvent.FAIL, QuadrupedForceControllerEnum.STEPPING, QuadrupedForceControllerEnum.FREEZE);

      // Manually triggered events to transition to main controllers.
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING, QuadrupedForceControllerEnum.STAND_READY,
                            QuadrupedForceControllerEnum.STEPPING);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING, QuadrupedForceControllerEnum.DO_NOTHING,
                            QuadrupedForceControllerEnum.STEPPING);
      factory
            .addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STEPPING, QuadrupedForceControllerEnum.FREEZE, QuadrupedForceControllerEnum.STEPPING);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedForceControllerEnum.STAND_READY,
                            QuadrupedForceControllerEnum.STAND_PREP);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedForceControllerEnum.FREEZE,
                            QuadrupedForceControllerEnum.STAND_PREP);
      factory
            .addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE, QuadrupedForceControllerEnum.DO_NOTHING, QuadrupedForceControllerEnum.FREEZE);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE, QuadrupedForceControllerEnum.STEPPING, QuadrupedForceControllerEnum.FREEZE);
      factory
            .addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE, QuadrupedForceControllerEnum.STAND_PREP, QuadrupedForceControllerEnum.FREEZE);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE, QuadrupedForceControllerEnum.STAND_READY,
                            QuadrupedForceControllerEnum.FREEZE);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_DO_NOTHING, QuadrupedForceControllerEnum.STEPPING,
                            QuadrupedForceControllerEnum.DO_NOTHING);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_DO_NOTHING, QuadrupedForceControllerEnum.FREEZE,
                            QuadrupedForceControllerEnum.DO_NOTHING);

      // Trigger do nothing
      for (QuadrupedForceControllerEnum state : QuadrupedForceControllerEnum.values)
      {
         factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_DO_NOTHING, state, QuadrupedForceControllerEnum.DO_NOTHING);
      }

      // Fall triggered events
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_FALL, QuadrupedForceControllerEnum.STEPPING, QuadrupedForceControllerEnum.FALL);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_FALL, QuadrupedForceControllerEnum.FREEZE, QuadrupedForceControllerEnum.FALL);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedForceControllerEnum.FALL,
                            QuadrupedForceControllerEnum.STAND_PREP);
      factory.addTransition(ControllerEvent.DONE, QuadrupedForceControllerEnum.FALL, QuadrupedForceControllerEnum.FREEZE);

      // Transitions from controllers back to stand prep.
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedForceControllerEnum.DO_NOTHING,
                            QuadrupedForceControllerEnum.STAND_PREP);
      factory.addTransition(QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedForceControllerEnum.STEPPING,
                            QuadrupedForceControllerEnum.STAND_PREP);

      factory.addStateChangedListener(new StateChangedListener<QuadrupedForceControllerEnum>()
      {
         @Override
         public void stateChanged(QuadrupedForceControllerEnum from, QuadrupedForceControllerEnum to)
         {
            byte fromByte = from == null ? -1 : from.toByte();
            byte toByte = to == null ? -1 : to.toByte();
            quadrupedControllerStateChangeMessage.setInitialControllerName(fromByte);
            quadrupedControllerStateChangeMessage.setEndControllerName(toByte);
            statusMessageOutputManager.reportStatusMessage(quadrupedControllerStateChangeMessage);
         }
      });

      return factory.build(initialState);
   }

   public void createControllerNetworkSubscriber(PeriodicThreadScheduler scheduler, PacketCommunicator packetCommunicator)
   {
      ControllerNetworkSubscriber controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusMessageOutputManager, scheduler,
                                                                                                packetCommunicator);
      //      controllerNetworkSubscriber.registerSubcriberWithMessageUnpacker(WholeBodyTrajectoryMessage.class, 9, MessageUnpackingTools.createWholeBodyTrajectoryMessageUnpacker());
      controllerNetworkSubscriber.addMessageCollector(QuadrupedControllerAPIDefinition.createDefaultMessageIDExtractor());
      controllerNetworkSubscriber.addMessageValidator(QuadrupedControllerAPIDefinition.createDefaultMessageValidation());
      closeableAndDisposableRegistry.registerCloseableAndDisposable(controllerNetworkSubscriber);
   }

   @Override
   public void closeAndDispose()
   {
      closeableAndDisposableRegistry.closeAndDispose();
   }
}
