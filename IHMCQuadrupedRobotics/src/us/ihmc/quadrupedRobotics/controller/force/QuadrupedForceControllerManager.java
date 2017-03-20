package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.Conversions;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerStatePacket;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedDcmBasedStandController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedDcmBasedStepController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedDoNothingController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedFallController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedFreezeController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedJointInitializationController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedSoleWaypointController;
import us.ihmc.quadrupedRobotics.controller.force.states.QuadrupedForceBasedStandPrepController;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.output.OutputProcessorBuilder;
import us.ihmc.quadrupedRobotics.output.StateChangeSmootherComponent;
import us.ihmc.communication.subscribers.ParameterPacketListener;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedPreplannedStepStream;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedStepStreamMultiplexer;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.quadrupedRobotics.providers.QuadrupedSoleWaypointInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPreplannedStepInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsInputProvider;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineState;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineYoVariableTrigger;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedForceControllerManager implements QuadrupedControllerManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<QuadrupedForceControllerRequestedEvent> lastEvent = new EnumYoVariable<>("lastEvent", registry, QuadrupedForceControllerRequestedEvent.class);

   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();
   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;
   private final QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider;
   private final QuadrupedPreplannedStepInputProvider preplannedStepProvider;
   private final QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;

   private final QuadrupedForceControllerStatePacket quadrupedForceControllerStatePacket;

   private final QuadrupedPreplannedStepStream preplannedStepStream;
   private final QuadrupedXGaitStepStream xGaitStepStream;
   private final QuadrupedStepStreamMultiplexer<QuadrupedForceControllerState> stepStreamMultiplexer;

   private final FiniteStateMachine<QuadrupedForceControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedForceControllerRequestedEvent> userEventTrigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final OutputProcessor outputProcessor;

   private final AtomicReference<QuadrupedForceControllerRequestedEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      this.controllerToolbox = new QuadrupedForceControllerToolbox(runtimeEnvironment, physicalProperties, registry);
      this.runtimeEnvironment = runtimeEnvironment;

      // Initialize input providers.
      postureProvider = new QuadrupedPostureInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      planarVelocityProvider = new QuadrupedPlanarVelocityInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      xGaitSettingsProvider = new QuadrupedXGaitSettingsInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      preplannedStepProvider = new QuadrupedPreplannedStepInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      soleWaypointInputProvider = new QuadrupedSoleWaypointInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);

      // Initialize input step streams.
      xGaitStepStream = new QuadrupedXGaitStepStream(planarVelocityProvider, xGaitSettingsProvider, controllerToolbox.getReferenceFrames(),
            runtimeEnvironment.getControlDT(), runtimeEnvironment.getRobotTimestamp(), registry);
      preplannedStepStream = new QuadrupedPreplannedStepStream(preplannedStepProvider, controllerToolbox.getReferenceFrames(),
            runtimeEnvironment.getRobotTimestamp(), registry);
      stepStreamMultiplexer = new QuadrupedStepStreamMultiplexer<>(QuadrupedForceControllerState.class, registry);
      stepStreamMultiplexer.addStepStream(QuadrupedForceControllerState.XGAIT, xGaitStepStream);
      stepStreamMultiplexer.addStepStream(QuadrupedForceControllerState.STEP, preplannedStepStream);
      stepStreamMultiplexer.selectStepStream(QuadrupedForceControllerState.XGAIT);

      // Initialize output processor
      StateChangeSmootherComponent stateChangeSmootherComponent = new StateChangeSmootherComponent(runtimeEnvironment.getControlDT(),
            runtimeEnvironment.getRobotTimestamp(), registry);
      FiniteStateMachineStateChangedListener stateChangedListener = stateChangeSmootherComponent.createFiniteStateMachineStateChangedListener();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         controllerToolbox.getFootStateMachine().get(robotQuadrant).attachStateChangedListener(stateChangedListener);
      OutputProcessorBuilder outputProcessorBuilder = new OutputProcessorBuilder(runtimeEnvironment.getFullRobotModel());
      outputProcessorBuilder.addComponent(stateChangeSmootherComponent);
      outputProcessor = outputProcessorBuilder.build();

      GlobalDataProducer globalDataProducer = runtimeEnvironment.getGlobalDataProducer();

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedForceControllerEventPacket.class, new PacketConsumer<QuadrupedForceControllerEventPacket>()
         {
            @Override
            public void receivedPacket(QuadrupedForceControllerEventPacket packet)
            {
               requestedEvent.set(packet.get());
            }
         });

         ParameterPacketListener parameterPacketListener = new ParameterPacketListener(globalDataProducer);
      }
      this.quadrupedForceControllerStatePacket = new QuadrupedForceControllerStatePacket();

      this.stateMachine = buildStateMachine(runtimeEnvironment, postureProvider);
      this.userEventTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry, QuadrupedForceControllerRequestedEvent.class);
   }

   /**
    * Hack for realtime controllers to run all states a lot of times. This hopefully kicks in the JIT compiler and avoids expensive interpreted code paths
    */
   public void warmup(int iterations)
   {
      DoubleYoVariable robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      double robotTimeBeforeWarmUp = robotTimestamp.getDoubleValue();
      for (QuadrupedForceControllerState state : QuadrupedForceControllerState.values)
      {
         FiniteStateMachineState<ControllerEvent> stateImpl = stateMachine.getState(state);

         stateImpl.onEntry();
         for (int i = 0; i < iterations; i++)
         {
            robotTimestamp.add(Conversions.millisecondsToSeconds(1));
            stateImpl.process();
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

   public FiniteStateMachineState<ControllerEvent> getState(QuadrupedForceControllerState state)
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
      // update fall detector
      if (controllerToolbox.getFallDetector().detect())
      {
         stateMachine.trigger(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL);
      }

      // update requested events
      QuadrupedForceControllerRequestedEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         lastEvent.set(reqEvent);
         stateMachine.trigger(QuadrupedForceControllerRequestedEvent.class, reqEvent);
      }
      if (preplannedStepProvider.isStepPlanAvailable())
      {
         if (stateMachine.getState() == QuadrupedForceControllerState.STAND)
         {
            // trigger step event if preplanned steps are available in stand state
            lastEvent.set(QuadrupedForceControllerRequestedEvent.REQUEST_STEP);
            stateMachine.trigger(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STEP);
         }
      }

      // update controller state machine
      stateMachine.process();

      // update contact state used for state estimation
      switch (stateMachine.getState())
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

      // Send state information
      quadrupedForceControllerStatePacket.set(stateMachine.getState());
      
      if (runtimeEnvironment.getGlobalDataProducer() != null)
      {
         runtimeEnvironment.getGlobalDataProducer().queueDataToSend(quadrupedForceControllerStatePacket);
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

   @Override
   public RobotMotionStatusHolder getMotionStatusHolder()
   {
      return motionStatusHolder;
   }

   private FiniteStateMachine<QuadrupedForceControllerState, ControllerEvent> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedPostureInputProviderInterface inputProvider)
   {
      // Initialize controllers.
      final QuadrupedController jointInitializationController = new QuadrupedForceBasedJointInitializationController(runtimeEnvironment);
      final QuadrupedController doNothingController = new QuadrupedForceBasedDoNothingController(runtimeEnvironment, registry);
      final QuadrupedController standPrepController = new QuadrupedForceBasedStandPrepController(runtimeEnvironment, controllerToolbox);
      final QuadrupedController freezeController = new QuadrupedForceBasedFreezeController(runtimeEnvironment, controllerToolbox);
      final QuadrupedController standController = new QuadrupedDcmBasedStandController(runtimeEnvironment, controllerToolbox, inputProvider);
      final QuadrupedDcmBasedStepController stepController = new QuadrupedDcmBasedStepController(runtimeEnvironment, controllerToolbox, inputProvider,
            stepStreamMultiplexer);
      final QuadrupedController fallController = new QuadrupedForceBasedFallController(runtimeEnvironment, controllerToolbox);
      final QuadrupedController soleWaypointController = new QuadrupedForceBasedSoleWaypointController(runtimeEnvironment, controllerToolbox,
            soleWaypointInputProvider);

      FiniteStateMachineBuilder<QuadrupedForceControllerState, ControllerEvent> builder = new FiniteStateMachineBuilder<>(QuadrupedForceControllerState.class,
            ControllerEvent.class, "forceControllerState", registry);

      builder.addState(QuadrupedForceControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedForceControllerState.DO_NOTHING, doNothingController);
      builder.addState(QuadrupedForceControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedForceControllerState.STAND_READY, freezeController);
      builder.addState(QuadrupedForceControllerState.FREEZE, freezeController);
      builder.addState(QuadrupedForceControllerState.STAND, standController);
      builder.addState(QuadrupedForceControllerState.STEP, stepController);
      builder.addState(QuadrupedForceControllerState.XGAIT, stepController);
      builder.addState(QuadrupedForceControllerState.FALL, fallController);
      builder.addState(QuadrupedForceControllerState.SOLE_WAYPOINT, soleWaypointController);

      // Add automatic transitions that lead into the stand state.
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.JOINT_INITIALIZATION, QuadrupedForceControllerState.DO_NOTHING);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND_READY);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.STAND);
      builder.addTransition(ControllerEvent.FAIL, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.XGAIT, QuadrupedForceControllerState.STAND);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.STAND_READY, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.DO_NOTHING, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STEP,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STEP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.XGAIT);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.STAND_READY, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE,
         QuadrupedForceControllerState.DO_NOTHING, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE,
            QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE,
            QuadrupedForceControllerState.STAND_READY, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FREEZE,
            QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_DO_NOTHING,
            QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.DO_NOTHING);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_DO_NOTHING,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.DO_NOTHING);

      // Trigger do nothing
      for(QuadrupedForceControllerState state: QuadrupedForceControllerState.values()){
         builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_DO_NOTHING,
               state, QuadrupedForceControllerState.DO_NOTHING);
      }

      // Fall triggered events
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.FALL);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.FALL);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.XGAIT, QuadrupedForceControllerState.FALL);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.FALL);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.FALL, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.FALL, QuadrupedForceControllerState.FREEZE);

      // Sole Waypoint events
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_SOLE_WAYPOINT,
            QuadrupedForceControllerState.STAND_READY, QuadrupedForceControllerState.SOLE_WAYPOINT);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_SOLE_WAYPOINT,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.SOLE_WAYPOINT);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_SOLE_WAYPOINT,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.SOLE_WAYPOINT);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_SOLE_WAYPOINT,
            QuadrupedForceControllerState.DO_NOTHING, QuadrupedForceControllerState.SOLE_WAYPOINT);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(ControllerEvent.FAIL, QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.FREEZE);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.DO_NOTHING, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STAND_PREP);

      // Callbacks functions.
      Runnable standToXGaitCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepStreamMultiplexer.selectStepStream(QuadrupedForceControllerState.XGAIT);
         }
      };
      builder.addCallback(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT,
            QuadrupedForceControllerState.STAND, standToXGaitCallback);

      Runnable xGaitToStandCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepController.halt();
         }
      };
      builder.addCallback(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.XGAIT, xGaitToStandCallback);

      Runnable standToStepCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepStreamMultiplexer.selectStepStream(QuadrupedForceControllerState.STEP);
         }
      };
      builder
            .addCallback(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STEP, QuadrupedForceControllerState.STAND,
                  standToStepCallback);

      Runnable stepToStandCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepController.halt();
         }
      };
      builder
            .addCallback(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND, QuadrupedForceControllerState.STEP,
                  stepToStandCallback);

      return builder.build(QuadrupedForceControllerState.JOINT_INITIALIZATION);
   }
}
