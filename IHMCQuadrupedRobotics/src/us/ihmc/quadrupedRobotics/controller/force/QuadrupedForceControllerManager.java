package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.quadrupedRobotics.communication.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.force.states.*;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.BooleanParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.params.ParameterPacketListener;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.providers.*;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachine;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineBuilder;
import us.ihmc.quadrupedRobotics.state.FiniteStateMachineYoVariableTrigger;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedForceControllerManager implements QuadrupedControllerManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final BooleanParameter bypassDoNothingStateParameter = parameterFactory.createBoolean("bypassDoNothingState", true);

   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final QuadrupedTimedStepInputProvider timedStepProvider;
   private final QuadrupedXGaitSettingsProvider xGaitSettingsProvider;
   private final QuadrupedSoleWaypointInputProvider soleWaypointInputProvider;

   private final FiniteStateMachine<QuadrupedForceControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedForceControllerRequestedEvent> userEventTrigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final AtomicReference<QuadrupedForceControllerRequestedEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      // Initialize input providers.
      inputProvider = new QuadrupedControllerInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      timedStepProvider = new QuadrupedTimedStepInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      xGaitSettingsProvider = new QuadrupedXGaitSettingsProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      soleWaypointInputProvider = new QuadrupedSoleWaypointInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);

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

      this.controllerToolbox = new QuadrupedForceControllerToolbox(runtimeEnvironment, physicalProperties, registry);
      this.stateMachine = buildStateMachine(runtimeEnvironment, inputProvider);
      this.userEventTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry, QuadrupedForceControllerRequestedEvent.class);
      this.runtimeEnvironment = runtimeEnvironment;
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      // update controller state machine
      QuadrupedForceControllerRequestedEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         stateMachine.trigger(QuadrupedForceControllerRequestedEvent.class, reqEvent);
      }
      stateMachine.process();

      // update contact state used for state estimation
      switch (stateMachine.getState())
      {
      case DO_NOTHING:
      case STAND_PREP:
      case STAND_READY:
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.FRONT_RIGHT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_LEFT).setFootContactState(false);
         runtimeEnvironment.getFootSwitches().get(RobotQuadrant.HIND_RIGHT).setFootContactState(true);
         break;
      default:
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
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
         QuadrupedControllerInputProviderInterface inputProvider)
   {
      // Initialize controllers.
      final QuadrupedController jointInitializationController = new QuadrupedForceBasedJointInitializationController(runtimeEnvironment);
      final QuadrupedController doNothingController = new QuadrupedForceBasedDoNothingController(runtimeEnvironment, registry);
      final QuadrupedController standPrepController = new QuadrupedForceBasedStandPrepController(runtimeEnvironment, controllerToolbox);
      final QuadrupedController freezeController = new QuadrupedForceBasedFreezeController(runtimeEnvironment, controllerToolbox);
      final QuadrupedController standController = new QuadrupedDcmBasedStandController(runtimeEnvironment, controllerToolbox, inputProvider);
      final QuadrupedDcmBasedStepController stepController = new QuadrupedDcmBasedStepController(runtimeEnvironment, controllerToolbox, inputProvider,
            timedStepProvider);
      final QuadrupedMpcBasedXGaitController xGaitController = new QuadrupedMpcBasedXGaitController(runtimeEnvironment, controllerToolbox, inputProvider,
            xGaitSettingsProvider);
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
      builder.addState(QuadrupedForceControllerState.XGAIT, xGaitController);
      builder.addState(QuadrupedForceControllerState.FALL, fallController);
      builder.addState(QuadrupedForceControllerState.SOLE_WAYPOINT, soleWaypointController);

      // Add automatic transitions that lead into the stand state.
      if (bypassDoNothingStateParameter.get())
      {
         builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.JOINT_INITIALIZATION, QuadrupedForceControllerState.STAND_PREP);
      }
      else
      {
         builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.JOINT_INITIALIZATION, QuadrupedForceControllerState.DO_NOTHING);
      }
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND_READY);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.STAND);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.XGAIT, QuadrupedForceControllerState.STAND);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.STAND_READY, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STEP,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STEP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.XGAIT);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.STAND_READY, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.FREEZE, QuadrupedForceControllerState.STAND_PREP);

      // Fall triggered events
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.FALL);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.FALL);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_FALL,
            QuadrupedForceControllerState.XGAIT, QuadrupedForceControllerState.FALL);
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
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.FREEZE);
      builder.addTransition(ControllerEvent.FAIL, QuadrupedForceControllerState.SOLE_WAYPOINT, QuadrupedForceControllerState.FREEZE);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.DO_NOTHING, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STAND_PREP);

      // Callbacks functions.
      Runnable xGaitHaltCallback = new Runnable()
      {
         @Override
         public void run()
         {
            xGaitController.halt();
         }
      };
      builder.addCallback(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND,
            QuadrupedForceControllerState.XGAIT, xGaitHaltCallback);
      Runnable stepHaltCallback = new Runnable()
      {
         @Override
         public void run()
         {
            stepController.halt();
         }
      };
      builder
            .addCallback(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND, QuadrupedForceControllerState.STEP,
                  stepHaltCallback);

      return builder.build(QuadrupedForceControllerState.JOINT_INITIALIZATION);
   }
}
