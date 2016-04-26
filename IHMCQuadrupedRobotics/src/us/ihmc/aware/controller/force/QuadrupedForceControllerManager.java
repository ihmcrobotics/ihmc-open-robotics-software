package us.ihmc.aware.controller.force;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.aware.controller.ControllerEvent;
import us.ihmc.aware.controller.QuadrupedController;
import us.ihmc.aware.controller.QuadrupedControllerManager;
import us.ihmc.aware.controller.force.states.*;
import us.ihmc.aware.model.QuadrupedPhysicalProperties;
import us.ihmc.aware.model.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.aware.params.ParameterPacketListener;
import us.ihmc.aware.planning.ContactState;
import us.ihmc.aware.providers.QuadrupedControllerInputProvider;
import us.ihmc.aware.providers.QuadrupedControllerInputProviderInterface;
import us.ihmc.aware.providers.QuadrupedTimedStepInputProvider;
import us.ihmc.aware.state.FiniteStateMachine;
import us.ihmc.aware.state.FiniteStateMachineBuilder;
import us.ihmc.aware.state.FiniteStateMachineYoVariableTrigger;
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
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();
   private final QuadrupedControllerInputProviderInterface inputProvider;
   private final QuadrupedTimedStepInputProvider stepProvider;

   private final FiniteStateMachine<QuadrupedForceControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedForceControllerRequestedEvent> userEventTrigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final AtomicReference<QuadrupedForceControllerRequestedEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedPhysicalProperties physicalProperties) throws IOException
   {
      // Initialize input providers.
      inputProvider = new QuadrupedControllerInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      stepProvider = new QuadrupedTimedStepInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);

      GlobalDataProducer globalDataProducer = runtimeEnvironment.getGlobalDataProducer();
      globalDataProducer.attachListener(QuadrupedForceControllerEventPacket.class, new PacketConsumer<QuadrupedForceControllerEventPacket>()
      {
         @Override
         public void receivedPacket(QuadrupedForceControllerEventPacket packet)
         {
            requestedEvent.set(packet.get());
         }
      });

      ParameterPacketListener parameterPacketListener = new ParameterPacketListener(globalDataProducer);

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
      QuadrupedForceControllerRequestedEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         stateMachine.trigger(QuadrupedForceControllerRequestedEvent.class, reqEvent);
      }

      stateMachine.process();

      // update contact state used for state estimation
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

   private FiniteStateMachine<QuadrupedForceControllerState, ControllerEvent> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedControllerInputProviderInterface inputProvider)
   {
      // Initialize controllers.
      QuadrupedController jointInitializationController = new QuadrupedForceBasedJointInitializationController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedForceBasedStandPrepController(runtimeEnvironment, controllerToolbox);
      QuadrupedController standController = new QuadrupedDcmBasedStandController(runtimeEnvironment, controllerToolbox, inputProvider);
      QuadrupedController stepController = new QuadrupedDcmBasedStepController(runtimeEnvironment, controllerToolbox, inputProvider, stepProvider);
      QuadrupedController trotController = new QuadrupedDcmBasedTrotController(runtimeEnvironment, controllerToolbox, inputProvider);
      QuadrupedController paceController = new QuadrupedDcmBasedPaceController(runtimeEnvironment, controllerToolbox, inputProvider);

      FiniteStateMachineBuilder<QuadrupedForceControllerState, ControllerEvent> builder = new FiniteStateMachineBuilder<>(
            QuadrupedForceControllerState.class, ControllerEvent.class, "forceControllerState", registry);

      builder.addState(QuadrupedForceControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedForceControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedForceControllerState.STAND, standController);
      builder.addState(QuadrupedForceControllerState.STEP, stepController);
      builder.addState(QuadrupedForceControllerState.TROT, trotController);
      builder.addState(QuadrupedForceControllerState.PACE, paceController);

      // Add automatic transitions that lead into the stand state.
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.JOINT_INITIALIZATION, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND);
      builder.addTransition(ControllerEvent.DONE, QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.STAND);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND, QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND, QuadrupedForceControllerState.TROT, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND, QuadrupedForceControllerState.PACE, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STEP, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STEP);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_TROT, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.TROT);
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_PACE, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.PACE);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedForceControllerRequestedEvent.class, QuadrupedForceControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STAND_PREP);

      return builder.build(QuadrupedForceControllerState.JOINT_INITIALIZATION);
   }
}
