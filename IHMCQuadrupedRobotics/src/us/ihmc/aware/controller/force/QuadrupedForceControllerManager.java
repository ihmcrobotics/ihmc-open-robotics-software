package us.ihmc.aware.controller.force;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.aware.providers.QuadrupedControllerInputProvider;
import us.ihmc.aware.providers.QuadrupedTimedStepInputProvider;
import us.ihmc.aware.controller.QuadrupedController;
import us.ihmc.aware.controller.QuadrupedControllerManager;
import us.ihmc.aware.packets.QuadrupedForceControllerEventPacket;
import us.ihmc.aware.model.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.ParameterPacketListener;
import us.ihmc.aware.state.FiniteStateMachine;
import us.ihmc.aware.state.FiniteStateMachineBuilder;
import us.ihmc.aware.state.FiniteStateMachineYoVariableTrigger;
import us.ihmc.aware.planning.ContactState;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.aware.providers.QuadrupedControllerInputProviderInterface;
import us.ihmc.aware.model.QuadrupedRobotParameters;
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

   private final FiniteStateMachine<QuadrupedForceControllerState, QuadrupedForceControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedForceControllerEvent> userEventTrigger;
   private final QuadrupedRuntimeEnvironment runtimeEnvironment;
   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final AtomicReference<QuadrupedForceControllerEvent> requestedEvent = new AtomicReference<>();

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters parameters) throws IOException
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

      this.controllerToolbox = new QuadrupedForceControllerToolbox(runtimeEnvironment, parameters.getPhysicalProperties(), registry);
      this.stateMachine = buildStateMachine(runtimeEnvironment, parameters, inputProvider);
      this.userEventTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry, QuadrupedForceControllerEvent.class);
      this.runtimeEnvironment = runtimeEnvironment;
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      QuadrupedForceControllerEvent reqEvent = requestedEvent.getAndSet(null);
      if (reqEvent != null)
      {
         stateMachine.trigger(reqEvent);
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

   private FiniteStateMachine<QuadrupedForceControllerState, QuadrupedForceControllerEvent> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedRobotParameters parameters, QuadrupedControllerInputProviderInterface inputProvider)
   {
      // Initialize controllers.
      QuadrupedForceController jointInitializationController = new QuadrupedForceJointInitializationController(runtimeEnvironment);
      QuadrupedVirtualModelBasedStandPrepController standPrepController = new QuadrupedVirtualModelBasedStandPrepController(runtimeEnvironment, controllerToolbox);
      QuadrupedController standController = new QuadrupedVirtualModelBasedStandController(runtimeEnvironment, controllerToolbox, inputProvider);
      QuadrupedController stepController = new QuadrupedVirtualModelBasedStepController(runtimeEnvironment, controllerToolbox, inputProvider, stepProvider);
      QuadrupedForceController trotController = new QuadrupedVirtualModelBasedTrotController(runtimeEnvironment, controllerToolbox, inputProvider);
      QuadrupedForceController paceController = new QuadrupedVirtualModelBasedPaceController(runtimeEnvironment, controllerToolbox, inputProvider);

      FiniteStateMachineBuilder<QuadrupedForceControllerState, QuadrupedForceControllerEvent> builder = new FiniteStateMachineBuilder<>(QuadrupedForceControllerState.class,
            "forceControllerState", registry);

      builder.addState(QuadrupedForceControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedForceControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedForceControllerState.STAND, standController);
      builder.addState(QuadrupedForceControllerState.STEP, stepController);
      builder.addState(QuadrupedForceControllerState.TROT, trotController);
      builder.addState(QuadrupedForceControllerState.PACE, paceController);

      // Add automatic transitions that lead into the stand state.
      builder.addTransition(QuadrupedForceControllerEvent.JOINTS_INITIALIZED, QuadrupedForceControllerState.JOINT_INITIALIZATION,
            QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerEvent.STARTING_POSE_REACHED, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerEvent.FINAL_STEP_COMPLETED, QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.STAND);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND, QuadrupedForceControllerState.STEP, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND, QuadrupedForceControllerState.TROT, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND, QuadrupedForceControllerState.PACE, QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STEP, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STEP);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STEP, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STEP);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_TROT, QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.TROT);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_TROT, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.TROT);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_PACE, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.PACE);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND_PREP, QuadrupedForceControllerState.STAND, QuadrupedForceControllerState.STAND_PREP);

      return builder.build(QuadrupedForceControllerState.JOINT_INITIALIZATION);
   }
}
