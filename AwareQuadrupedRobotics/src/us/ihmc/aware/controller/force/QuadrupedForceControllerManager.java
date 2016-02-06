package us.ihmc.aware.controller.force;

import us.ihmc.aware.controller.QuadrupedController;
import us.ihmc.aware.controller.QuadrupedControllerManager;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineYoVariableTrigger;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal finite state
 * machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedForceControllerManager implements QuadrupedControllerManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<QuadrupedForceControllerState, QuadrupedForceControllerEvent> stateMachine;
   private final StateMachineYoVariableTrigger<QuadrupedForceControllerEvent> userEventTrigger;

   public QuadrupedForceControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedRobotParameters parameters)
   {
      ParameterMapRepository paramMapRepository = new ParameterMapRepository(registry);

      QuadrupedForceController jointInitializationController = new QuadrupedForceJointInitializationController(
            runtimeEnvironment, parameters);
      QuadrupedVirtualModelBasedStandPrepController standPrepController = new QuadrupedVirtualModelBasedStandPrepController(
            runtimeEnvironment, parameters, paramMapRepository);
      QuadrupedController standController = new QuadrupedVirtualModelBasedStandController(runtimeEnvironment,
            parameters, paramMapRepository);
      QuadrupedController stepController = new QuadrupedVirtualModelBasedStepController(runtimeEnvironment, parameters,
            paramMapRepository);

      StateMachineBuilder<QuadrupedForceControllerState, QuadrupedForceControllerEvent> builder = new StateMachineBuilder<>(
            QuadrupedForceControllerState.class, "forceControllerState", registry);

      builder.addState(QuadrupedForceControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedForceControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedForceControllerState.STAND, standController);
      builder.addState(QuadrupedForceControllerState.STEP, stepController);

      builder.addTransition(QuadrupedForceControllerEvent.JOINTS_INITIALIZED,
            QuadrupedForceControllerState.JOINT_INITIALIZATION, QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerEvent.STARTING_POSE_REACHED,
            QuadrupedForceControllerState.STAND_PREP, QuadrupedForceControllerState.STAND);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND, QuadrupedForceControllerState.STAND_PREP,
            QuadrupedForceControllerState.STAND);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STEP, QuadrupedForceControllerState.STAND_PREP,
            QuadrupedForceControllerState.STEP);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STEP, QuadrupedForceControllerState.STAND,
            QuadrupedForceControllerState.STEP);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND_PREP, QuadrupedForceControllerState.STAND,
            QuadrupedForceControllerState.STAND_PREP);
      builder.addTransition(QuadrupedForceControllerEvent.REQUEST_STAND_PREP, QuadrupedForceControllerState.STEP,
            QuadrupedForceControllerState.STAND_PREP);

      this.stateMachine = builder.build(QuadrupedForceControllerState.JOINT_INITIALIZATION);
      this.userEventTrigger = new StateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry,
            QuadrupedForceControllerEvent.class);
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      stateMachine.process();
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
}
