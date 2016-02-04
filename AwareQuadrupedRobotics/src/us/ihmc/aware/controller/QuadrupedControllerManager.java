package us.ihmc.aware.controller;

import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.params.ParameterMapRepository;
import us.ihmc.aware.state.StateMachine;
import us.ihmc.aware.state.StateMachineBuilder;
import us.ihmc.aware.state.StateMachineYoVariableTrigger;
import us.ihmc.aware.vmc.QuadrupedVirtualModelController;
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
public class QuadrupedControllerManager implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<QuadrupedControllerState, QuadrupedControllerEvent> stateMachine;
   private final StateMachineYoVariableTrigger<QuadrupedControllerEvent> userEventTrigger;

   public QuadrupedControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedRobotParameters parameters)
   {
      ParameterMapRepository paramMapRepository = new ParameterMapRepository(registry);

      QuadrupedController jointInitializationController = new QuadrupedJointInitializationController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedStandPrepController(runtimeEnvironment,
            parameters, paramMapRepository);
      QuadrupedController standReadyController = new QuadrupedStandReadyController(runtimeEnvironment);
      QuadrupedController crawlController = new QuadrupedPositionBasedCrawlControllerAdapter(runtimeEnvironment, parameters, paramMapRepository);
      QuadrupedVirtualModelController virtualModelController = new QuadrupedVirtualModelController(
            runtimeEnvironment.getFullRobotModel(), parameters, registry, runtimeEnvironment.getGraphicsListRegistry());
      QuadrupedController virtualModelBasedStandController = new QuadrupedVirtualModelBasedStandController(
            runtimeEnvironment, parameters, paramMapRepository, virtualModelController);
      QuadrupedController virtualModelBasedStepController = new QuadrupedVirtualModelBasedStepController(
            runtimeEnvironment, parameters, paramMapRepository, virtualModelController);

      StateMachineBuilder<QuadrupedControllerState, QuadrupedControllerEvent> builder = new StateMachineBuilder<>();

      builder.addState(QuadrupedControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedControllerState.STAND_READY, standReadyController);
      builder.addState(QuadrupedControllerState.POSITION_BASED_CRAWL, crawlController);
      builder.addState(QuadrupedControllerState.VIRTUAL_MODEL_BASED_STAND, virtualModelBasedStandController);
      builder.addState(QuadrupedControllerState.VIRTUAL_MODEL_BASED_STEP, virtualModelBasedStepController);

      // TODO: Define more state transitions.
      builder.addTransition(QuadrupedControllerEvent.JOINTS_INITIALIZED, QuadrupedControllerState.JOINT_INITIALIZATION,
            QuadrupedControllerState.STAND_PREP);
      builder.addTransition(QuadrupedControllerEvent.STARTING_POSE_REACHED, QuadrupedControllerState.STAND_PREP,
            QuadrupedControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedControllerEvent.POSITION_BASED_CRAWL, QuadrupedControllerState.STAND_READY,
            QuadrupedControllerState.POSITION_BASED_CRAWL);
      builder.addTransition(QuadrupedControllerEvent.VIRTUAL_MODEL_BASED_STAND, QuadrupedControllerState.STAND_READY,
            QuadrupedControllerState.VIRTUAL_MODEL_BASED_STAND);
      builder.addTransition(QuadrupedControllerEvent.VIRTUAL_MODEL_BASED_STEP, QuadrupedControllerState.STAND_READY,
            QuadrupedControllerState.VIRTUAL_MODEL_BASED_STEP);

      this.stateMachine = builder.build(QuadrupedControllerState.JOINT_INITIALIZATION);
      this.userEventTrigger = new StateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry,
            QuadrupedControllerEvent.class);
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
