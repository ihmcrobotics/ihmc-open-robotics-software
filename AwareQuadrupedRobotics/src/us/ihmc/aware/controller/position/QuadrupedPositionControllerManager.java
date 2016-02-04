package us.ihmc.aware.controller.position;

import us.ihmc.aware.controller.QuadrupedController;
import us.ihmc.aware.controller.QuadrupedControllerManager;
import us.ihmc.aware.controller.force.QuadrupedVirtualModelBasedStandController;
import us.ihmc.aware.controller.force.QuadrupedVirtualModelBasedStepController;
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
public class QuadrupedPositionControllerManager implements QuadrupedControllerManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final StateMachine<QuadrupedPositionControllerState, QuadrupedPositionControllerEvent> stateMachine;
   private final StateMachineYoVariableTrigger<QuadrupedPositionControllerEvent> userEventTrigger;

   public QuadrupedPositionControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedRobotParameters parameters)
   {
      ParameterMapRepository paramMapRepository = new ParameterMapRepository(registry);

      QuadrupedController jointInitializationController = new QuadrupedPositionJointInitializationController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedPositionStandPrepController(runtimeEnvironment,
            parameters, paramMapRepository);
      QuadrupedController standReadyController = new QuadrupedPositionStandReadyController(runtimeEnvironment);
      QuadrupedController crawlController = new QuadrupedPositionBasedCrawlControllerAdapter(runtimeEnvironment, parameters, paramMapRepository);

      StateMachineBuilder<QuadrupedPositionControllerState, QuadrupedPositionControllerEvent> builder = new StateMachineBuilder<>();

      builder.addState(QuadrupedPositionControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedPositionControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedPositionControllerState.STAND_READY, standReadyController);
      builder.addState(QuadrupedPositionControllerState.CRAWL, crawlController);

      // TODO: Define more state transitions.
      builder.addTransition(QuadrupedPositionControllerEvent.JOINTS_INITIALIZED, QuadrupedPositionControllerState.JOINT_INITIALIZATION,
            QuadrupedPositionControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionControllerEvent.STARTING_POSE_REACHED, QuadrupedPositionControllerState.STAND_PREP,
            QuadrupedPositionControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedPositionControllerEvent.REQUEST_POSITION_BASED_CRAWL, QuadrupedPositionControllerState.STAND_READY,
            QuadrupedPositionControllerState.CRAWL);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedPositionControllerEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.CRAWL,
            QuadrupedPositionControllerState.STAND_PREP);

      this.stateMachine = builder.build(QuadrupedPositionControllerState.JOINT_INITIALIZATION);
      this.userEventTrigger = new StateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry,
            QuadrupedPositionControllerEvent.class);
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
