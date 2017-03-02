package us.ihmc.quadrupedRobotics.controller.position;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlController;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionDoNothingController;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionJointInitializationController;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionStandPrepController;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionStandReadyController;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineYoVariableTrigger;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

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

   private final FiniteStateMachine<QuadrupedPositionControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedPositionControllerRequestedEvent> userEventTrigger;

   public QuadrupedPositionControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, QuadrupedSimulationInitialPositionParameters initialPositionParameters, QuadrupedPositionBasedCrawlControllerParameters crawlControllerParameters, QuadrupedLegInverseKinematicsCalculator legIKCalculator)
   {
      // Initialize input providers.
      QuadrupedPostureInputProvider postureProvider = new QuadrupedPostureInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);
      QuadrupedPlanarVelocityInputProvider planarVelocityProvider = new QuadrupedPlanarVelocityInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);

      QuadrupedController jointInitializationController = new QuadrupedPositionJointInitializationController(
            runtimeEnvironment);
      QuadrupedController doNothingController = new QuadrupedPositionDoNothingController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedPositionStandPrepController(runtimeEnvironment, initialPositionParameters);
      QuadrupedController standReadyController = new QuadrupedPositionStandReadyController(runtimeEnvironment);
      QuadrupedController crawlController = new QuadrupedPositionBasedCrawlController(runtimeEnvironment, modelFactory, physicalProperties, crawlControllerParameters, postureProvider, planarVelocityProvider, legIKCalculator);

      FiniteStateMachineBuilder<QuadrupedPositionControllerState, ControllerEvent> builder = new FiniteStateMachineBuilder<>(
            QuadrupedPositionControllerState.class, ControllerEvent.class, "positionControllerState", registry);

      builder.addState(QuadrupedPositionControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedPositionControllerState.DO_NOTHING, doNothingController);
      builder.addState(QuadrupedPositionControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedPositionControllerState.STAND_READY, standReadyController);
      builder.addState(QuadrupedPositionControllerState.CRAWL, crawlController);

      // TODO: Define more state transitions.
      builder.addTransition(ControllerEvent.DONE, QuadrupedPositionControllerState.JOINT_INITIALIZATION, QuadrupedPositionControllerState.DO_NOTHING);
      builder.addTransition(ControllerEvent.DONE, QuadrupedPositionControllerState.STAND_PREP, QuadrupedPositionControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.DO_NOTHING,
            QuadrupedPositionControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_CRAWL, QuadrupedPositionControllerState.STAND_READY,
            QuadrupedPositionControllerState.CRAWL);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.CRAWL,
            QuadrupedPositionControllerState.STAND_PREP);

      this.stateMachine = builder.build(QuadrupedPositionControllerState.JOINT_INITIALIZATION);
      this.userEventTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry, QuadrupedPositionControllerRequestedEvent.class);
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
