package us.ihmc.quadrupedRobotics.controller.positionDevelopment;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.controller.position.states.*;
import us.ihmc.quadrupedRobotics.controller.positionDevelopment.states.QuadrupedPositionBasedCenterOfMassVerificationController;
import us.ihmc.quadrupedRobotics.controller.positionDevelopment.states.QuadrupedPositionBasedLegJointSliderBoardController;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachine;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineBuilder;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineYoVariableTrigger;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;

/**
 * A {@link RobotController} for switching between other robot controllers according to an internal finite state machine.
 * <p/>
 * Users can manually fire events on the {@code userTrigger} YoVariable.
 */
public class QuadrupedPositionDevelopmentControllerManager implements QuadrupedControllerManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final RobotMotionStatusHolder motionStatusHolder = new RobotMotionStatusHolder();

   private final FiniteStateMachine<QuadrupedPositionDevelopmentControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedPositionDevelopmentControllerRequestedEvent> userEventTrigger;

   public QuadrupedPositionDevelopmentControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedModelFactory modelFactory,
         QuadrupedPhysicalProperties physicalProperties, QuadrupedSimulationInitialPositionParameters initialPositionParameters,
         QuadrupedLegInverseKinematicsCalculator legIKCalculator)
   {
      this.stateMachine = buildStateMachine(runtimeEnvironment, modelFactory, physicalProperties, initialPositionParameters, legIKCalculator);
      this.userEventTrigger = new FiniteStateMachineYoVariableTrigger<>(stateMachine, "userTrigger", registry,
            QuadrupedPositionDevelopmentControllerRequestedEvent.class);
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

   private FiniteStateMachine<QuadrupedPositionDevelopmentControllerState, ControllerEvent> buildStateMachine(QuadrupedRuntimeEnvironment runtimeEnvironment,
         QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties,
         QuadrupedSimulationInitialPositionParameters initialPositionParameters,
         QuadrupedLegInverseKinematicsCalculator legIKCalculator)
   {
      QuadrupedController jointInitializationController = new QuadrupedPositionJointInitializationController(runtimeEnvironment);
      QuadrupedController doNothingController = new QuadrupedPositionDoNothingController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedPositionStandPrepController(runtimeEnvironment, initialPositionParameters);
      QuadrupedController standReadyController = new QuadrupedPositionStandReadyController(runtimeEnvironment);
      QuadrupedController jointSliderBoardController = new QuadrupedPositionBasedLegJointSliderBoardController(runtimeEnvironment, registry);
      QuadrupedController comVerificationController = new QuadrupedPositionBasedCenterOfMassVerificationController(runtimeEnvironment, modelFactory,
            physicalProperties, legIKCalculator, registry);

      FiniteStateMachineBuilder<QuadrupedPositionDevelopmentControllerState, ControllerEvent> builder = new FiniteStateMachineBuilder<>(
            QuadrupedPositionDevelopmentControllerState.class, ControllerEvent.class, "positionDevelopmentControllerState", registry);

      builder.addState(QuadrupedPositionDevelopmentControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedPositionDevelopmentControllerState.DO_NOTHING, doNothingController);
      builder.addState(QuadrupedPositionDevelopmentControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedPositionDevelopmentControllerState.STAND_READY, standReadyController);
      builder.addState(QuadrupedPositionDevelopmentControllerState.JOINT_SLIDER_BOARD, jointSliderBoardController);
      builder.addState(QuadrupedPositionDevelopmentControllerState.COM_VERIFICATION, comVerificationController);

      builder.addTransition(ControllerEvent.DONE, QuadrupedPositionDevelopmentControllerState.JOINT_INITIALIZATION,
            QuadrupedPositionDevelopmentControllerState.DO_NOTHING);
      builder.addTransition(ControllerEvent.DONE, QuadrupedPositionDevelopmentControllerState.STAND_PREP,
            QuadrupedPositionDevelopmentControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedPositionDevelopmentControllerRequestedEvent.class, QuadrupedPositionDevelopmentControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedPositionDevelopmentControllerState.DO_NOTHING, QuadrupedPositionDevelopmentControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionDevelopmentControllerRequestedEvent.class,
            QuadrupedPositionDevelopmentControllerRequestedEvent.REQUEST_COM_VERIFICATION, QuadrupedPositionDevelopmentControllerState.STAND_READY,
            QuadrupedPositionDevelopmentControllerState.COM_VERIFICATION);
      builder.addTransition(QuadrupedPositionDevelopmentControllerRequestedEvent.class,
            QuadrupedPositionDevelopmentControllerRequestedEvent.REQUEST_JOINT_SLIDER_BOARD, QuadrupedPositionDevelopmentControllerState.STAND_READY,
            QuadrupedPositionDevelopmentControllerState.JOINT_SLIDER_BOARD);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedPositionDevelopmentControllerRequestedEvent.class, QuadrupedPositionDevelopmentControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedPositionDevelopmentControllerState.COM_VERIFICATION, QuadrupedPositionDevelopmentControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionDevelopmentControllerRequestedEvent.class, QuadrupedPositionDevelopmentControllerRequestedEvent.REQUEST_STAND_PREP,
            QuadrupedPositionDevelopmentControllerState.JOINT_SLIDER_BOARD, QuadrupedPositionDevelopmentControllerState.STAND_PREP);

      return builder.build(QuadrupedPositionDevelopmentControllerState.JOINT_INITIALIZATION);
   }
}
