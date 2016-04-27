package us.ihmc.aware.controller.position;

import us.ihmc.aware.controller.ControllerEvent;
import us.ihmc.aware.controller.QuadrupedController;
import us.ihmc.aware.controller.QuadrupedControllerManager;
import us.ihmc.aware.controller.position.states.*;
import us.ihmc.aware.controller.positiondev.states.QuadrupedPositionBasedCenterOfMassVerificationController;
import us.ihmc.aware.controller.positiondev.states.QuadrupedPositionBasedLegJointSliderBoardController;
import us.ihmc.aware.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.aware.model.QuadrupedModelFactory;
import us.ihmc.aware.model.QuadrupedPhysicalProperties;
import us.ihmc.aware.model.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.providers.QuadrupedControllerInputProvider;
import us.ihmc.aware.state.FiniteStateMachine;
import us.ihmc.aware.state.FiniteStateMachineBuilder;
import us.ihmc.aware.state.FiniteStateMachineYoVariableTrigger;
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

   private final FiniteStateMachine<QuadrupedPositionControllerState, ControllerEvent> stateMachine;
   private final FiniteStateMachineYoVariableTrigger<QuadrupedPositionControllerRequestedEvent> userEventTrigger;

   public QuadrupedPositionControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, QuadrupedPositionStandPrepControllerParameters initialPositionParameters, QuadrupedPositionBasedCrawlControllerParameters crawlControllerParameters, QuadrupedLegInverseKinematicsCalculator legIKCalculator)
   {
      // Initialize input providers.
      QuadrupedControllerInputProvider inputProvider = new QuadrupedControllerInputProvider(runtimeEnvironment.getGlobalDataProducer(), registry);

      QuadrupedController jointInitializationController = new QuadrupedPositionJointInitializationController(
            runtimeEnvironment);
      QuadrupedController doNothingController = new QuadrupedPositionDoNothingController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedPositionStandPrepController(runtimeEnvironment, initialPositionParameters);
      QuadrupedController standReadyController = new QuadrupedPositionStandReadyController(runtimeEnvironment);
      QuadrupedController crawlController = new QuadrupedPositionBasedCrawlController(runtimeEnvironment, modelFactory, physicalProperties, crawlControllerParameters, inputProvider, legIKCalculator);
      QuadrupedController jointSliderBoardController = new QuadrupedPositionBasedLegJointSliderBoardController(runtimeEnvironment, registry);
      QuadrupedController comVerificationController = new QuadrupedPositionBasedCenterOfMassVerificationController(runtimeEnvironment, modelFactory, physicalProperties, legIKCalculator, registry);

      FiniteStateMachineBuilder<QuadrupedPositionControllerState, ControllerEvent> builder = new FiniteStateMachineBuilder<>(
            QuadrupedPositionControllerState.class, ControllerEvent.class, "positionControllerState", registry);

      builder.addState(QuadrupedPositionControllerState.JOINT_INITIALIZATION, jointInitializationController);
      builder.addState(QuadrupedPositionControllerState.DO_NOTHING, doNothingController);
      builder.addState(QuadrupedPositionControllerState.STAND_PREP, standPrepController);
      builder.addState(QuadrupedPositionControllerState.STAND_READY, standReadyController);
      builder.addState(QuadrupedPositionControllerState.CRAWL, crawlController);
      builder.addState(QuadrupedPositionControllerState.COM_VERIFICATION, comVerificationController);
      builder.addState(QuadrupedPositionControllerState.JOINT_SLIDER_BOARD, jointSliderBoardController);

      // TODO: Define more state transitions.
      builder.addTransition(ControllerEvent.DONE, QuadrupedPositionControllerState.JOINT_INITIALIZATION, QuadrupedPositionControllerState.DO_NOTHING);
      builder.addTransition(ControllerEvent.DONE, QuadrupedPositionControllerState.STAND_PREP, QuadrupedPositionControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.DO_NOTHING,
            QuadrupedPositionControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_CRAWL, QuadrupedPositionControllerState.STAND_READY,
            QuadrupedPositionControllerState.CRAWL);
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_COM_VERIFICATION, QuadrupedPositionControllerState.STAND_READY,
            QuadrupedPositionControllerState.COM_VERIFICATION);
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_JOINT_SLIDER_BOARD, QuadrupedPositionControllerState.STAND_READY,
            QuadrupedPositionControllerState.JOINT_SLIDER_BOARD);

      // Transitions from controllers back to stand prep.
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.CRAWL,
            QuadrupedPositionControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.COM_VERIFICATION,
            QuadrupedPositionControllerState.STAND_PREP);
      builder.addTransition(QuadrupedPositionControllerRequestedEvent.class, QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.JOINT_SLIDER_BOARD,
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
