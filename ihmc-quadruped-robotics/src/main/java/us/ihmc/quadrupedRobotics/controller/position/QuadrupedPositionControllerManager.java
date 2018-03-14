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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.EventBasedStateMachineFactory;
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

   private final StateMachine<QuadrupedPositionControllerState, QuadrupedController> stateMachine;

   public QuadrupedPositionControllerManager(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, QuadrupedSimulationInitialPositionParameters initialPositionParameters, QuadrupedPositionBasedCrawlControllerParameters crawlControllerParameters, QuadrupedLegInverseKinematicsCalculator legIKCalculator)
   {
      // Initialize input providers.
      QuadrupedPostureInputProvider postureProvider = new QuadrupedPostureInputProvider(physicalProperties, runtimeEnvironment.getGlobalDataProducer(),
                                                                                        registry);
      QuadrupedPlanarVelocityInputProvider planarVelocityProvider = new QuadrupedPlanarVelocityInputProvider(runtimeEnvironment.getGlobalDataProducer(),
                                                                                                             registry);

      QuadrupedController jointInitializationController = new QuadrupedPositionJointInitializationController(runtimeEnvironment);
      QuadrupedController doNothingController = new QuadrupedPositionDoNothingController(runtimeEnvironment);
      QuadrupedController standPrepController = new QuadrupedPositionStandPrepController(runtimeEnvironment, initialPositionParameters, registry);
      QuadrupedController standReadyController = new QuadrupedPositionStandReadyController(runtimeEnvironment);
      QuadrupedController crawlController = new QuadrupedPositionBasedCrawlController(runtimeEnvironment, modelFactory, physicalProperties,
                                                                                      crawlControllerParameters, postureProvider, planarVelocityProvider,
                                                                                      legIKCalculator);

      EventBasedStateMachineFactory<QuadrupedPositionControllerState, QuadrupedController> factory = new EventBasedStateMachineFactory<>(QuadrupedPositionControllerState.class);
      factory.setNamePrefix("positionControllerState").setRegistry(registry).buildYoClock(runtimeEnvironment.getRobotTimestamp());
      factory.buildYoEventTrigger("userTrigger", QuadrupedPositionControllerRequestedEvent.class);

      factory.addState(QuadrupedPositionControllerState.JOINT_INITIALIZATION, jointInitializationController);
      factory.addState(QuadrupedPositionControllerState.DO_NOTHING, doNothingController);
      factory.addState(QuadrupedPositionControllerState.STAND_PREP, standPrepController);
      factory.addState(QuadrupedPositionControllerState.STAND_READY, standReadyController);
      factory.addState(QuadrupedPositionControllerState.CRAWL, crawlController);

      // TODO: Define more state transitions.
      factory.addTransition(ControllerEvent.DONE, QuadrupedPositionControllerState.JOINT_INITIALIZATION, QuadrupedPositionControllerState.DO_NOTHING);
      factory.addTransition(ControllerEvent.DONE, QuadrupedPositionControllerState.STAND_PREP, QuadrupedPositionControllerState.STAND_READY);

      // Manually triggered events to transition to main controllers.
      factory.addTransition(QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.DO_NOTHING,
                            QuadrupedPositionControllerState.STAND_PREP);
      factory.addTransition(QuadrupedPositionControllerRequestedEvent.REQUEST_CRAWL, QuadrupedPositionControllerState.STAND_READY,
                            QuadrupedPositionControllerState.CRAWL);

      // Transitions from controllers back to stand prep.
      factory.addTransition(QuadrupedPositionControllerRequestedEvent.REQUEST_STAND_PREP, QuadrupedPositionControllerState.CRAWL,
                            QuadrupedPositionControllerState.STAND_PREP);

      this.stateMachine = factory.build(QuadrupedPositionControllerState.JOINT_INITIALIZATION);
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
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
