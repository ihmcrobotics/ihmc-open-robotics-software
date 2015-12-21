package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedVirtualModelController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedControllerManager implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStateEstimator stateEstimator;
   private final QuadrupedVirtualModelController virtualModelController;

   private final GenericStateMachine<QuadrupedControllerState, QuadrupedController> stateMachine;

   private final EnumYoVariable<QuadrupedControllerState> requestedState;
   private final EnumYoVariable<SliderBoardModes> sliderboardMode = new EnumYoVariable<>("sliderboardMode", registry, SliderBoardModes.class);

   private final DoubleYoVariable robotTimestamp = new DoubleYoVariable("robotTimestamp", registry);

   private final RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();

   public enum SliderBoardModes
   {
      POSITIONCRAWL_COM_SHIFT, POSITIONCRAWL_FOOTSTEP_CHOOSER, POSITIONCRAWL_ORIENTATION_TUNING
   }

   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters, SDFFullRobotModel sdfFullRobotModel,
         QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators, QuadrupedStateEstimator stateEstimator, GlobalDataProducer globalDataProducer,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      this(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, inverseKinematicsCalculators, stateEstimator, globalDataProducer, yoGraphicsListRegistry,
            yoGraphicsListRegistryForDetachedOverhead, QuadrupedControllerState.DO_NOTHING);
   }

   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters, SDFFullRobotModel sdfFullRobotModel,
         QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators, QuadrupedStateEstimator stateEstimator, GlobalDataProducer globalDataProducer,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead,
         QuadrupedControllerState startState)
   {
      this.stateEstimator = stateEstimator;
      this.virtualModelController = new QuadrupedVirtualModelController(sdfFullRobotModel, quadrupedRobotParameters, registry, yoGraphicsListRegistry);

      // configure state machine
      stateMachine = new GenericStateMachine<>("QuadrupedControllerStateMachine", "QuadrupedControllerSwitchTime", QuadrupedControllerState.class,
            robotTimestamp, registry);
      requestedState = new EnumYoVariable<>("QuadrupedControllerStateMachineRequestedState", registry, QuadrupedControllerState.class, true);
      
      QuadrupedDoNothingController doNothingController = new QuadrupedDoNothingController(sdfFullRobotModel);

      QuadrupedStandPrepController standPrepController = new QuadrupedStandPrepController(quadrupedRobotParameters, sdfFullRobotModel,
            simulationDT);

      QuadrupedStandReadyController standReadyController = new QuadrupedStandReadyController();

      QuadrupedVirtualModelBasedStandController virtualModelBasedStandController = new QuadrupedVirtualModelBasedStandController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, virtualModelController, robotTimestamp, registry, yoGraphicsListRegistry);

      QuadrupedPositionBasedCrawlController positionBasedCrawlController = new QuadrupedPositionBasedCrawlController(simulationDT, quadrupedRobotParameters,
            sdfFullRobotModel, stateEstimator, inverseKinematicsCalculators, globalDataProducer, robotTimestamp, registry, yoGraphicsListRegistry,
            yoGraphicsListRegistryForDetachedOverhead);

      QuadrupedLegJointSliderBoardController sliderBoardController = new QuadrupedLegJointSliderBoardController(sdfFullRobotModel, registry);

      stateMachine.addState(doNothingController);
      stateMachine.addState(standPrepController);
      stateMachine.addState(standReadyController);
      stateMachine.addState(virtualModelBasedStandController);
      stateMachine.addState(positionBasedCrawlController);
      stateMachine.addState(sliderBoardController);
      
      // Can transition from do nothing to only stand prep
      doNothingController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.STAND_PREP));

      // Add automatic state transition from stand prep to stand ready.
      standPrepController.addStateTransition(new StateTransition<>(QuadrupedControllerState.STAND_READY,
            new QuadrupedStandPrepControllerExitCondition(standPrepController)));

      // Can transition to more complex controllers from stand ready.
      standReadyController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.VMC_STAND));
      standReadyController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.POSITION_CRAWL));
      standReadyController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.SLIDER_BOARD));

      // Can only go back to stand prep.
      positionBasedCrawlController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.STAND_PREP));
      virtualModelBasedStandController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.STAND_PREP));
      sliderBoardController
            .addStateTransition(new PermissiveRequestedStateTransition<>(requestedState, QuadrupedControllerState.STAND_PREP));

      // TODO: Start in a "freeze" state.
      stateMachine.setCurrentState(startState);
      stateMachine.getCurrentState().doTransitionIntoAction();
   }

   @Override
   public void initialize()
   {

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
      return null;
   }

   @Override
   public void doControl()
   {
      robotTimestamp.set(stateEstimator.getCurrentTime());

      robotMotionStatusHolder.setCurrentRobotMotionStatus(stateMachine.getCurrentState().getMotionStatus());

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   public RobotMotionStatusHolder getRobotMotionStatusHolder()
   {
      return robotMotionStatusHolder;
   }
}
