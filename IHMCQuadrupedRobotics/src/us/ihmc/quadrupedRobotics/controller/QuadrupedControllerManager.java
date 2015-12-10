package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.DefaultQuadrupedStandPrepParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedStandPrepParameters;
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
      this.stateEstimator = stateEstimator;
      this.virtualModelController = new QuadrupedVirtualModelController(sdfFullRobotModel, quadrupedRobotParameters, registry, yoGraphicsListRegistry);

      // configure state machine
      stateMachine = new GenericStateMachine<>("QuadrupedControllerStateMachine", "QuadrupedControllerSwitchTime", QuadrupedControllerState.class,
            robotTimestamp, registry);
      requestedState = new EnumYoVariable<>("QuadrupedControllerStateMachineRequestedState", registry, QuadrupedControllerState.class, true);

      QuadrupedStandPrepController standPrepController = new QuadrupedStandPrepController(quadrupedRobotParameters, sdfFullRobotModel,
            simulationDT);

      QuadrupedStandReadyController standReadyController = new QuadrupedStandReadyController(sdfFullRobotModel);

      QuadrupedVirtualModelBasedStandController virtualModelBasedStandController = new QuadrupedVirtualModelBasedStandController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, virtualModelController, robotTimestamp, registry, yoGraphicsListRegistry);

      QuadrupedPositionBasedCrawlController positionBasedCrawlController = new QuadrupedPositionBasedCrawlController(simulationDT, quadrupedRobotParameters,
            sdfFullRobotModel, stateEstimator, inverseKinematicsCalculators, globalDataProducer, robotTimestamp, registry, yoGraphicsListRegistry,
            yoGraphicsListRegistryForDetachedOverhead);

      stateMachine.addState(standPrepController);
      stateMachine.addState(standReadyController);
      stateMachine.addState(virtualModelBasedStandController);
      stateMachine.addState(positionBasedCrawlController);

      // Add automatic state transition from stand prep to stand ready.
      standPrepController.addStateTransition(new StateTransition<QuadrupedControllerState>(QuadrupedControllerState.STAND_READY,
            new QuadrupedStandPrepControllerExitCondition(standPrepController)));

      // Can transition to more complex controllers from stand ready.
      standReadyController
            .addStateTransition(new PermissiveRequestedStateTransition<QuadrupedControllerState>(requestedState, QuadrupedControllerState.VMC_STAND));
      standReadyController
            .addStateTransition(new PermissiveRequestedStateTransition<QuadrupedControllerState>(requestedState, QuadrupedControllerState.POSITION_CRAWL));

      // Can only go back to stand prep.
      positionBasedCrawlController
            .addStateTransition(new PermissiveRequestedStateTransition<QuadrupedControllerState>(requestedState, QuadrupedControllerState.STAND_PREP));
      virtualModelBasedStandController
            .addStateTransition(new PermissiveRequestedStateTransition<QuadrupedControllerState>(requestedState, QuadrupedControllerState.STAND_PREP));

      // TODO: Start in a "freeze" state.
      stateMachine.setCurrentState(QuadrupedControllerState.POSITION_CRAWL);
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
