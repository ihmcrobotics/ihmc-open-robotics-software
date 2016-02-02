package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerStateMachineBuilder;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedCommonControllerParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.sliderboard.SliderBoardModes;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedVirtualModelController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

/**
 * A {@link RobotController} for switching between quadruped control states. The manager will ensure that the robot is
 * always in a valid control state by disallowing dangerous transitions between states (e.g. transitioning from a run to
 * a stand controller in mid stride).
 */
public class QuadrupedControllerManager implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final GenericStateMachine<QuadrupedControllerState, QuadrupedController> stateMachine;
   private final EnumYoVariable<QuadrupedControllerState> requestedState = new EnumYoVariable<>("QuadrupedControllerStateMachineRequestedState", registry,
            QuadrupedControllerState.class, true);

   private final QuadrupedStateEstimator stateEstimator;
   private final DoubleYoVariable robotTimestamp = new DoubleYoVariable("robotTimestamp", registry);
   private final RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();

   private final EnumYoVariable<SliderBoardModes> sliderboardMode = new EnumYoVariable<>("sliderboardMode", registry,
         SliderBoardModes.class);

   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters,
         SDFFullRobotModel sdfFullRobotModel, QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators,
         QuadrupedStateEstimator stateEstimator, GlobalDataProducer globalDataProducer,
         YoGraphicsListRegistry yoGraphicsListRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      // Default to QuadrupedControllerState#DO_NOTHING.
      this(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, inverseKinematicsCalculators, stateEstimator,
            globalDataProducer, yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead,
            QuadrupedControllerState.DO_NOTHING);
   }

   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters,
         SDFFullRobotModel sdfFullRobotModel, QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators,
         QuadrupedStateEstimator stateEstimator, GlobalDataProducer globalDataProducer,
         YoGraphicsListRegistry yoGraphicsListRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead, QuadrupedControllerState startState)
   {
      this.stateEstimator = stateEstimator;

      QuadrupedCommonControllerParameters commonControllerParameters = new QuadrupedCommonControllerParameters(
            simulationDT, robotTimestamp, sdfFullRobotModel, stateEstimator, registry, yoGraphicsListRegistry,
            yoGraphicsListRegistryForDetachedOverhead);

      QuadrupedVirtualModelController virtualModelController = new QuadrupedVirtualModelController(sdfFullRobotModel,
            quadrupedRobotParameters, registry, yoGraphicsListRegistry);

      // Build controller state machine.
      QuadrupedControllerStateMachineBuilder builder = new QuadrupedControllerStateMachineBuilder(
            commonControllerParameters, quadrupedRobotParameters, requestedState);
      builder.addDoNothingController();
      builder.addStandPrepController();

      if (startState == QuadrupedControllerState.STAND_READY)
      {
         builder.addStandReadyController();
      }
      if (startState == QuadrupedControllerState.POSITION_CRAWL)
      {
         builder.addPositionBasedCrawlController(inverseKinematicsCalculators, globalDataProducer);
      }
      if (startState == QuadrupedControllerState.VMC_STAND)
      {
         builder.addVirtualModelBasedStandController(virtualModelController);
      }
      if (startState == QuadrupedControllerState.TROT_WALK)
      {
         builder.addTrotWalkController();
      }
      if (startState == QuadrupedControllerState.SLIDER_BOARD)
      {
         builder.addSliderBoardController();
      }

      builder.addJointsInitializedCondition(QuadrupedControllerState.DO_NOTHING, QuadrupedControllerState.STAND_PREP);
      builder.addStandingExitCondition(QuadrupedControllerState.STAND_PREP, startState);

      this.stateMachine = builder.build();

      // Transition to the start state. The entry action must be triggered manually.
      stateMachine.setCurrentState(QuadrupedControllerState.STAND_PREP);
      stateMachine.getCurrentState().doTransitionIntoAction();

      // Set the initial requested state to the actual start state.
      requestedState.set(QuadrupedControllerState.STAND_PREP);
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
