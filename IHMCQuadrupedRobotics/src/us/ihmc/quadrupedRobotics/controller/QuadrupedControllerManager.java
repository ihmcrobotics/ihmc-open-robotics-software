package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerStateMachineBuilder;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedCommonControllerParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.sliderboard.QuadrupedSliderBoardMode;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedVirtualModelController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
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

   private final DoubleYoVariable controllerTimestamp;
   private final RobotMotionStatusHolder robotMotionStatusHolder = new RobotMotionStatusHolder();

   private final EnumYoVariable<QuadrupedSliderBoardMode> sliderboardMode = QuadrupedSliderBoardMode.createYoVariable(registry);

   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters,
         SDFFullRobotModel sdfFullRobotModel, QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators,
         QuadrantDependentList<FootSwitchInterface> footSwitches, GlobalDataProducer globalDataProducer,
         DoubleYoVariable robotTimestamp, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      // Default to QuadrupedControllerState#DO_NOTHING.
      this(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, inverseKinematicsCalculators, footSwitches,
            globalDataProducer, robotTimestamp, yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead,
            QuadrupedControllerState.DO_NOTHING);
   }

   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters,
         SDFFullRobotModel sdfFullRobotModel, QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators,
         QuadrantDependentList<FootSwitchInterface> footSwitches, GlobalDataProducer globalDataProducer,
         DoubleYoVariable controllerTimestamp, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead, QuadrupedControllerState startState)
   {
      this.controllerTimestamp = controllerTimestamp;
      
      QuadrupedCommonControllerParameters commonControllerParameters = new QuadrupedCommonControllerParameters(
            simulationDT, controllerTimestamp, sdfFullRobotModel, footSwitches, registry, yoGraphicsListRegistry,
            yoGraphicsListRegistryForDetachedOverhead);

      QuadrupedVirtualModelController virtualModelController = new QuadrupedVirtualModelController(sdfFullRobotModel,
            quadrupedRobotParameters, registry, yoGraphicsListRegistry);

      // Build controller state machine.
      QuadrupedControllerStateMachineBuilder builder = new QuadrupedControllerStateMachineBuilder(
            commonControllerParameters, quadrupedRobotParameters, requestedState);
      builder.addDoNothingController();
      builder.addStandPrepController();

      builder.addStandReadyController();
//      builder.addQuadrupedCenterOfMassVerificationController(inverseKinematicsCalculators);
      builder.addPositionBasedCrawlController(inverseKinematicsCalculators, globalDataProducer);
      builder.addVirtualModelBasedStandController(virtualModelController);
      builder.addTrotWalkController();
      builder.addSliderBoardController();
      
      builder.addPermissibleTransition(QuadrupedControllerState.POSITION_CRAWL, QuadrupedControllerState.SLIDER_BOARD);
      builder.addPermissibleTransition(QuadrupedControllerState.DO_NOTHING, QuadrupedControllerState.STAND_PREP);
      builder.addPermissibleTransition(QuadrupedControllerState.DO_NOTHING, QuadrupedControllerState.SLIDER_BOARD);
      builder.addPermissibleTransition(QuadrupedControllerState.STAND_PREP, QuadrupedControllerState.POSITION_CRAWL);
      builder.addPermissibleTransition(QuadrupedControllerState.STAND_PREP, QuadrupedControllerState.TROT_WALK);
      builder.addPermissibleTransition(QuadrupedControllerState.STAND_PREP, QuadrupedControllerState.SLIDER_BOARD);
//      builder.addPermissibleTransition(QuadrupedControllerState.STAND_PREP, QuadrupedControllerState.COM_VERIFICATION);
//      builder.addPermissibleTransition(QuadrupedControllerState.COM_VERIFICATION, QuadrupedControllerState.STAND_PREP);
      builder.addPermissibleTransition(QuadrupedControllerState.SLIDER_BOARD, QuadrupedControllerState.STAND_PREP);

      this.stateMachine = builder.build();

      // Transition to the start state. The entry action must be triggered manually.
      stateMachine.setCurrentState(startState);
      stateMachine.getCurrentState().doTransitionIntoAction();

      // Set the initial requested state to the actual start state.
      requestedState.set(startState);
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
      robotMotionStatusHolder.setCurrentRobotMotionStatus(stateMachine.getCurrentState().getMotionStatus());

      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   public RobotMotionStatusHolder getRobotMotionStatusHolder()
   {
      return robotMotionStatusHolder;
   }
}
