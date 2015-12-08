package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.GenericStateMachine;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedControllerManager implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStateEstimator stateEstimator;

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

      // configure state machine
      stateMachine = new GenericStateMachine<>("QuadrupedControllerStateMachine", "QuadrupedControllerSwitchTime", QuadrupedControllerState.class,
            robotTimestamp, registry);
      requestedState = new EnumYoVariable<>("QuadrupedControllerStateMachineRequestedState", registry, QuadrupedControllerState.class, true);

      QuadrupedVMCStandController vmcStandController = new QuadrupedVMCStandController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel,
            robotTimestamp, registry, yoGraphicsListRegistry);

      QuadrupedPositionBasedCrawlController positionBasedCrawlController = new QuadrupedPositionBasedCrawlController(simulationDT, quadrupedRobotParameters,
            sdfFullRobotModel, stateEstimator, inverseKinematicsCalculators, globalDataProducer, robotTimestamp, registry, yoGraphicsListRegistry,
            yoGraphicsListRegistryForDetachedOverhead);

      stateMachine.addState(vmcStandController);
      stateMachine.addState(positionBasedCrawlController);

      // Add valid transitions from controller to controller.
      // TODO: More comprehensive transition conditions can be implemented. For
      // instance, checking if the robot is stationary before the transition to
      // a standing controller.
      positionBasedCrawlController
            .addStateTransition(new PermissiveRequestedStateTransition<QuadrupedControllerState>(requestedState, QuadrupedControllerState.VMC_STAND));
      vmcStandController
            .addStateTransition(new PermissiveRequestedStateTransition<QuadrupedControllerState>(requestedState, QuadrupedControllerState.POSITION_CRAWL));

      // TODO: Start in a "freeze" state.
      stateMachine.setCurrentState(QuadrupedControllerState.POSITION_CRAWL);
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