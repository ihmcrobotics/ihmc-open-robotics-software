package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedDataProvider;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedControllerManager implements RobotController
{
   private final SDFFullRobotModel sdfFullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final StateMachine<QuadrupedControllerState> stateMachine;
   private final EnumYoVariable<QuadrupedControllerState> requestedState;
   private final EnumYoVariable<SliderBoardModes> sliderboardMode = new EnumYoVariable<>("sliderboardMode", registry, SliderBoardModes.class);
   
   public enum SliderBoardModes
   {
      POSITIONCRAWL_COM_SHIFT, POSITIONCRAWL_FOOTSTEP_CHOOSER, POSITIONCRAWL_ORIENTATION_TUNING
   }
   
   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters, SDFFullRobotModel sdfFullRobotModel,
         QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators, QuadrupedStateEstimator stateEstimator, QuadrupedDataProvider quadrupedDataProvider,
         DoubleYoVariable robotTimestamp, YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.yoGraphicsListRegistryForDetachedOverhead = yoGraphicsListRegistryForDetachedOverhead;
      this.robotTimestamp = robotTimestamp;
      
      // configure state machine
      stateMachine = new StateMachine<>("QuadrupedControllerStateMachine", "QuadrupedControllerSwitchTime", QuadrupedControllerState.class, robotTimestamp, registry);
      requestedState = new EnumYoVariable<>("QuadrupedControllerStateMachineRequestedState", registry, QuadrupedControllerState.class, true);
      
      QuadrupedVMCStandController vmcStandController = new QuadrupedVMCStandController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, robotTimestamp, registry, yoGraphicsListRegistry);
      
      QuadrupedPositionBasedCrawlController positionBasedCrawlController = new QuadrupedPositionBasedCrawlController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel,
            stateEstimator, inverseKinematicsCalculators, quadrupedDataProvider, robotTimestamp, registry, yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead);
      
      stateMachine.addState(vmcStandController);
      stateMachine.addState(positionBasedCrawlController);

      stateMachine.setCurrentState(QuadrupedControllerState.POSITION_CRAWL);
      requestedState.set(null);
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
      if(requestedState.getEnumValue() != null)
      {
         stateMachine.setCurrentState(requestedState.getEnumValue());
         requestedState.set(null);
      }
      
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

}