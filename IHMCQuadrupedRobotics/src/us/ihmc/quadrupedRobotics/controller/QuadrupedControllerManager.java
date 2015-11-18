package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedDataProvider;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.simulationconstructionset.robotController.RawSensorReader;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedControllerManager implements RobotController
{
   private final RawSensorReader sensorReader;
   private final OutputWriter outputWriter;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final StateMachine<QuadrupedControllerState> stateMachine;
   private final EnumYoVariable<QuadrupedControllerState> requestedState;
   
   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters, QuadrupedDataProvider quadrupedDataProvider,
         RawSensorReader sensorReader, OutputWriter outputWriter, SDFFullRobotModel sdfFullRobotModel, DoubleYoVariable robotTimestamp,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead, QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators)
   {
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.yoGraphicsListRegistryForDetachedOverhead = yoGraphicsListRegistryForDetachedOverhead;
      this.robotTimestamp = robotTimestamp;
      
      // configure state machine
      stateMachine = new StateMachine<>("QuadrupedControllerStateMachine", "QuadrupedControllerSwitchTime", QuadrupedControllerState.class, robotTimestamp, registry);
      requestedState = new EnumYoVariable<>("QuadrupedControllerStateMachineRequestedState", registry, QuadrupedControllerState.class, true);
      QuadrupedVMCStandController vmcStandController = new QuadrupedVMCStandController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, robotTimestamp, yoGraphicsListRegistry);
      stateMachine.addState(vmcStandController);
      registry.addChild(vmcStandController.getYoVariableRegistry());
      
      QuadrupedPositionBasedCrawlController positionBasedCrawlController = new QuadrupedPositionBasedCrawlController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel,
            inverseKinematicsCalculators, yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead, quadrupedDataProvider, robotTimestamp);
      registry.addChild(positionBasedCrawlController.getYoVariableRegistry());
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
      
      sensorReader.read();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      outputWriter.write();
   }

}