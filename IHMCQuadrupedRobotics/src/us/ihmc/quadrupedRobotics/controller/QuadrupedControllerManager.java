package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.dataProviders.QuadrupedDataProvider;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
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
   
   public QuadrupedControllerManager(double simulationDT, QuadrupedRobotParameters quadrupedRobotParameters, QuadrupedDataProvider quadrupedDataProvider,
         RawSensorReader sensorReader, OutputWriter outputWriter, SDFFullRobotModel sdfFullRobotModel, DoubleYoVariable robotTimestamp,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.yoGraphicsListRegistryForDetachedOverhead = yoGraphicsListRegistryForDetachedOverhead;
      this.robotTimestamp = robotTimestamp;
      
      // configure state machine
      stateMachine = new StateMachine<>("QuadrupedControllerStateMachine", "QuadrupedControllerSwitchTime", QuadrupedControllerState.class, robotTimestamp, registry);
      QuadrupedVMCStandController state = new QuadrupedVMCStandController(simulationDT, quadrupedRobotParameters, sdfFullRobotModel, robotTimestamp, yoGraphicsListRegistry);
      stateMachine.addState(state);
      stateMachine.setCurrentState(QuadrupedControllerState.VMC_STAND);
      registry.addChild(state.getYovariableRegistry());
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      // TODO Auto-generated method stub
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void doControl()
   {
      sensorReader.read();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      outputWriter.write();
   }

}