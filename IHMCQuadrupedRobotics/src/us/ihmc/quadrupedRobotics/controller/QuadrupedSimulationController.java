package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.simulationconstructionset.robotController.RawSensorReader;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class QuadrupedSimulationController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final RawSensorReader sensorReader;
   private final OutputWriter outputWriter;
   private final RobotController gaitControlManager;
   private RobotController headController; //not implemented yet
   private QuadrupedStateEstimator stateEstimator; //not implemented yet
   private final DRCPoseCommunicator poseCommunicator;
   
   public QuadrupedSimulationController(RawSensorReader sensorReader, OutputWriter outputWriter, QuadrupedControllerManager gaitControlManager, QuadrupedStateEstimator stateEstimator, DRCPoseCommunicator poseCommunicator)
   {
      this.poseCommunicator = poseCommunicator;
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.gaitControlManager = gaitControlManager;
      this.stateEstimator = stateEstimator;
      registry.addChild(gaitControlManager.getYoVariableRegistry());
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
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }

   @Override
   public void doControl()
   {
      sensorReader.read();
      stateEstimator.doControl();
      gaitControlManager.doControl();
      poseCommunicator.write();
      outputWriter.write();
   }
}
