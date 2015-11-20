package us.ihmc.quadrupedRobotics.controller;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
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
   private RobotController stateEstimator; //not implemented yet
   
   
   public QuadrupedSimulationController(RawSensorReader sensorReader, OutputWriter outputWriter, QuadrupedControllerManager gaitControlManager)
   {
      this.sensorReader = sensorReader;
      this.outputWriter = outputWriter;
      this.gaitControlManager = gaitControlManager;
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
      gaitControlManager.doControl();
      outputWriter.write();
   }
}
