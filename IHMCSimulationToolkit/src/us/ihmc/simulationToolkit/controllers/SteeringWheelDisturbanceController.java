package us.ihmc.simulationToolkit.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.environments.ContactableToroidRobot;

public class SteeringWheelDisturbanceController implements RobotController
{  
   private final String NAME = "SteeringWheelDisturbanceController";
   private final ContactableToroidRobot robot;
   
   private YoVariableRegistry registry = new YoVariableRegistry(NAME);
   private YoFunctionGenerator fnGenerator;

   public SteeringWheelDisturbanceController(ContactableToroidRobot robot, YoFunctionGeneratorMode mode)
   {
      fnGenerator = new YoFunctionGenerator(NAME + "FunctionGenerator", robot.getYoTime(), registry);
      this.robot = robot;
      fnGenerator.setMode(mode);
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
      return NAME;
   }

   @Override
   public String getDescription()
   {      
      return "Perturb steering wheel based on a YoFunctionGenerator";
   }

   @Override
   public void doControl()
   {
      robot.setTau(fnGenerator.getValue());
   }

}
