package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class Step0Controller implements RobotController
{
   //Variables
   private Step0Robot rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   
   private double deltaT;
   private PIDController zController;
   private DoubleYoVariable desiredZ;
   private DoubleYoVariable controlOutput;
   
   //Constructor
   public Step0Controller(Step0Robot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      //PD controller
      zController = new PIDController("z", controllerRegistry);
      zController.setProportionalGain(1e6);
      zController.setDerivativeGain(30000);
      //zController.setDerivativeGain(0.0);
      
      desiredZ = new DoubleYoVariable("desiredZ", controllerRegistry);
      desiredZ.set(1.4); //Height I want to maintain
      
      controlOutput = new DoubleYoVariable("controlOutput",controllerRegistry);
   }
   
   //Methods

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return controllerRegistry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      //controlOutput.set(zController.compute(rob.getKneePositionZ(), desiredZ.getDoubleValue(), rob.getKneeVelocityZ(), 0.0, deltaT)); //the 0.0 is because we are trying for it to have velocity 0 (dampen the oscillation)
      controlOutput.set(zController.compute(rob.getBodyPositionZ(), desiredZ.getDoubleValue(), rob.getBodyVelocityZ(), 0.0, deltaT));
      rob.setKneeForce(-controlOutput.getDoubleValue() + 24.0 * 9.81); //PD + FF
   }

}
