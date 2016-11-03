package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class Step2Controller implements RobotController
{
   
  /**
   * Variables
   */
   private Step2Robot rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   
   private double deltaT;
   private PIDController controllerBodyZ;
   private PIDController controllerBodyPitch;
   private DoubleYoVariable desiredBodyZ;
   private DoubleYoVariable desiredBodyPitch;
   private DoubleYoVariable controlKneeTau;
   private DoubleYoVariable controlHipTau;
   
   /**
    * Constructor
    */
   public Step2Controller(Step2Robot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;
      
      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(1e4);
      controllerBodyZ.setDerivativeGain(1e3);
      desiredBodyZ = new DoubleYoVariable("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.4);
      controlKneeTau = new DoubleYoVariable("controlKneeTau",controllerRegistry);
      
      controllerBodyPitch = new PIDController("bodyPitch", controllerRegistry);
      controllerBodyPitch.setProportionalGain(1e3);
      controllerBodyPitch.setDerivativeGain(1e2);
      desiredBodyPitch = new DoubleYoVariable("desiredBodyPitch", controllerRegistry);
      desiredBodyPitch.set(-0.1);
      controlHipTau = new DoubleYoVariable("controlHipTau", controllerRegistry);
      
   }
   
   
   /**
    * Methods
    */

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
      controlKneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), rob.getBodyVelocityZ(), 0.0, deltaT));
      rob.setKneeForce(-controlKneeTau.getDoubleValue() ); //- 24.0 * 9.81
      
      controlHipTau.set(controllerBodyPitch.compute(rob.getBodyPitch(), desiredBodyPitch.getDoubleValue(), rob.getBodyPitchVel(), 0.0, deltaT));
      rob.setHipForce(-controlHipTau.getDoubleValue());
   }

}
