package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;

public class Step4Controller implements RobotController
{

   //Variables
   private Step4Robot rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   private double deltaT;
   //private double ffComponent = -38.0 * 9.81; TODO CAREFULL!! Now both feet are on the ground so the FF term is different
   private YoDouble distanceBetweenFeet;
   private YoDouble desiredLegLength;
   private YoDouble desiredFeetSpacing;

   private PIDController controllerBodyZ;
   private PIDController controllerBodyX;
   private PIDController controllerLegPitch;
   private PIDController controllerKneeZ;

   private YoDouble desiredBodyZ;
   private YoDouble desiredBodyX;
   private YoDouble desiredLegPitch;
   private YoDouble desiredKneeZ;
   private YoDouble desiredBodyPitch;

   private YoDouble kneeTau;
   private YoDouble hipTau;

   public Step4Controller(Step4Robot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(50000.0);
      controllerBodyZ.setDerivativeGain(5000.0);
      desiredBodyZ = new YoDouble("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.4);

      controllerBodyX = new PIDController("bodyX", controllerRegistry);
      controllerBodyX.setProportionalGain(1e3);
      controllerBodyX.setDerivativeGain(1e2);
      desiredBodyX = new YoDouble("desiredBodyX", controllerRegistry);
      desiredBodyX.set(0.5);

      controllerLegPitch = new PIDController("legPitch", controllerRegistry);
      controllerLegPitch.setProportionalGain(500000.0); //50000 (previous value)
      controllerLegPitch.setDerivativeGain(50000.0);  //5000
      desiredLegPitch = new YoDouble("desiredLegPitch", controllerRegistry);
      desiredLegPitch.set(0.4);

      controllerKneeZ = new PIDController("kneeZ", controllerRegistry);
      controllerKneeZ.setProportionalGain(15000.0);
      controllerKneeZ.setDerivativeGain(2000.0);
      desiredKneeZ = new YoDouble("desiredKneeZ", controllerRegistry);
      desiredKneeZ.set(0.2);

      hipTau = new YoDouble("hipTau", controllerRegistry);
      kneeTau = new YoDouble("kneeTau", controllerRegistry);
      distanceBetweenFeet = new YoDouble("distaceBetweenFeet", controllerRegistry);
      desiredLegLength = new YoDouble("legLength", controllerRegistry);
      desiredFeetSpacing = new YoDouble("desiredFeetSpacing", controllerRegistry);
      desiredBodyPitch = new YoDouble("desiredBodyPitch", controllerRegistry);
      desiredBodyPitch.set(0.0);
   }

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
      distanceBetweenFeet.set(rob.getFeetDistance()); // Variable for visualization purposes only

      for (RobotSide robotSide : RobotSide.values())
      {
         /********** Body height ****************/
         kneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocityZ(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue()); //+ ffComponent);

         if (robotSide == RobotSide.LEFT)
         {
            /********** Body Pitch ************/
            hipTau.set(controllerLegPitch.compute(rob.getBodyPitch(), 0.0, rob.getHipVelocity(robotSide), 0.0, deltaT));
            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         }
         else
         {
            /********** Body Pitch ************/
            hipTau.set(controllerLegPitch.compute(rob.getBodyPitch(), 0.0, rob.getHipVelocity(robotSide), 0.0, deltaT));
            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         }
        
         //(1) THIS IS VERY WRONG BECAUSE THE KNEE CANNOT CONTROL BOTH THE HEIGHT AND THE FEET SPACING! EACH JOINT MUST CONTROL ONE PARAMETER
         //       /************** Leg Length **************/ 
         //       kneeTau.set(controllerBodyZ.compute(rob.getLegLength(robotSide), desiredLegLength.getDoubleValue(), -rob.getKneeVelocityZ(robotSide),0.0, deltaT));
         //       rob.setKneeTau(robotSide, -kneeTau.getDoubleValue());

         //(2) WORKS 
         //         if (robotSide == RobotSide.LEFT)
         //         {
         //            /********** Leg Swing ************/
         //            hipTau.set(controllerLegPitch.compute(rob.getLegPitch(robotSide), -desiredLegPitch.getDoubleValue(), rob.getLegPitchVel(robotSide), 0.0,deltaT));
         //            rob.setHipTau(robotSide, hipTau.getDoubleValue());  
         //         }
         //         else
         //         {
         //            /********** Leg Swing ************/
         //            hipTau.set(controllerLegPitch.compute(rob.getLegPitch(robotSide), desiredLegPitch.getDoubleValue(), rob.getLegPitchVel(robotSide), 0.0,deltaT));
         //            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //          }

         //(3) EXPLODES
         //         if (robotSide == RobotSide.LEFT)
         //          {
         //            /********** Feet Spacing ************/
         //            hipTau.set(controllerLegPitch.compute(rob.getFeetDistance(), desiredFeetSpacing.getDoubleValue() , rob.getLegPitchVel(robotSide), 0.0, deltaT));
         //            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //          }
         // 
         //          else
         //          {
         //             /********** Feet Spacing ************/
         //             hipTau.set(controllerLegPitch.compute(rob.getFeetDistance()/2.0, desiredFeetSpacing.getDoubleValue()/2.0 , rob.getLegPitchVel(robotSide), 0.0, deltaT));
         //             rob.setHipTau(robotSide, -hipTau.getDoubleValue());
         //         }

      }
   }
}
