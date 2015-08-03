package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class Step5Controller implements RobotController
{

   //Variables
   private Step5IDandSCSRobot rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   private double deltaT;

   private PIDController controllerBodyZ;
   private PIDController controllerBodyX;
   private PIDController controllerLegPitch;
   private PIDController controllerKneeZ;

   private DoubleYoVariable desiredBodyZ;
   private DoubleYoVariable desiredBodyX;
   private DoubleYoVariable desiredLegPitch;
   private DoubleYoVariable desiredKneeZ;
   private DoubleYoVariable desiredBodyPitch;

   private DoubleYoVariable kneeTau;
   private DoubleYoVariable hipTau;

   Point3d bodyPositionToPack = new Point3d(); //global so that it is created only once to avoid generating garbage
   private Quat4d rotationToPack;

   // Controllers and Gains
   public Step5Controller(Step5IDandSCSRobot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(50000.0);
      controllerBodyZ.setDerivativeGain(5000.0);
      desiredBodyZ = new DoubleYoVariable("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.4);

      //      controllerBodyX = new PIDController("bodyX", controllerRegistry);
      //      controllerBodyX.setProportionalGain(1e3);
      //      controllerBodyX.setDerivativeGain(1e2);
      //      desiredBodyX = new DoubleYoVariable("desiredBodyX", controllerRegistry);
      //      desiredBodyX.set(0.5);

      controllerLegPitch = new PIDController("legPitch", controllerRegistry);
      controllerLegPitch.setProportionalGain(50000.0);
      controllerLegPitch.setDerivativeGain(5000.0);
      desiredLegPitch = new DoubleYoVariable("desiredLegPitch", controllerRegistry);
      desiredLegPitch.set(0.4);

      //      controllerKneeZ = new PIDController("kneeZ", controllerRegistry);
      //      controllerKneeZ.setProportionalGain(15000.0);
      //      controllerKneeZ.setDerivativeGain(2000.0);
      //      desiredKneeZ = new DoubleYoVariable("desiredKneeZ", controllerRegistry);
      //      desiredKneeZ.set(0.2);

      hipTau = new DoubleYoVariable("hipTau", controllerRegistry);
      kneeTau = new DoubleYoVariable("kneeTau", controllerRegistry);

      desiredBodyPitch = new DoubleYoVariable("desiredBodyPitch", controllerRegistry);
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

   //Control Actions
   @Override
   public void doControl()
   {
      rob.updateIDRobot();
      
      for (RobotSide robotSide : RobotSide.values())
      {
         /********** Body height ****************/
         rob.getBodyPoint(bodyPositionToPack);
         kneeTau.set(controllerBodyZ.compute(bodyPositionToPack.getZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue());

         //         if (robotSide == RobotSide.LEFT)
         //         {
         //            /********** Body Pitch ************/
         //            hipTau.set(controllerLegPitch.compute(rob.getBodyPitch(rotationToPack), desiredBodyPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         //            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //         }
         //         else
         //         {
         //            /********** Body Pitch ************/
         //            hipTau.set(controllerLegPitch.compute(rob.getBodyPitch(rotationToPack), 0.0, rob.getHipVelocity(robotSide), 0.0, deltaT));
         //            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //         }
         
         if (robotSide == RobotSide.LEFT)
         {
            /********** Leg Swing ************/
            hipTau.set(controllerLegPitch.compute(rob.getHipPitch(robotSide), -desiredLegPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         }
         else
         {
            /********** Leg Swing ************/
            hipTau.set(controllerLegPitch.compute(rob.getHipPitch(robotSide), desiredLegPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         }
         
         rob.applyTorques(); //Note: this is getting called twice, one per side
         
      }
      
      
   }
}
