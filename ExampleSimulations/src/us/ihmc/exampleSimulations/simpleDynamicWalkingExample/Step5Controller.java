package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;

public class Step5Controller implements RobotController
{

   //Variables
   private Step5IDandSCSRobot rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");
   private double deltaT;

   private PIDController controllerBodyZ;
   private PIDController controllerLegPitch;
   private PIDController controllerKneeZ;
   private PIDController controllerAnkle;
   
   private YoDouble desiredBodyZ;
   private YoDouble desiredLegPitch;
   private YoDouble desiredKneeZ;
   private YoDouble desiredBodyPitch;
   private YoDouble desiredAnklePitch;

   private YoDouble kneeTau;
   private YoDouble hipTau;
   private YoDouble ankleTau;

   Point3D bodyPositionToPack = new Point3D(); //global so that it is created only once to avoid generating garbage
   private Quaternion rotationToPack = new Quaternion();
   private double ffComponent = -38.0 * 9.81;
   private Vector3D velocityToPack = new Vector3D();

   
   // Controllers and Gains
   public Step5Controller(Step5IDandSCSRobot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(50000.0);
      controllerBodyZ.setDerivativeGain(5000.0);
      desiredBodyZ = new YoDouble("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.2);

      controllerLegPitch = new PIDController("legPitch", controllerRegistry);
      controllerLegPitch.setProportionalGain(50000.0);
      controllerLegPitch.setDerivativeGain(5000.0);
      desiredLegPitch = new YoDouble("desiredLegPitch", controllerRegistry);
      desiredLegPitch.set(0.2);

      controllerKneeZ = new PIDController("kneeZ", controllerRegistry);
      controllerKneeZ.setProportionalGain(15000.0);
      controllerKneeZ.setDerivativeGain(2000.0);
      desiredKneeZ = new YoDouble("desiredKneeZ", controllerRegistry);
      desiredKneeZ.set(0.2);
      
      controllerAnkle = new PIDController("anklePitch", controllerRegistry);
      controllerAnkle.setProportionalGain(1000.0);
      controllerAnkle.setDerivativeGain(100.0);
      desiredAnklePitch = new YoDouble("desiredAnklePitch", controllerRegistry);
      desiredAnklePitch.set(0.0);
      
      desiredBodyPitch = new YoDouble("desiredBodyPitch", controllerRegistry);
      desiredBodyPitch.set(0.0);
      
      hipTau = new YoDouble("hipTau", controllerRegistry);
      kneeTau = new YoDouble("kneeTau", controllerRegistry);
      ankleTau = new YoDouble("ankleTau", controllerRegistry);
   }

   /////////////////////////////////////////////////////////////////////////////////////////////
   public void doControl()
   {
      rob.updatePositionsIDrobot();

      for (RobotSide robotSide : RobotSide.values())
      {
         /********** Body Height ****************/
         kneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue()); // + ffComponent);
         
         /*********** Ankle Pitch **************/
         ankleTau.set(controllerAnkle.compute(rob.getAnklePitch(robotSide), desiredAnklePitch.getDoubleValue(), -rob.getAnkleVelocity(robotSide), 0.0, deltaT));
         rob.setAnkleTau(robotSide, ankleTau.getDoubleValue());

    /**
     * 1
     */
         
//         if (robotSide == RobotSide.RIGHT)
//         {
//
//            /********** Body Height (RIGHT KNEE) ****************/
//            kneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
//            rob.setKneeTau(robotSide, -kneeTau.getDoubleValue()); // + ffComponent);
//
//            /********** Body Pitch (RIGHT HIP) ****************/
//            rob.getBodyPitch(rotationToPack);
//            double pitchFromQuaternion = RotationFunctions.getPitchFromQuaternion(rotationToPack);
//            
//            rob.getBodyAngularVel(velocityToPack);
//            double bodyAngularVel = velocityToPack.getY(); //pitch velocity
//            
//            hipTau.set(controllerLegPitch.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), bodyAngularVel, 0.0, deltaT));
//            rob.setHipTau(robotSide, -hipTau.getDoubleValue());
//         }
//
//         else
//         {
//            /********** Knee Height (LEFT KNEE) ****************/
//            kneeTau.set(controllerKneeZ.compute(rob.getKneePositionZ(robotSide), desiredKneeZ.getDoubleValue(), rob.getKneeVelocity(robotSide), 0.0, deltaT));
//            rob.setKneeTau(robotSide, kneeTau.getDoubleValue());
//
//            /********** Right Leg Pitch (LEFT HIP)************/
//            hipTau.set(controllerLegPitch.compute(rob.getHipPitch(robotSide), desiredLegPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
//            rob.setHipTau(robotSide, hipTau.getDoubleValue());
//         }

    /**
     * 2     
     */
         
         //         /********** Body height ****************/
         //         rob.getBodyPoint(bodyPositionToPack);
         //         kneeTau.set(controllerBodyZ.compute(bodyPositionToPack.getZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
         //         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue());
         //
         //         if (robotSide == RobotSide.LEFT)
         //         {
         //            /********** Body Pitch ************/
         //            rob.getBodyPitch(rotationToPack);
         //            double pitchFromQuaternion = RotationFunctions.getPitchFromQuaternion(rotationToPack);
         //            hipTau.set(controllerLegPitch.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         //            rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //         }
         //         else
         //         {
         //            /********** Body Pitch ************/
         //            rob.getBodyPitch(rotationToPack);
         //            double pitchFromQuaternion = RotationFunctions.getPitchFromQuaternion(rotationToPack);
         //            hipTau.set(controllerLegPitch.compute(pitchFromQuaternion, desiredBodyPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         //            rob.setHipTau(robotSide, -hipTau.getDoubleValue());
         //         }

         /**
          * 2.b
          */
         
         
         //         if (robotSide == RobotSide.LEFT)
         //         {
         //         /********** Leg Swing ************/
         //         hipTau.set(controllerLegPitch.compute(rob.getHipPitch(robotSide), -desiredLegPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         //         rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //         }
         //         else
         //         {
         //         /********** Leg Swing ************/
         //         hipTau.set(controllerLegPitch.compute(rob.getHipPitch(robotSide), desiredLegPitch.getDoubleValue(), rob.getHipVelocity(robotSide), 0.0, deltaT));
         //         rob.setHipTau(robotSide, hipTau.getDoubleValue());
         //         }

         rob.updateTorquesSCSrobot(); //Note: this is getting called twice, one per side
      }
   }

   /////////////////////////////////////////////////////////////////////////////////////////////

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return controllerRegistry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

}
