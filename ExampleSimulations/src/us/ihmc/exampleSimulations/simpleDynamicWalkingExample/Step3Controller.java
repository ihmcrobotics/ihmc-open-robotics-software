package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;

public class Step3Controller implements RobotController
{

   /**
    * Variables
    */
   private Step3Robot rob;
   private String name;
   private YoVariableRegistry controllerRegistry = new YoVariableRegistry("controllerRegistry");

   private double deltaT;
   private PIDController controllerBodyZ;
   private PIDController controllerBodyPitch;
   private PIDController controllerLegPitch;
   private PIDController controllerKneeZ;

   private DoubleYoVariable desiredBodyZ;
   private DoubleYoVariable desiredBodyPitch;
   private DoubleYoVariable desiredLegPitch;
   private DoubleYoVariable desiredKneeZ;

   private DoubleYoVariable controlKneeTau;
   private DoubleYoVariable controlBodyTau;
   private DoubleYoVariable controlHipTau;
   private DoubleYoVariable controlKneeForce;

   private BooleanYoVariable onTheFloor;
   private DoubleYoVariable gcHeightRight;

   private double ffComponent = -38.0 * 9.81;

   /**
    * Constructor
    */
   public Step3Controller(Step3Robot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(2000.0);
      controllerBodyZ.setDerivativeGain(500.0);
      desiredBodyZ = new DoubleYoVariable("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.4); //larger than initial
      controlKneeTau = new DoubleYoVariable("controlKneeTau", controllerRegistry);

      controllerBodyPitch = new PIDController("bodyPitch", controllerRegistry);
      controllerBodyPitch.setProportionalGain(1e3);
      controllerBodyPitch.setDerivativeGain(1e2);
      desiredBodyPitch = new DoubleYoVariable("desiredBodyPitch", controllerRegistry);
      desiredBodyPitch.set(-0.2);
      controlBodyTau = new DoubleYoVariable("controlBodyTau", controllerRegistry);

      controllerLegPitch = new PIDController("legPitch", controllerRegistry);
      controllerLegPitch.setProportionalGain(50000.0);
      controllerLegPitch.setDerivativeGain(5000.0);
      desiredLegPitch = new DoubleYoVariable("desiredLegPitch", controllerRegistry);
      desiredLegPitch.set(0.4);
      controlHipTau = new DoubleYoVariable("controlLegTau", controllerRegistry);

      controllerKneeZ = new PIDController("kneeZ", controllerRegistry);
      controllerKneeZ.setProportionalGain(15000.0);
      controllerKneeZ.setDerivativeGain(2000.0);
      desiredKneeZ = new DoubleYoVariable("desiredKneeZ", controllerRegistry);
      desiredKneeZ.set(0.2);
      controlKneeForce = new DoubleYoVariable("controlKneeForce", controllerRegistry);

      onTheFloor = new BooleanYoVariable("onTheFloor", controllerRegistry);
      onTheFloor.set(true);

      gcHeightRight = new DoubleYoVariable("gcHeightRight", controllerRegistry);
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
   /**
    * Right leg is the support and is transparent
    * Right knee controls the body height 
    * Right hip controls body pitch
    *   
    * Left knee controls leg flexion 
    * Left hip controls leg pitch 
    *
    * The knee shouldn't try to pull down the body when it is in the air....simplemente no deber'ia de levantarse del suelo jajajaja
    **/

   {

      for (RobotSide robotSide : RobotSide.values())
      {

         if (robotSide == RobotSide.RIGHT)
         {

            /********** Body Height (RIGHT KNEE) ****************/
            //WORKS

            onTheFloor.set(rob.isInContactV2(robotSide)); // This is just a check point to make sure that the support leg (right) is on the floor, but it isn't used for calculations.

            //TODO BE CAREFULL WITH THE SIGNS!!!! the knee velocity has a minus sign because a negative velocity implies extension, which means that the body is going up. Therefore, the velocity of the body and the knee have opposite signs
            controlKneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocityZ(robotSide), 0.0, deltaT));
            rob.setKneeForce(robotSide, -controlKneeTau.getDoubleValue() + ffComponent);

            //The following lines do the same, but more developed so that it is easier to see the sign "issue":
            // controlKneeTau.set(-controllerBodyZ.getProportionalGain() * (desiredBodyZ.getDoubleValue() - rob.getBodyPositionZ())); //this line is causing the knee to retract when the body is in the air
            // controlKneeTau.add(controllerBodyZ.getDerivativeGain() * (0.0 - rob.getKneeVelocityZ(robotSide))); //TODO Tell Silvane that I was missing the 0.0 (desiredVel)
            // controlKneeTau.add(-27.0 * 9.81);

            /********** Body Pitch (RIGHT HIP) ****************/
            controlBodyTau.set(controllerBodyPitch.compute(rob.getBodyPitch(), desiredBodyPitch.getDoubleValue(), rob.getBodyPitchVel(), 0.0, deltaT));
            rob.setHipTau(robotSide, -controlBodyTau.getDoubleValue());

         }

         else
         {
            /********** Knee Height (LEFT KNEE) ****************/
            //WORKS
            controlKneeForce.set(controllerKneeZ.compute(rob.getKneePositionZ(robotSide), desiredKneeZ.getDoubleValue(), rob.getKneeVelocityZ(robotSide), 0.0,
                  deltaT));
            rob.setKneeForce(robotSide, controlKneeForce.getDoubleValue());

            /********** Right Leg Pitch (LEFT HIP)************/
            controlHipTau.set(controllerLegPitch.compute(rob.getLegPitch(robotSide), desiredLegPitch.getDoubleValue(), rob.getLegPitchVel(robotSide), 0.0,
                  deltaT));
            rob.setHipTau(robotSide, controlHipTau.getDoubleValue());
         }

      }

   }
}
