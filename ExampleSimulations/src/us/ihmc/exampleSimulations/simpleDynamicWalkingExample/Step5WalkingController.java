package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public class Step5WalkingController implements RobotController
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
   
   private DoubleYoVariable desiredBodyZ;
   private DoubleYoVariable desiredLegPitch;
   private DoubleYoVariable desiredKneeZ;
   private DoubleYoVariable desiredBodyPitch;
   private DoubleYoVariable desiredAnklePitch;

   private DoubleYoVariable kneeTau;
   private DoubleYoVariable hipTau;
   private DoubleYoVariable ankleTau;

   Point3d bodyPositionToPack = new Point3d(); //global so that it is created only once to avoid generating garbage
   private Quat4d rotationToPack = new Quat4d();
   private double ffComponent = -38.0 * 9.81;
   private Vector3d velocityToPack = new Vector3d();

   // State Machine
   private enum States
   {
      SUPPORT, TOE_OFF, SWING, STRAIGHTEN
   }

   // Controllers and Gains
   public Step5WalkingController(Step5IDandSCSRobot rob, String name, double deltaT)
   {
      this.rob = rob;
      this.name = name;
      this.deltaT = deltaT;

      controllerBodyZ = new PIDController("bodyZ", controllerRegistry);
      controllerBodyZ.setProportionalGain(50000.0);
      controllerBodyZ.setDerivativeGain(5000.0);
      desiredBodyZ = new DoubleYoVariable("desiredBodyZ", controllerRegistry);
      desiredBodyZ.set(1.2);

      controllerLegPitch = new PIDController("legPitch", controllerRegistry);
      controllerLegPitch.setProportionalGain(50000.0);
      controllerLegPitch.setDerivativeGain(5000.0);
      desiredLegPitch = new DoubleYoVariable("desiredLegPitch", controllerRegistry);
      desiredLegPitch.set(0.2);

      controllerKneeZ = new PIDController("kneeZ", controllerRegistry);
      controllerKneeZ.setProportionalGain(15000.0);
      controllerKneeZ.setDerivativeGain(2000.0);
      desiredKneeZ = new DoubleYoVariable("desiredKneeZ", controllerRegistry);
      desiredKneeZ.set(0.2);
      
      controllerAnkle = new PIDController("anklePitch", controllerRegistry);
      controllerAnkle.setProportionalGain(1000.0);
      controllerAnkle.setDerivativeGain(100.0);
      desiredAnklePitch = new DoubleYoVariable("desiredAnklePitch", controllerRegistry);
      desiredAnklePitch.set(0.0);
      
      desiredBodyPitch = new DoubleYoVariable("desiredBodyPitch", controllerRegistry);
      desiredBodyPitch.set(0.0);
      
      hipTau = new DoubleYoVariable("hipTau", controllerRegistry);
      kneeTau = new DoubleYoVariable("kneeTau", controllerRegistry);
      ankleTau = new DoubleYoVariable("ankleTau", controllerRegistry);
   }

   /////////////////////////////////////////////////////////////////////////////////////////////
   public void doControl()
   {
      rob.updateIDRobot();

      for (RobotSide robotSide : RobotSide.values())
      {
         /********** Body Height ****************/
         kneeTau.set(controllerBodyZ.compute(rob.getBodyPositionZ(), desiredBodyZ.getDoubleValue(), -rob.getKneeVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, -kneeTau.getDoubleValue()); // + ffComponent);
         
         /*********** Ankle Pitch **************/
         ankleTau.set(controllerAnkle.compute(rob.getAnklePitch(robotSide), desiredAnklePitch.getDoubleValue(), -rob.getAnkleVelocity(robotSide), 0.0, deltaT));
         rob.setKneeTau(robotSide, ankleTau.getDoubleValue());

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

         rob.applyTorques(); //Note: this is getting called twice, one per side
      }
   }

   /////////////////////////////////////////////////////////////////////////////////////////////

   /**
    * State Machine
    */

   //   private void setUpStateMachine()
   //   {
   //         // States and Actions:
   //         State<States> leftSupportState = new SupportState(RobotSide.LEFT, States.SUPPORT);
   //         State<States> rightSupportState = new SupportState(RobotSide.RIGHT, States.SUPPORT);  
   //         State<States> leftToeOffState = new ToeOffState(RobotSide.LEFT, States.TOE_OFF);
   //         State<States> rightToeOffState = new ToeOffState(RobotSide.RIGHT, States.TOE_OFF);
   //         State<States> leftSwingState = new SwingState(RobotSide.LEFT, States.SWING);
   //         State<States> rightSwingState = new SwingState(RobotSide.RIGHT, States.SWING);
   //         State<States> leftStraightenState = new StraightenState(RobotSide.LEFT, States.STRAIGHTEN);
   //         State<States> rightStraightenState = new StraightenState(RobotSide.RIGHT, States.STRAIGHTEN);
   //
   //         // Transition Conditions:
   //         StateTransitionCondition leftHealUnloaded = new HealUnloadedCondition(RobotSide.LEFT);
   //         StateTransitionCondition leftFootUnloaded = new FootUnloadedCondition(RobotSide.LEFT);
   //         StateTransitionCondition leftFootTouchedDown = new FootTouchedDownCondition(RobotSide.LEFT);
   //
   //         StateTransitionCondition rightHealUnloaded = new HealUnloadedCondition(RobotSide.RIGHT);
   //         StateTransitionCondition rightFootUnloaded = new FootUnloadedCondition(RobotSide.RIGHT);
   //         StateTransitionCondition rightFootTouchedDown = new FootTouchedDownCondition(RobotSide.RIGHT);
   //
   //         // Left State Transitions:
   //         StateTransition<States> leftSupportToToeOff = new StateTransition<States>(States.TOE_OFF, leftHealUnloaded);
   //         leftSupportToToeOff.addTimePassedCondition(min_support_time);
   //         leftSupportState.addStateTransition(leftSupportToToeOff);
   //
   //         StateTransition<States> leftToeOffToSwing = new StateTransition<States>(States.SWING, leftFootUnloaded);
   //         leftToeOffState.addStateTransition(leftToeOffToSwing);
   //
   //         StateTransition<States> leftSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swing_time);
   //         leftSwingState.addStateTransition(leftSwingToStraighten);
   //
   //         StateTransition<States> leftStraightenToSupport = new StateTransition<States>(States.SUPPORT, leftFootTouchedDown);
   //         leftStraightenState.addStateTransition(leftStraightenToSupport);
   //
   //         // Right State Transitions:
   //         StateTransition<States> rightSupportToToeOff = new StateTransition<States>(States.TOE_OFF, rightHealUnloaded);
   //         rightSupportToToeOff.addTimePassedCondition(min_support_time);
   //         rightSupportState.addStateTransition(rightSupportToToeOff);
   //
   //         StateTransition<States> rightToeOffToSwing = new StateTransition<States>(States.SWING, rightFootUnloaded);
   //         rightToeOffState.addStateTransition(rightToeOffToSwing);
   //
   //         StateTransition<States> rightSwingToStraighten = new StateTransition<States>(States.STRAIGHTEN, swing_time);
   //         rightSwingState.addStateTransition(rightSwingToStraighten);
   //
   //         StateTransition<States> rightStraightenToSupport = new StateTransition<States>(States.SUPPORT, rightFootTouchedDown);
   //         rightStraightenState.addStateTransition(rightStraightenToSupport);
   //
   //
   //         // Assemble the Left State Machine:
   //         leftStateMachine.addState(leftSupportState);
   //         leftStateMachine.addState(leftToeOffState);
   //         leftStateMachine.addState(leftSwingState);
   //         leftStateMachine.addState(leftStraightenState);
   //
   //
   //         // Assemble the Right State Machine:
   //         rightStateMachine.addState(rightSupportState);
   //         rightStateMachine.addState(rightToeOffState);
   //         rightStateMachine.addState(rightSwingState);
   //         rightStateMachine.addState(rightStraightenState);
   //
   //         // Set the Initial States:
   //
   //         leftStateMachine.setCurrentState(States.STRAIGHTEN);
   //         rightStateMachine.setCurrentState(States.SUPPORT);
   //   }
   //
   //  

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
