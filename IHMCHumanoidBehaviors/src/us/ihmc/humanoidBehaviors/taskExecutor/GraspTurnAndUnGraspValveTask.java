package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.GraspTurnAndUnGraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior.ValveGraspMethod;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class GraspTurnAndUnGraspValveTask extends BehaviorTask
{
   private final GraspTurnAndUnGraspValveBehavior graspValveTurnAndUnGraspBehavior;
   private final RigidBodyTransform valveTransformToWorld;
   private final Axis valvePinJointAxisInValveFrame;
   private final double valveRadius;
   private final double turnValveAngle;
   private final double valveRotationRateRadPerSec;

   private final ValveGraspLocation graspLocation;
   private final ValveGraspMethod graspMethod;
   private final double graspApproachConeAngle;

   boolean stopHandIfGraspCollision;
   boolean stopHandIfTurnCollision;
   
   private final boolean DEBUG = false;

   public GraspTurnAndUnGraspValveTask(GraspTurnAndUnGraspValveBehavior graspTurnAndUnGraspValveBehavior, RigidBodyTransform valveTransformToWorld,
         ValveGraspLocation graspLocation, ValveGraspMethod graspMethod, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame,
         double valveRadius, double turnValveAngle, double valveRotationRateRadPerSec, boolean stopHandIfGraspCollision, boolean stopHandIfTurnCollision, DoubleYoVariable yoTime)
   {
      super(graspTurnAndUnGraspValveBehavior, yoTime);
      this.graspValveTurnAndUnGraspBehavior = graspTurnAndUnGraspValveBehavior;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.graspLocation = graspLocation;
      this.graspMethod = graspMethod;
      this.graspApproachConeAngle = graspApproachConeAngle;
      this.valvePinJointAxisInValveFrame = valvePinJointAxisInValveFrame;
      this.valveRadius = valveRadius;
      this.turnValveAngle = turnValveAngle;
      this.valveRotationRateRadPerSec = valveRotationRateRadPerSec;
      this.stopHandIfGraspCollision = stopHandIfGraspCollision;
      this.stopHandIfTurnCollision = stopHandIfTurnCollision;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveTurnAndUnGraspBehavior.setInput(valveTransformToWorld, graspLocation, graspMethod, graspApproachConeAngle,
              valvePinJointAxisInValveFrame, valveRadius, turnValveAngle, valveRotationRateRadPerSec, stopHandIfGraspCollision, stopHandIfTurnCollision);
   }
}
