package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.GraspTurnAndUnGraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspTurnAndUnGraspValveTask extends BehaviorTask
{
   private final GraspTurnAndUnGraspValveBehavior graspValveTurnAndUnGraspBehavior;
   private final RigidBodyTransform valveTransformToWorld;
   private final Axis valvePinJointAxisInValveFrame;
   private final double valveRadius;
   private final double turnValveAngle;

   private final ValveGraspLocation valveGraspLocation;
   private final double graspApproachConeAngle;

   private final boolean DEBUG = false;

   public GraspTurnAndUnGraspValveTask(GraspTurnAndUnGraspValveBehavior graspTurnAndUnGraspValveBehavior, RigidBodyTransform valveTransformToWorld,
         ValveGraspLocation valveGraspLocation, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame,
         double valveRadius, double turnValveAngle, DoubleYoVariable yoTime)
   {
      super(graspTurnAndUnGraspValveBehavior, yoTime);
      this.graspValveTurnAndUnGraspBehavior = graspTurnAndUnGraspValveBehavior;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.valveGraspLocation = valveGraspLocation;
      this.graspApproachConeAngle = graspApproachConeAngle;
      this.valvePinJointAxisInValveFrame = valvePinJointAxisInValveFrame;
      this.valveRadius = valveRadius;
      this.turnValveAngle = turnValveAngle;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveTurnAndUnGraspBehavior.setInput(valveTransformToWorld, valveGraspLocation, graspApproachConeAngle,
            valvePinJointAxisInValveFrame, valveRadius, turnValveAngle);
   }
}
