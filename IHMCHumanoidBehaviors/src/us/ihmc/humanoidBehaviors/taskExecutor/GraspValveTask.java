package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveTurnDirection;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveTask extends BehaviorTask
{
   private final GraspValveBehavior graspValveBehavior;
   private final RigidBodyTransform valveTransformToWorld;
   private final ValveGraspLocation valveGraspLocation;
   private final ValveTurnDirection valveTurnDirection;
   private final double graspApproachConeAngle;
   private final Axis valvePinJointAxisInValveFrame;
   private final double valveRadius;

   private final boolean DEBUG = false;

   public GraspValveTask(GraspValveBehavior graspValveBehavior, RigidBodyTransform valveTransformToWorld, ValveGraspLocation valveGraspLocation,
         ValveTurnDirection valveTurnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame, double valveRadius, DoubleYoVariable yoTime)
   {
      super(graspValveBehavior, yoTime);
      this.graspValveBehavior = graspValveBehavior;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.graspApproachConeAngle = graspApproachConeAngle;
      this.valvePinJointAxisInValveFrame = valvePinJointAxisInValveFrame;
      this.valveGraspLocation = valveGraspLocation;
      this.valveTurnDirection = valveTurnDirection;
      this.valveRadius = valveRadius;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveBehavior.setGraspPose(RobotSide.RIGHT, valveTransformToWorld, valveRadius, valveGraspLocation, valveTurnDirection, graspApproachConeAngle,
            valvePinJointAxisInValveFrame);
   }
}
