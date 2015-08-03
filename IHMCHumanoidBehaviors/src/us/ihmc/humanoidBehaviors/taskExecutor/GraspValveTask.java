package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveTurnDirection;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior.ValveGraspMethod;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveTask extends BehaviorTask
{
   private final GraspValveBehavior graspValveBehavior;
   private final RigidBodyTransform valveTransformToWorld;
   private final ValveGraspLocation graspLocation;
   private final ValveGraspMethod graspMethod;
   private final ValveTurnDirection turnDirection;
   private final double graspApproachConeAngle;
   private final Axis valvePinJointAxisInValveFrame;
   private final double valveRadius;
   private final boolean stopHandIfCollision;

   private final boolean DEBUG = false;

   public GraspValveTask(GraspValveBehavior graspValveBehavior, RigidBodyTransform valveTransformToWorld, ValveGraspLocation graspLocation,  ValveGraspMethod graspMethod,
	         ValveTurnDirection turnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame, double valveRadius, boolean stopHandIfCollision, DoubleYoVariable yoTime)
   {
      super(graspValveBehavior, yoTime);
      this.graspValveBehavior = graspValveBehavior;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.graspApproachConeAngle = graspApproachConeAngle;
      this.valvePinJointAxisInValveFrame = valvePinJointAxisInValveFrame;
      this.graspLocation = graspLocation;
      this.graspMethod = graspMethod;
      this.turnDirection = turnDirection;
      this.valveRadius = valveRadius;
      this.stopHandIfCollision = stopHandIfCollision;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveBehavior.setGraspPose(RobotSide.RIGHT, valveTransformToWorld, valveRadius, graspLocation, graspMethod, turnDirection, graspApproachConeAngle,
              valvePinJointAxisInValveFrame, stopHandIfCollision);
   }
}
