package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior.ValveGraspLocation;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveTask extends BehaviorTask
{
   private final GraspValveBehavior graspValveBehavior;
   private final RigidBodyTransform valveTransformToWorld;
   private final Vector3d approachDirection;
   private final double valveRadius;
   private final boolean graspValveRim;

   private final boolean DEBUG = false;

   public GraspValveTask(GraspValveBehavior graspValveBehavior, RigidBodyTransform valveTransformToWorld, Vector3d approachDirection,
         boolean graspValveRim, double valveRadius, DoubleYoVariable yoTime)
   {
      super(graspValveBehavior, yoTime);
      this.graspValveBehavior = graspValveBehavior;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.approachDirection = new Vector3d(approachDirection);
      this.valveRadius = valveRadius;
      this.graspValveRim = graspValveRim;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveBehavior.setGraspPose(RobotSide.RIGHT, valveTransformToWorld, valveRadius, ValveGraspLocation.TWELVE_O_CLOCK);
   }
}
