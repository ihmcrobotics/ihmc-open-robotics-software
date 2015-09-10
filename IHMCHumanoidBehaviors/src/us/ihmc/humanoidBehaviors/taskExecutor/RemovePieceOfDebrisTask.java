package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.RemoveSingleDebrisBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class RemovePieceOfDebrisTask extends BehaviorTask
{
   private final RemoveSingleDebrisBehavior removeSingleDebrisBehavior;
   private final RigidBodyTransform debrisTransform;
   private final Point3d graspPosition;
   private final Vector3d graspVector;

   public RemovePieceOfDebrisTask(RemoveSingleDebrisBehavior removeSingleDebrisBehavior, RigidBodyTransform debrisTransform, Point3d graspPosition,
         Vector3d graspVector, DoubleYoVariable yoTime)
   {
      super(removeSingleDebrisBehavior, yoTime);
      this.removeSingleDebrisBehavior = removeSingleDebrisBehavior;
      this.debrisTransform = debrisTransform;
      this.graspPosition = graspPosition;
      this.graspVector = graspVector;
   }

   @Override
   protected void setBehaviorInput()
   {
      removeSingleDebrisBehavior.setInputs(debrisTransform, graspPosition, graspVector);
   }
}
