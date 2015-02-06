package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.RemoveSingleDebrisBehavior;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.taskExecutor.Task;

public class RemovePieceOfDebrisTask implements Task
{
   private final RemoveSingleDebrisBehavior removeSingleDebrisBehavior;
   private final RigidBodyTransform debrisTransform;
   private final Point3d graspPosition;
   private final Vector3d graspVector;

   public RemovePieceOfDebrisTask(RemoveSingleDebrisBehavior removeSingleDebrisBehavior, RigidBodyTransform debrisTransform, Point3d graspPosition,
         Vector3d graspVector)
   {
      this.removeSingleDebrisBehavior = removeSingleDebrisBehavior;
      this.debrisTransform = debrisTransform;
      this.graspPosition = graspPosition;
      this.graspVector = graspVector;
   }

   @Override
   public void doTransitionIntoAction()
   {
      removeSingleDebrisBehavior.initialize();
      removeSingleDebrisBehavior.setInputs(debrisTransform, graspPosition, graspVector);
   }

   @Override
   public void doAction()
   {
      removeSingleDebrisBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      removeSingleDebrisBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return removeSingleDebrisBehavior.isDone();
   }
}
