package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.taskExecutor.Task;

public class WalkToLocationTask implements Task
{
   private static final boolean DEBUG = false;
   
   private final FramePose2d targetPoseInWorld;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final double walkingYawOrientationAngle;
   private double footstepLength;

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double walkingYawOrientationAngle,
         double footstepLength)
   {
      this.targetPoseInWorld = new FramePose2d(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.walkingYawOrientationAngle = walkingYawOrientationAngle;
      this.footstepLength = footstepLength;
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("entering walkToLocationTask");
      walkToLocationBehavior.initialize();
      walkToLocationBehavior.setTarget(targetPoseInWorld);
      walkToLocationBehavior.setwalkingYawOrientationAngle(walkingYawOrientationAngle);
      walkToLocationBehavior.setFootstepLength(footstepLength);
   }

   @Override
   public void doAction()
   {
      walkToLocationBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("exiting walkToLocationTask");
      walkToLocationBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return walkToLocationBehavior.isDone();
   }

}
