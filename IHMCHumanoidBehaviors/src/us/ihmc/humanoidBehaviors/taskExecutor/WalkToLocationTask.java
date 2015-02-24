package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WalkToLocationTask extends BehaviorTask
{
   private static final boolean DEBUG = false;

   private final FramePose2d targetPoseInWorld;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final double orientationRelativeToPathDirection;
   private double footstepLength;

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength, DoubleYoVariable yoTime, double sleepTime)
   {
      super(walkToLocationBehavior, yoTime, sleepTime);
      this.targetPoseInWorld = new FramePose2d(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.orientationRelativeToPathDirection = orientationRelativeToPathDirection;
      this.footstepLength = footstepLength;
   }

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength, DoubleYoVariable yoTime)
   {
      this(targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength, yoTime, 0.0);
   }

   @Override
   protected void setBehaviorInput()
   {
      walkToLocationBehavior.setWalkingOrientationRelativeToPathDirection(orientationRelativeToPathDirection);
      walkToLocationBehavior.setFootstepLength(footstepLength);
      walkToLocationBehavior.setTarget(targetPoseInWorld);
   }
}
