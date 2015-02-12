package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WalkToLocationTask extends BehaviorTask
{
   private static final boolean DEBUG = false;

   private final FramePose2d targetPoseInWorld;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final double walkingYawOrientationAngle;
   private double footstepLength;

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double walkingYawOrientationAngle,
         double footstepLength, DoubleYoVariable yoTime)
   {
      super(walkToLocationBehavior, yoTime);
      this.targetPoseInWorld = new FramePose2d(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.walkingYawOrientationAngle = walkingYawOrientationAngle;
      this.footstepLength = footstepLength;
   }

   @Override
   protected void setBehaviorInput()
   {
      walkToLocationBehavior.setTarget(targetPoseInWorld);
      walkToLocationBehavior.setwalkingYawOrientationAngle(walkingYawOrientationAngle);
      walkToLocationBehavior.setFootstepLength(footstepLength);
   }
}
