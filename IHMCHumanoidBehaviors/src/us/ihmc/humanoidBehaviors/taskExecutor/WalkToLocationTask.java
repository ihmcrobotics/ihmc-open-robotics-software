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

   private double transferTime;
   private double swingTime;

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength, double swingTime, double transferTime, DoubleYoVariable yoTime, double sleepTime)
   {
      super(walkToLocationBehavior, yoTime, sleepTime);
      this.targetPoseInWorld = new FramePose2d(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.orientationRelativeToPathDirection = orientationRelativeToPathDirection;
      this.footstepLength = footstepLength;
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }
   
   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength, double swingTime, double transferTime, DoubleYoVariable yoTime)
   {
      this(targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength, swingTime, transferTime, yoTime, 0.0);
   }
   
   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength, DoubleYoVariable yoTime, double sleepTime)
   {
      this(targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength, Double.NaN, Double.NaN, yoTime, sleepTime);
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
      if(swingTime != Double.NaN)
         walkToLocationBehavior.setSwingTime(swingTime);
      if(transferTime != Double.NaN)
         walkToLocationBehavior.setTransferTime(transferTime);
      walkToLocationBehavior.setTarget(targetPoseInWorld);
   }
}
