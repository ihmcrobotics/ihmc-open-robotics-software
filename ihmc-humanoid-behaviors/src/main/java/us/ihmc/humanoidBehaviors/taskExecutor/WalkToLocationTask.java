package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior.WalkingOrientation;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class WalkToLocationTask extends BehaviorAction
{
   private final FramePose2D targetPoseInWorld;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final double orientationRelativeToPathDirection;
   private final WalkingOrientation walkingOrientation;
   private double footstepLength;

   private double transferTime;
   private double swingTime;

   public WalkToLocationTask(FramePose2D targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
                             double footstepLength, double swingTime, double transferTime)
   {
      super(walkToLocationBehavior);
      this.targetPoseInWorld = new FramePose2D(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.orientationRelativeToPathDirection = orientationRelativeToPathDirection;
      this.walkingOrientation = WalkingOrientation.CUSTOM;
      this.footstepLength = footstepLength;
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }

   public WalkToLocationTask(FramePose2D targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
                             double footstepLength)
   {
      this(targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength, Double.NaN, Double.NaN);
   }

   public WalkToLocationTask(FramePose2D targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, WalkingOrientation walkingOrientation,
                             double footstepLength)
   {
      super(walkToLocationBehavior);
      this.targetPoseInWorld = new FramePose2D(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.orientationRelativeToPathDirection = 0.0;
      this.walkingOrientation = walkingOrientation;
      this.footstepLength = footstepLength;
   }

   @Override
   protected void setBehaviorInput()
   {
      if (walkingOrientation.equals(WalkingOrientation.CUSTOM))
      {
         walkToLocationBehavior.setWalkingOrientationRelativeToPathDirection(orientationRelativeToPathDirection);
      }
      walkToLocationBehavior.setFootstepLength(footstepLength);
      if (!Double.isNaN(swingTime))
      {
         walkToLocationBehavior.setSwingTime(swingTime);
      }
      if (!Double.isNaN(transferTime))
      {
         walkToLocationBehavior.setTransferTime(transferTime);
      }
      walkToLocationBehavior.setTarget(targetPoseInWorld, walkingOrientation);
   }
}
