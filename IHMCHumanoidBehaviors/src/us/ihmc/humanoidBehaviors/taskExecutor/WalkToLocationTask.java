package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior.WalkingOrientation;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.robotics.geometry.FramePose2d;

public class WalkToLocationTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final FramePose2d targetPoseInWorld;
   private final WalkToLocationBehavior walkToLocationBehavior;

   private final double orientationRelativeToPathDirection;
   private final WalkingOrientation walkingOrientation;
   private double footstepLength;

   private double transferTime;
   private double swingTime;

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength, double swingTime, double transferTime)
   {
      this(null, targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength, swingTime, transferTime);
   }

   public WalkToLocationTask(E stateEnum, FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior,
         double orientationRelativeToPathDirection, double footstepLength, double swingTime, double transferTime)
   {
      super(stateEnum, walkToLocationBehavior);
      this.targetPoseInWorld = new FramePose2d(targetPoseInWorld);
      this.walkToLocationBehavior = walkToLocationBehavior;
      this.orientationRelativeToPathDirection = orientationRelativeToPathDirection;
      this.walkingOrientation = WalkingOrientation.CUSTOM;
      this.footstepLength = footstepLength;
      this.swingTime = swingTime;
      this.transferTime = transferTime;
   }

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, double orientationRelativeToPathDirection,
         double footstepLength)
   {
      this(null, targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength);
   }

   public WalkToLocationTask(E stateEnum, FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior,
         double orientationRelativeToPathDirection, double footstepLength)
   {
      this(stateEnum, targetPoseInWorld, walkToLocationBehavior, orientationRelativeToPathDirection, footstepLength, Double.NaN, Double.NaN);
   }

   public WalkToLocationTask(FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, WalkingOrientation walkingOrientation,
         double footstepLength)
   {
      this(null, targetPoseInWorld, walkToLocationBehavior, walkingOrientation, footstepLength);
   }

   public WalkToLocationTask(E stateEnum, FramePose2d targetPoseInWorld, WalkToLocationBehavior walkToLocationBehavior, WalkingOrientation walkingOrientation,
         double footstepLength)
   {
      super(stateEnum, walkToLocationBehavior);
      this.targetPoseInWorld = new FramePose2d(targetPoseInWorld);
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
