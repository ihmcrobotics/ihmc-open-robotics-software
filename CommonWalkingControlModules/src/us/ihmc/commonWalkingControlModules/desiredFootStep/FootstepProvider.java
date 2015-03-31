package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;

public interface FootstepProvider
{
   public abstract Footstep poll();

   public abstract Footstep peek();

   public abstract Footstep peekPeek();

   public abstract boolean isEmpty();

   public abstract void notifyComplete(FramePose footPoseInWorld);

   public abstract void notifyWalkingComplete();
   
   public abstract int getNumberOfFootstepsToProvide();
   
   public abstract boolean isBlindWalking();
   
   public abstract boolean isPaused();

   public abstract void cancelPlan();
}
