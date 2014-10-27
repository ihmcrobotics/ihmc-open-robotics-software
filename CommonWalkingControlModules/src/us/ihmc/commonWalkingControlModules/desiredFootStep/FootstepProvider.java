package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

public interface FootstepProvider
{
   public abstract Footstep poll();

   public abstract Footstep peek();

   public abstract Footstep peekPeek();

   public abstract boolean isEmpty();

   public abstract void notifyComplete();

   public abstract void notifyWalkingComplete();
   
   public abstract int getNumberOfFootstepsToProvide();
   
   public abstract boolean isBlindWalking();
}
