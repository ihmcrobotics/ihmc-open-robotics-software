package us.ihmc.commonWalkingControlModules.desiredFootStep;

public interface FootstepProvider
{
   public abstract Footstep poll();

   public abstract Footstep peek();

   public abstract Footstep peekPeek();

   public abstract boolean isEmpty();

   public abstract void notifyComplete();
   
   public abstract int getNumberOfFootstepsToProvide();
   
   public abstract boolean isBlindWalking();
}
