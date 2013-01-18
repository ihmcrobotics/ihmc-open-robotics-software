package us.ihmc.commonWalkingControlModules.desiredFootStep;

public interface FootstepProvider
{
   public abstract Footstep poll();

   public abstract boolean isEmpty();

   public void notifyComplete();
}
