package us.ihmc.commonWalkingControlModules.desiredFootStep;

public interface AbortWalkingProvider
{
   public boolean shouldAbortWalking();

   public void walkingAborted();

   public void triggerAbort();
}
