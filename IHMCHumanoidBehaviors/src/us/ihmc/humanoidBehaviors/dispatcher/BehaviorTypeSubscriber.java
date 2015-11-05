package us.ihmc.humanoidBehaviors.dispatcher;

public interface BehaviorTypeSubscriber<E extends Enum<E>>
{
   boolean checkForNewBehaviorRequested();
   E getRequestedBehavior();
}