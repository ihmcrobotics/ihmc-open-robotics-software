package us.ihmc.behaviors.sequence;

public class BehaviorActionStateBasics
{
   /** The action's unique ID. */
   private final long id;

   public BehaviorActionStateBasics()
   {
      // TODO: Make parameter
      id = BehaviorActionSequence.NEXT_ID.getAndIncrement();
   }

   public long getID()
   {
      return id;
   }
}
