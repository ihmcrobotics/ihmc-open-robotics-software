package us.ihmc.behaviors.sequence;

public class BehaviorActionStateBasics implements BehaviorActionState
{
   /** The action's unique ID. */
   private final long id;

   public BehaviorActionStateBasics()
   {
      // TODO: Make parameter
      id = BehaviorActionSequence.NEXT_ID.getAndIncrement();
   }

   @Override
   public long getID()
   {
      return id;
   }
}
