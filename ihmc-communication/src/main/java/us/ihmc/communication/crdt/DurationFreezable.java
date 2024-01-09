package us.ihmc.communication.crdt;

import us.ihmc.commons.Conversions;

/**
 * Implementation of {@link Freezable} that stays frozen for a predefined
 * period of time and then unfreezes.
 */
public class DurationFreezable implements Freezable
{
   /**
    * Certain changes to this node will cause a freeze of that data
    * from being modified from incoming messages.
    * Things are usually being synced at 30 Hz or faster, so 1 second
    * should allow plenty of time for changes to propagate.
    */
   public static final double FREEZE_DURATION_ON_MODIFICATION = 1.0;
   /**
    * This time is used as a time of modification of this node's
    * data so it won't accept updates from other sources for a short period of time.
    * This is to allow the changes to propagate elsewhere.
    *
    * Setting it to MIN_VALUE effectively unfreezes the node.
    */
   private double freezeTime = Double.MIN_VALUE;

   @Override
   public void freeze()
   {
      freezeTime = Conversions.nanosecondsToSeconds(System.nanoTime());
   }

   @Override
   public boolean isFrozen()
   {
      double now = Conversions.nanosecondsToSeconds(System.nanoTime());

      return now - freezeTime < FREEZE_DURATION_ON_MODIFICATION;
   }

   @Override
   public void unfreeze()
   {
      freezeTime = Double.MIN_VALUE;
   }
}
