package us.ihmc.communication.crdt;

import us.ihmc.tools.Timer;

/**
 * Part of a simple CRDT algorithm where something is frozen when modified
 * to allow the changes to propagate without being immediately overritten
 * with out of date data.
 */
public class Freezable
{
   /**
    * Certain changes to this node will cause a freeze of that data
    * from being modified from incoming messages.
    * Things are usually being synced at 30 Hz or faster, so 1 second
    * should allow plenty of time for changes to propagate.
    */
   public static final double FREEZE_DURATION_ON_MODIFICATION = 1.0;
   /**
    * This timer is used in the case that an operator can "mark modified" this node's
    * data so it won't accept updates from other sources for a short period of time.
    * This is to allow the changes to propagate elsewhere.
    */
   private final Timer modifiedTimer = new Timer();

   public void freezeFromModification()
   {
      modifiedTimer.reset();
   }

   public boolean isFrozenFromModification()
   {
      return modifiedTimer.isRunning(FREEZE_DURATION_ON_MODIFICATION);
   }
}
