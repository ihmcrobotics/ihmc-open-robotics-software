package us.ihmc.communication.crdt;

import us.ihmc.log.LogTools;

/**
 * Represents a notification that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalNotification
{
   private final Confirmable confirmable;

   private boolean isSet = false;

   public CRDTBidirectionalNotification(Confirmable confirmable)
   {
      this.confirmable = confirmable;
   }

   public boolean poll()
   {
      boolean wasSet = isSet;

      if (wasSet)
      {
         isSet = false;
         confirmable.freeze();
         LogTools.info("POLLED");
      }

      return wasSet;
   }

   public boolean peek()
   {
      return isSet;
   }

   public void set()
   {
      if (!isSet)
      {
         LogTools.info("SETTING");

         isSet = true;
         confirmable.freeze();
      }
   }

   public void fromMessage(boolean isSet)
   {
      if (!confirmable.isFrozen())
      {
         if (isSet)
            LogTools.info("FROM MESSAGE SET Actor: %s".formatted(confirmable.getCRDTInfo().getActorDesignation()));
         this.isSet = isSet;
      }
   }

   public boolean toMessage()
   {
      return isSet;
   }
}
