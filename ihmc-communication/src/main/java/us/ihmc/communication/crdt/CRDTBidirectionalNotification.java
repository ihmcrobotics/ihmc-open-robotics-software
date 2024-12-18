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
   private final LatestTimestampModifiable latestTimestampModifiable;

   private boolean isSet = false;

   public CRDTBidirectionalNotification(LatestTimestampModifiable latestTimestampModifiable)
   {
      this.latestTimestampModifiable = latestTimestampModifiable;
   }

   public boolean poll()
   {
      boolean wasSet = isSet;

      if (wasSet)
      {
         isSet = false;
         latestTimestampModifiable.modify();
         LogTools.debug(1, "Polled. Actor: %s".formatted(latestTimestampModifiable.getCRDTInfo().getActorDesignation()));
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
         LogTools.debug(1, "Setting. Actor: %s".formatted(latestTimestampModifiable.getCRDTInfo().getActorDesignation()));

         isSet = true;
         latestTimestampModifiable.modify();
      }
   }

   public boolean toMessage()
   {
      return isSet;
   }

   public void fromMessage(boolean isSet)
   {
      if (latestTimestampModifiable.isOutOfDate())
      {
         if (isSet != this.isSet)
            LogTools.debug("%b -> %b Actor: %s".formatted(this.isSet, isSet, latestTimestampModifiable.getCRDTInfo().getActorDesignation()));

         this.isSet = isSet;
      }
   }
}
