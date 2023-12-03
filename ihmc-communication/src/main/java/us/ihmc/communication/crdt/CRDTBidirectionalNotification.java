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
   private final RequestConfirmFreezable requestConfirmFreezable;

   private boolean isSet = false;

   public CRDTBidirectionalNotification(RequestConfirmFreezable requestConfirmFreezable)
   {
      this.requestConfirmFreezable = requestConfirmFreezable;
   }

   public boolean poll()
   {
      boolean wasSet = isSet;

      if (wasSet)
      {
         isSet = false;
         requestConfirmFreezable.freeze();
         LogTools.debug(1, "Polled. Actor: %s".formatted(requestConfirmFreezable.getCRDTInfo().getActorDesignation()));
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
         LogTools.debug(1, "Setting. Actor: %s".formatted(requestConfirmFreezable.getCRDTInfo().getActorDesignation()));

         isSet = true;
         requestConfirmFreezable.freeze();
      }
   }

   public boolean toMessage()
   {
      return isSet;
   }

   public void fromMessage(boolean isSet)
   {
      if (!requestConfirmFreezable.isFrozen())
      {
         if (isSet != this.isSet)
            LogTools.debug("%b -> %b Actor: %s".formatted(this.isSet, isSet, requestConfirmFreezable.getCRDTInfo().getActorDesignation()));

         this.isSet = isSet;
      }
   }
}
