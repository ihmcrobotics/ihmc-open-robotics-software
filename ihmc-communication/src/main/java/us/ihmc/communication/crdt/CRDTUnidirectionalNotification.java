package us.ihmc.communication.crdt;

import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;

/**
 * Represents a notification that can be set by one actor and polled by the other.
 */
public class CRDTUnidirectionalNotification
{
   private final ROS2ActorDesignation sideThatCanSet;
   private final CRDTInfo crdtInfo;
   private final RequestConfirmFreezable requestConfirmFreezable;

   private boolean isSet = false;

   public CRDTUnidirectionalNotification(ROS2ActorDesignation sideThatCanSet, CRDTInfo crdtInfo, RequestConfirmFreezable requestConfirmFreezable)
   {
      this.sideThatCanSet = sideThatCanSet;
      this.crdtInfo = crdtInfo;
      this.requestConfirmFreezable = requestConfirmFreezable;
   }

   public boolean poll()
   {
      if (sideThatCanSet == crdtInfo.getActorDesignation())
         throw new RuntimeException("%s is not allowed to poll this notification.".formatted(crdtInfo.getActorDesignation()));

      boolean wasSet = isSet;

      if (wasSet)
      {
         LogTools.debug("%s Update #%d: Poll.".formatted(crdtInfo.getActorDesignation(), crdtInfo.getUpdateNumber()));
         requestConfirmFreezable.freeze();
         isSet = false;
      }

      return wasSet;
   }

   public boolean peek()
   {
      return isSet;
   }

   public void set()
   {
      if (sideThatCanSet != crdtInfo.getActorDesignation())
         throw new RuntimeException("%s is not allowed to set this notification.".formatted(crdtInfo.getActorDesignation()));

      if (!isSet)
      {
         LogTools.debug("%s Update #%d: Set.".formatted(crdtInfo.getActorDesignation(), crdtInfo.getUpdateNumber()));
         requestConfirmFreezable.freeze();
         isSet = true;
      }
   }

   public boolean toMessage()
   {
      return isSet;
   }

   public void fromMessage(boolean isSet)
   {
      if (isSet != this.isSet && !requestConfirmFreezable.isFrozen())
      {
         boolean pollConfirmed = !isSet && sideThatCanSet == crdtInfo.getActorDesignation();
         boolean setRequested = isSet && sideThatCanSet != crdtInfo.getActorDesignation();
         if (pollConfirmed || setRequested)
         {
            LogTools.debug("%s Update #%d: %b -> %b".formatted(crdtInfo.getActorDesignation(),
                                                               crdtInfo.getUpdateNumber(),
                                                               this.isSet,
                                                               isSet));
            this.isSet = isSet;
         }
      }
   }
}