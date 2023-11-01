package us.ihmc.communication.crdt;

import ihmc_common_msgs.msg.dds.CRDTNotificationMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Timer;

/**
 * A way of sending a notification via CRDT synced state messages.
 * By using 3 values, we can set a notification and receive a
 * confirmation that the other side polled it without sync errors.
 */
public class CRDTNotification
{
   /** Reset if poll not confirmed in 1 second. */
   private static final double TIMEOUT = 1.0;

   /** The actor that is the one allowed to set the notification. */
   private final ROS2ActorDesignation sourceActor;
   /** The type of actor that holds this instance. */
   private final ROS2ActorDesignation thisActor;

   private boolean isSet = false;
   private final Notification locallySetOrPolled = new Notification();
   private final Timer resetTimer = new Timer();

   public CRDTNotification(ROS2ActorDesignation sourceActor, ROS2ActorDesignation thisActor)
   {
      this.sourceActor = sourceActor;
      this.thisActor = thisActor;
   }

   public void set()
   {
      if (thisActor != sourceActor)
         LogTools.error("Only the source actor may call this.");

      isSet = true;
      locallySetOrPolled.set();
      resetTimer.reset();
   }

   public boolean poll()
   {
      if (thisActor == sourceActor)
         LogTools.error("Only the polling actor may call this.");

      boolean wasSet = isSet;
      isSet = false;
      locallySetOrPolled.set();
      return wasSet;
   }

   public boolean peek()
   {
      return isSet;
   }

   public void toMessage(CRDTNotificationMessage message)
   {
      message.setValue(CRDTNotificationMessage.NOOP);

      if (locallySetOrPolled.poll())
      {
         if (thisActor == sourceActor)
         {
            message.setValue(CRDTNotificationMessage.SET_REQUEST);
         }
         else
         {
            message.setValue(CRDTNotificationMessage.POLL_CONFIRMED);
         }
      }
   }

   public void fromMessage(CRDTNotificationMessage message)
   {
      if (thisActor == sourceActor)
      {
         if (message.getValue() == CRDTNotificationMessage.SET_REQUEST)
         {
            // Another source actor set the notification so we update our state
            isSet = true;
            resetTimer.reset();
         }
      }
      else
      {
         if (message.getValue() == CRDTNotificationMessage.POLL_CONFIRMED)
         {
            isSet = false;
         }
      }

      // Avoiding getting stuck waiting for poll confirmation
      // This could happen in an update method but if it works, this is simpler.
      if (resetTimer.isExpired(TIMEOUT))
      {
         isSet = false;
      }
   }
}
