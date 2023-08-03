package us.ihmc.communication.property;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Subscribes to stored property set messages and buffers the reception, so we
 * don't get parameters changing in the middle of computation.
 */
public class StoredPropertySetROS2Input
{
   private final TypedNotification<StoredPropertySetMessage> receptionNotification = new TypedNotification<>();
   private boolean waitingForUpdate = true;
   private boolean isUpdateAvailable = false;
   private final StoredPropertySetBasics storedPropertySetToUpdate;
   private final AtomicBoolean anyValuesChanged = new AtomicBoolean(false);
   private record PropertyChangeNotification(StoredPropertyKey<?> propertyKey,
                                             Notification notification,
                                             MutableObject<Object> previousValue) { }
   private final ArrayList<PropertyChangeNotification> propertyChangeNotifications = new ArrayList<>();
   private final double expirationDuration = ROS2StoredPropertySet.STATUS_PERIOD + ROS2StoredPropertySet.STATUS_PERIOD * 0.5; // give it a little extra
   private final Timer receptionTimer = new Timer();
   private long numberOfMessagesReceived = 0;

   public StoredPropertySetROS2Input(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2Topic<StoredPropertySetMessage> inputTopic,
                                     StoredPropertySetBasics storedPropertySetToUpdate)
   {
      this.storedPropertySetToUpdate = storedPropertySetToUpdate;
      ros2PublishSubscribeAPI.subscribeViaCallback(inputTopic, this::acceptMessage);
   }

   private void acceptMessage(StoredPropertySetMessage message)
   {
      receptionNotification.set(message);
      ++numberOfMessagesReceived;
      receptionTimer.reset();
   }

   /**
    * On next update allow the stored properties to be updated by the latest recieved ROS 2 message.
    * This is so a UI button can queue up an update and the update can happen at the appropriate time.
    */
   public void setToAcceptUpdate()
   {
      waitingForUpdate = true;
   }

   /**
    * Handles incoming updates as allowed by setToAcceptUpdate().
    * @return if the stored property set was updated with any non-equal values
    */
   public boolean update()
   {
      anyValuesChanged.set(false);

      for (PropertyChangeNotification propertyChangeNotification : propertyChangeNotifications)
      {
         propertyChangeNotification.previousValue().setValue(storedPropertySetToUpdate.get(propertyChangeNotification.propertyKey()));
      }

      if (receptionNotification.peekHasValue())
      {
         isUpdateAvailable = !StoredPropertySetMessageTools.valuesAreAllEqual(receptionNotification.peek(), storedPropertySetToUpdate);
      }

      if (waitingForUpdate && receptionNotification.poll())
      {
         waitingForUpdate = false;
         isUpdateAvailable = false;
         StoredPropertySetMessageTools.copyToStoredPropertySet(receptionNotification.read(),
                                                               storedPropertySetToUpdate,
                                                               () ->
         {
            LogTools.info("Accepting property set update for {}", storedPropertySetToUpdate.getTitle());
            anyValuesChanged.set(true);

            for (PropertyChangeNotification propertyChangeNotification : propertyChangeNotifications)
            {
               Object previousValue = propertyChangeNotification.previousValue().getValue();
               Object newValue = storedPropertySetToUpdate.get(propertyChangeNotification.propertyKey());
               if (!previousValue.equals(newValue))
               {
                  LogTools.info("{} changed. {} -> {}", propertyChangeNotification.propertyKey().getTitleCasedName(), previousValue, newValue);
                  propertyChangeNotification.notification().set();
               }
            }
         });
      }
      return anyValuesChanged.get();
   }

   public boolean getWaitingForUpdate()
   {
      return waitingForUpdate;
   }

   public boolean getUpdateAvailable()
   {
      return isUpdateAvailable;
   }

   public boolean getIsExpired()
   {
      return !receptionTimer.isRunning(expirationDuration);
   }

   public long getNumberOfMessagesReceived()
   {
      return numberOfMessagesReceived;
   }

   /**
    * This notification should be checked after a call to update().
    * The parameter set will have the new values when this is called.
    */
   public Notification registerPropertyChangedNotification(StoredPropertyKey<?> storedPropertyKey)
   {
      Notification notification = new Notification();
      propertyChangeNotifications.add(new PropertyChangeNotification(storedPropertyKey, notification, new MutableObject<>()));
      return notification;
   }
}
