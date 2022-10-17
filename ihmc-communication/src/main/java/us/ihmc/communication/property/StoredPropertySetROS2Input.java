package us.ihmc.communication.property;

import ihmc_common_msgs.msg.dds.StoredPropertySetMessage;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class StoredPropertySetROS2Input
{
   private final TypedNotification<StoredPropertySetMessage> receptionNotification = new TypedNotification<>();
   private boolean waitingForUpdate = true;
   private StoredPropertySetBasics storedPropertySetToUpdate;

   public StoredPropertySetROS2Input(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                     ROS2Topic<StoredPropertySetMessage> inputTopic,
                                     StoredPropertySetBasics storedPropertySetToUpdate)
   {
      this.storedPropertySetToUpdate = storedPropertySetToUpdate;
      ros2PublishSubscribeAPI.subscribeViaCallback(inputTopic, receptionNotification::set);
   }

   /**
    * On next update allow the stored properties to be updated by the latest recieved ROS 2 message.
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
      if (waitingForUpdate && receptionNotification.poll())
      {
         StoredPropertySetMessageTools.copyToStoredPropertySet(receptionNotification.read(),
                                                               storedPropertySetToUpdate,
                                                               () -> LogTools.info("Accepting property set update for {}",
                                                                                   storedPropertySetToUpdate.getCapitalizedClassName()));
         waitingForUpdate = false;
         return true;
      }
      return false;
   }

   public boolean getWaitingForUpdate()
   {
      return waitingForUpdate;
   }
}
