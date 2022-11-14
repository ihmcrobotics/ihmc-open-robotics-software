package us.ihmc.communication.property;

import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.thread.Throttler;

/**
 * ROS 2 enabled, synced, interprocess stored property set. It allows external process
 * to command parameter updates and gives a periodic status, so they can see the current
 * parameters.
 */
public class ROS2StoredPropertySet
{
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final StoredPropertySetROS2TopicPair topicPair;
   private final StoredPropertySetBasics storedPropertySet;
   private final StoredPropertySetROS2Input commandInput;
   private final Throttler parameterOutputThrottler = new Throttler();

   public ROS2StoredPropertySet(ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                                StoredPropertySetROS2TopicPair topicPair,
                                StoredPropertySetBasics storedPropertySet)
   {
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;
      this.topicPair = topicPair;
      this.storedPropertySet = storedPropertySet;
      commandInput = new StoredPropertySetROS2Input(ros2PublishSubscribeAPI, topicPair.getCommandTopic(), storedPropertySet);
   }

   public void update()
   {
      commandInput.setToAcceptUpdate(); // Always accept updates
      commandInput.update();

      // Heartbeat so remote UI tuners can stay up to date
      if (parameterOutputThrottler.run(1.0))
      {
         ros2PublishSubscribeAPI.publish(topicPair.getStatusTopic(), StoredPropertySetMessageTools.newMessage(storedPropertySet));
      }
   }

   public StoredPropertySetROS2Input getCommandInput()
   {
      return commandInput;
   }
}
