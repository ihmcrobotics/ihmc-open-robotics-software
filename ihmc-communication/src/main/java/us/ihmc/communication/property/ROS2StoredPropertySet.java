package us.ihmc.communication.property;

import ihmc_common_msgs.PrimitiveDataVectorMessage;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * ROS 2 enabled, synced, interprocess stored property set. It allows external process
 * to command parameter updates and gives a periodic status, so they can see the current
 * parameters.
 */
public class ROS2StoredPropertySet<T extends StoredPropertySetBasics>
{
   public static final double STATUS_PERIOD = 1.0;

   private final ROS2Node ros2Node;
   private final StoredPropertySetROS2TopicPair topicPair;
   private final T storedPropertySet;
   private final StoredPropertySetROS2Input commandInput;
   private final Throttler parameterOutputThrottler = new Throttler();
   private final ROS2Publisher<PrimitiveDataVectorMessage> publisher;

   public ROS2StoredPropertySet(ROS2Node ros2Node,
                                StoredPropertySetROS2TopicPair topicPair,
                                T storedPropertySet)
   {
      this.ros2Node = ros2Node;
      this.topicPair = topicPair;
      this.storedPropertySet = storedPropertySet;
      commandInput = new StoredPropertySetROS2Input(ros2Node, topicPair.getCommandTopic(), storedPropertySet);
      publisher = ros2Node.createPublisher(topicPair.getStatusTopic());
   }

   public void updateAndPublishThrottledStatus()
   {
      update();
      publishThrottledStatus();
   }

   public void updateAndPublishStatus()
   {
      update();
      publishStatus();
   }

   /**
    * Synchronized so different threads can call update() without worry.
    */
   public synchronized void update()
   {
      commandInput.setToAcceptUpdate(); // Always accept updates
      commandInput.update();
   }

   public void publishThrottledStatus()
   {
      // Heartbeat so remote UI tuners can stay up to date
      if (parameterOutputThrottler.run(STATUS_PERIOD))
      {
         publishStatus();
      }
   }

   public void publishStatus()
   {
      publisher.publish(StoredPropertySetMessageTools.newMessage(storedPropertySet));
   }

   public StoredPropertySetROS2Input getCommandInput()
   {
      return commandInput;
   }

   public T getStoredPropertySet()
   {
      return storedPropertySet;
   }

   @Override
   public String toString()
   {
      return "{" + this.topicPair.getCommandTopic().getName() + ", " + this.topicPair.getStatusTopic().getName() + "}";
   }
}
