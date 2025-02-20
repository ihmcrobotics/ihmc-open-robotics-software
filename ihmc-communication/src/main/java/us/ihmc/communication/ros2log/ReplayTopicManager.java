package us.ihmc.communication.ros2log;

import gnu.trove.list.array.TLongArrayList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;

class ReplayTopicManager<T>
{
   private final ROS2Publisher<T> publisher;
   private final List<T> messages = new ArrayList<>();
   private final TLongArrayList timestamps = new TLongArrayList();

   private int lastSentIndex = -1;
   private boolean isDone = false;

   ReplayTopicManager(ROS2Topic<T> ros2Topic, ROS2Node ros2Node)
   {
      this.publisher = ros2Node.createPublisher(ros2Topic);
   }

   boolean update(long currentTime)
   {
      if (!isDone)
         updateInternal(currentTime);
      return isDone;
   }

   private void updateInternal(long currentTime)
   {
      int latestIndex = getLatestIndex(currentTime);

      if (latestIndex != lastSentIndex)
      {
         publisher.publish(messages.get(latestIndex));
         lastSentIndex = latestIndex;
      }
      isDone = latestIndex == timestamps.size() - 1;
   }

   /**
    * Highest index timestamp less than the current time
    */
   private int getLatestIndex(long currentTime)
   {
      int indexToCheck = lastSentIndex + 1;
      while (indexToCheck < timestamps.size() && timestamps.get(indexToCheck) < currentTime)
      {
         indexToCheck++;
      }
      return indexToCheck - 1;
   }

   List<T> getMessages()
   {
      return messages;
   }

   TLongArrayList getTimestamps()
   {
      return timestamps;
   }
}
