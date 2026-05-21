package us.ihmc.communication.ros2log;

import gnu.trove.list.array.TLongArrayList;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.ObjLongConsumer;

class ReplayTopicManager<T extends ROS2Message<T>>
{
   private final String topicName;
   private final Consumer<T> messageConsumer;
   private final List<T> messages = new ArrayList<>();
   private final TLongArrayList timestamps = new TLongArrayList();

   private int lastSentIndex = -1;
   private boolean isDone = false;
   private ObjLongConsumer<T> mutator = (m, t) -> {};

   ReplayTopicManager(ROS2Topic<T> ros2Topic, Consumer<T> messageConsumer)
   {
      this.topicName = ros2Topic.getName();
      this.messageConsumer = messageConsumer;
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
         mutator.accept(messages.get(latestIndex), currentTime);
         messageConsumer.accept(messages.get(latestIndex));
         lastSentIndex = latestIndex;
      }
      isDone = latestIndex == timestamps.size() - 1;
   }

   public void updateInternalRepeat(long currentTime)
   {
      if (lastSentIndex >= 0 && lastSentIndex < messages.size())
      {
         mutator.accept(messages.get(lastSentIndex), currentTime);
         messageConsumer.accept(messages.get(lastSentIndex));
      }
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

   public String getTopicName()
   {
      return topicName;
   }

   public void setMutator(ObjLongConsumer<?> mutator)
   {
      this.mutator = (ObjLongConsumer<T>) mutator;
   }

   List<T> getMessages()
   {
      return messages;
   }

   TLongArrayList getTimestamps()
   {
      return timestamps;
   }

   void reset()
   {
      lastSentIndex = -1;
      isDone = false;
   }
}
