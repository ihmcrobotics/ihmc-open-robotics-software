package us.ihmc.communication.ros2log;

import gnu.trove.list.array.TLongArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.LongSupplier;

class RecordTopicManager<T extends ROS2Message<T>>
{
   private final ROS2Topic<T> ros2Topic;
   private final AtomicReference<Pair<Long, T>> latestData = new AtomicReference<>();
   private final List<T> messages = new ArrayList<>();
   private final TLongArrayList timestamps = new TLongArrayList();

   /**
    * Record constructor. LongSupplier provides a timestamp in milliseconds and should be thread-safe
    */
   RecordTopicManager(ROS2Topic<T> ros2Topic, ROS2Node ros2Node, LongSupplier timestampSupplier)
   {
      this(ros2Topic, ros2Node, timestampSupplier, true);
   }

   /**
    * Record constructor. LongSupplier provides a timestamp in milliseconds and should be thread-safe.
    * When subscribeToTopic is false, data is expected to be provided via setData(...).
    */
   RecordTopicManager(ROS2Topic<T> ros2Topic, ROS2Node ros2Node, LongSupplier timestampSupplier, boolean subscribeToTopic)
   {
      this.ros2Topic = ros2Topic;

      if (subscribeToTopic)
      {
         ros2Node.createSubscription(ros2Topic, reader ->
         {
            T message = reader.read();
            if (message == null)
               return;

            long timestamp = timestampSupplier.getAsLong();
            latestData.set(Pair.of(timestamp, message));
         });
      }
   }

   boolean update()
   {
      Pair<Long, T> data = this.latestData.getAndSet(null);
      boolean hasData = data != null;

      if (hasData)
      {
         timestamps.add(data.getLeft());
         messages.add(data.getRight());
      }

      return hasData;
   }

   void clear()
   {
      messages.clear();
      timestamps.clear();
      latestData.set(null);
   }

   ROS2Topic<T> getTopic()
   {
      return ros2Topic;
   }

   List<T> getMessages()
   {
      return messages;
   }

   TLongArrayList getTimestamps()
   {
      return timestamps;
   }

   /* for testing */
   AtomicReference<Pair<Long,T>> getDataReference()
   {
      return latestData;
   }

   void setData(long timestamp, T data)
   {
      latestData.set(Pair.of(timestamp, data));
   }
}
