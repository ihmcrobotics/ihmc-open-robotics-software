package us.ihmc.communication.ros2log;

import gnu.trove.list.array.TLongArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.LongSupplier;

class RecordTopicManager<T>
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
      this.ros2Topic = ros2Topic;

      ros2Node.createSubscription(ros2Topic, s ->
      {
         long timestamp = timestampSupplier.getAsLong();
         latestData.set(Pair.of(timestamp, s.takeNextData()));
      });
   }

   void update()
   {
      Pair<Long, T> data = this.latestData.getAndSet(null);

      if (data != null)
      {
         timestamps.add(data.getLeft());
         messages.add(data.getRight());
      }
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
}
