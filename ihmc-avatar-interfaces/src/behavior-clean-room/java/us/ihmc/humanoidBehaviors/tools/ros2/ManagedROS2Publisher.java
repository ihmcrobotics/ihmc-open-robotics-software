package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.ros2.ROS2PublisherBasics;

import java.io.IOException;
import java.util.function.Supplier;

public class ManagedROS2Publisher<T> implements ROS2PublisherBasics<T>
{
   private final ROS2PublisherBasics<T> publisher;
   private final Supplier<Boolean> enabled;

   public ManagedROS2Publisher(ROS2PublisherBasics<T> publisher, Supplier<Boolean> enabled)
   {
      this.publisher = publisher;
      this.enabled = enabled;
   }

   @Override
   public boolean publish(T data) throws IOException
   {
      if (enabled.get())
      {
         publisher.publish(data);
         return true;
      }
      return false;
   }

   @Override
   public void remove()
   {
      publisher.remove();
   }
}
