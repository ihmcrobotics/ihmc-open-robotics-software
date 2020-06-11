package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.ros2.Ros2PublisherBasics;

import java.io.IOException;
import java.util.function.Supplier;

public class ManagedROS2Publisher<T> implements Ros2PublisherBasics<T>
{
   private final Ros2PublisherBasics<T> publisher;
   private final Supplier<Boolean> enabled;

   public ManagedROS2Publisher(Ros2PublisherBasics<T> publisher, Supplier<Boolean> enabled)
   {
      this.publisher = publisher;
      this.enabled = enabled;
   }

   @Override
   public void publish(T data) throws IOException
   {
      if (enabled.get())
      {
         publisher.publish(data);
      }
   }

   @Override
   public void remove()
   {
      publisher.remove();
   }
}
