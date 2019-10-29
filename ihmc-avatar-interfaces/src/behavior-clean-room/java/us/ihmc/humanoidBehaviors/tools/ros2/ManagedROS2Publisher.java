package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.ros2.Ros2PublisherBasics;

import java.io.IOException;

public class ManagedROS2Publisher<T> implements Ros2PublisherBasics<T>
{
   private final Ros2PublisherBasics<T> publisher;
   private boolean enabled = true;

   public ManagedROS2Publisher(Ros2PublisherBasics<T> publisher)
   {
      this.publisher = publisher;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   @Override
   public void publish(T data) throws IOException
   {
      if (enabled)
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
