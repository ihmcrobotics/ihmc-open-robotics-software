package us.ihmc.communication;

import us.ihmc.ros2.RealtimeRos2Publisher;

public class ROS2ObjectPublisher<T>
{
   private final RealtimeRos2Publisher<T> realtimeRos2Publisher;

   ROS2ObjectPublisher(RealtimeRos2Publisher<T> realtimeRos2Publisher)
   {
      this.realtimeRos2Publisher = realtimeRos2Publisher;
   }

   @SuppressWarnings("unchecked")
   public boolean publish(Object message)
   {
      return realtimeRos2Publisher.publish((T) message);
   }
}
