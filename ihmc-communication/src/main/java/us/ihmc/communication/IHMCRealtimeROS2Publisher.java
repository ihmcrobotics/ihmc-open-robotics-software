package us.ihmc.communication;

import us.ihmc.log.LogTools;
import us.ihmc.ros2.RealtimeROS2Publisher;

public class IHMCRealtimeROS2Publisher<T>
{
   private final RealtimeROS2Publisher<T> realtimeROS2Publisher;
   private int numberOfExceptions = 0;

   IHMCRealtimeROS2Publisher(RealtimeROS2Publisher<T> realtimeROS2Publisher)
   {
      this.realtimeROS2Publisher = realtimeROS2Publisher;
   }

   public boolean publish(T message)
   {
      try
      {
         return realtimeROS2Publisher.publish(message);
      }
      catch (Exception e)
      {
         if (numberOfExceptions < 6)
         {
            e.printStackTrace();
            numberOfExceptions++;

            if (numberOfExceptions >= 6)
            {

               LogTools.error("Stopping to print exceptions after 5.");
            }
         }

         return false;
      }
   }
}
