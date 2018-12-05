package us.ihmc.communication;

import us.ihmc.commons.PrintTools;
import us.ihmc.ros2.RealtimeRos2Publisher;

public class IHMCRealtimeROS2Publisher<T>
{
   private final RealtimeRos2Publisher<T> realtimeRos2Publisher;

   int numberOfExceptions = 0;

   IHMCRealtimeROS2Publisher(RealtimeRos2Publisher<T> realtimeRos2Publisher)
   {
      this.realtimeRos2Publisher = realtimeRos2Publisher;
   }

   public boolean publish(T message)
   {
      try
      {
         return realtimeRos2Publisher.publish(message);
      }
      catch (Exception e)
      {
         if (numberOfExceptions < 6)
         {
            e.printStackTrace();
            numberOfExceptions++;

            if (numberOfExceptions >= 6)
            {

               PrintTools.error("Stopping to print exceptions after 5.");
            }
         }

         return false;
      }
   }
}
