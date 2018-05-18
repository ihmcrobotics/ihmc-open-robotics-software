package us.ihmc.communication;

import us.ihmc.commons.PrintTools;
import us.ihmc.ros2.RealtimeRos2Publisher;

public class ROS2ObjectPublisher<T>
{
   private final RealtimeRos2Publisher<T> realtimeRos2Publisher;
   
   int numberOfExceptions = 0;

   ROS2ObjectPublisher(RealtimeRos2Publisher<T> realtimeRos2Publisher)
   {
      this.realtimeRos2Publisher = realtimeRos2Publisher;
   }

   @SuppressWarnings("unchecked")
   public boolean publish(Object message)
   {
      try
      {
         return realtimeRos2Publisher.publish((T) message);
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
