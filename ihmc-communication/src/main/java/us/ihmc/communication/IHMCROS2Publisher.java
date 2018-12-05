package us.ihmc.communication;

import us.ihmc.commons.PrintTools;
import us.ihmc.ros2.Ros2Publisher;

public class IHMCROS2Publisher<T>
{
   private final Ros2Publisher<T> ros2Publisher;

   int numberOfExceptions = 0;

   IHMCROS2Publisher(Ros2Publisher<T> ros2Publisher)
   {
      this.ros2Publisher = ros2Publisher;
   }

   public void publish(T message)
   {
      try
      {
         ros2Publisher.publish(message);
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
      }
   }
}
