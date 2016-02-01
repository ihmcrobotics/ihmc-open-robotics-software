package us.ihmc.utilities.ros.publisher;
import org.ros.message.Time;

import rosgraph_msgs.Clock;

public class RosClockPublisher extends RosTopicPublisher<Clock>
{
   public RosClockPublisher()
   {
       super(Clock._TYPE, false);
   }

   @Override
   public void connected()
   {
   }

   public void publish(Time t)
   {
       Clock message = getMessage();
       message.setClock(t);
       try
       {
            publish(message);
       }
       catch (RuntimeException e)
       {
           System.out.println("cant publish clock?");
           e.printStackTrace();
       }
   }

}