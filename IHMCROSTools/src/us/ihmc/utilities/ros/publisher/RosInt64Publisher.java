package us.ihmc.utilities.ros.publisher;


public class RosInt64Publisher extends RosTopicPublisher<std_msgs.Int64>
{

   public RosInt64Publisher(boolean latched)
   {
      super(std_msgs.Int64._TYPE, latched);
   }
   
   @Override
   public void connected()
   {
   }

   public void publish(long value)
   {
      std_msgs.Int64 message = getMessage();
      message.setData(value);
      publish(message);
   }
}
