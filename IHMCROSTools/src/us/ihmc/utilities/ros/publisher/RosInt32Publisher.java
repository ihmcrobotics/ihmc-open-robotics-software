package us.ihmc.utilities.ros.publisher;


public class RosInt32Publisher extends RosTopicPublisher<std_msgs.Int32>
{
   
   public RosInt32Publisher(boolean latched)
   {
      super(std_msgs.Int32._TYPE, latched);
   }
   
   @Override
   public void connected()
   {
   }

   public void publish(int value)
   {
      std_msgs.Int32 message = getMessage();
      message.setData(value);
      publish(message);
   }
}
