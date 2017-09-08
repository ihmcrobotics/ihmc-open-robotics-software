package us.ihmc.utilities.ros.publisher;


public class RosStringPublisher extends RosTopicPublisher<std_msgs.String>
{
   private final String initialValue;
   
   public RosStringPublisher(boolean latched)
   {
      this(latched, null);
   }
   
   public RosStringPublisher(boolean latched, String initialValue)
   {
      super(std_msgs.String._TYPE, latched);
      this.initialValue = initialValue;
   }
   
   @Override
   public void connected()
   {
      if(initialValue != null)
      {
         publish(initialValue);
      }
   }

   public void publish(String value)
   {
      std_msgs.String message = getMessage();
      message.setData(value);
      publish(message);
   }
}
