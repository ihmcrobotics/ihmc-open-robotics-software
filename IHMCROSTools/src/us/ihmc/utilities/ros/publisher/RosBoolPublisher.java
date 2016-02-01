package us.ihmc.utilities.ros.publisher;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosBoolPublisher extends RosTopicPublisher<std_msgs.Bool>
{
   public RosBoolPublisher(boolean latched)
   {
      super(std_msgs.Bool._TYPE, latched);
   }

   public void publish(boolean value)
   {
      std_msgs.Bool message = getMessage();
      message.setData(value);
      publish(message);
   }
}
