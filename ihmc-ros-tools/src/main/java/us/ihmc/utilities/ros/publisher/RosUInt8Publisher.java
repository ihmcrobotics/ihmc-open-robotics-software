package us.ihmc.utilities.ros.publisher;

import std_msgs.UInt8;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosUInt8Publisher extends RosTopicPublisher<std_msgs.UInt8>
{

   public RosUInt8Publisher(boolean latched)
   {
      super(std_msgs.UInt8._TYPE, latched);
   }

   public void publish(byte value)
   {
      UInt8 message = getMessage();
      message.setData(value);
      publish(message);
   }
}
