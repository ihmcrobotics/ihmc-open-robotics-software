package us.ihmc.darpaRoboticsChallenge.ros;

import ihmc_msgs.HighLevelStatePacketMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericRosMessageConverter;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosHighLevelStatePublisher extends RosTopicPublisher<ihmc_msgs.HighLevelStatePacketMessage>
{
   public RosHighLevelStatePublisher(boolean latched)
   {
      super(ihmc_msgs.HighLevelStatePacketMessage._TYPE, latched);
   }

   public void publish(HighLevelState highLevelState)
   {
      HighLevelStatePacketMessage message = getMessage();
      message.setHighLevelState(GenericRosMessageConverter.convertEnumToByte(highLevelState));
      publish(message);
   }
}
