package us.ihmc.avatar.ros;

import ihmc_msgs.HighLevelStateRosMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosHighLevelStatePublisher extends RosTopicPublisher<ihmc_msgs.HighLevelStateRosMessage>
{
   public RosHighLevelStatePublisher(boolean latched)
   {
      super(ihmc_msgs.HighLevelStateRosMessage._TYPE, latched);
   }

   public void publish(HighLevelState highLevelState)
   {
//      HighLevelStateRosMessage message = getMessage();
//      message.setHighLevelState(GenericRosMessageConverter.convertEnumToByte(highLevelState));
//      publish(message);
   }
}
