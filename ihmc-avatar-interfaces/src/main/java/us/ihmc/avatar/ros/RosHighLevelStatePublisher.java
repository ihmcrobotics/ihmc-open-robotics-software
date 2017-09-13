package us.ihmc.avatar.ros;

import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
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

   public void publish(HighLevelControllerState highLevelControllerState)
   {
//      HighLevelStateRosMessage message = getMessage();
//      message.setHighLevelControllerState(GenericRosMessageConverter.convertEnumToByte(highLevelControllerState));
//      publish(message);
   }
}
