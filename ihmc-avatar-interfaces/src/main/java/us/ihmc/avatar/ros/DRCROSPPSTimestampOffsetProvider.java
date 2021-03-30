package us.ihmc.avatar.ros;

import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosNodeInterface;

public interface DRCROSPPSTimestampOffsetProvider extends PPSTimestampOffsetProvider
{
   public void subscribeROS1(RosNodeInterface ros1Node);

   public default void unsubscribeROS1(RosNodeInterface ros1Node)
   {
   }
}
