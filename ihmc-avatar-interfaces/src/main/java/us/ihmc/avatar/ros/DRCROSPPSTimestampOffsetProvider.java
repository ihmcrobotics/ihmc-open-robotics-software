package us.ihmc.avatar.ros;

import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosNodeInterface;

public interface DRCROSPPSTimestampOffsetProvider extends PPSTimestampOffsetProvider
{
   public void subscribeToROS1Topics(RosNodeInterface ros1Node);

   /**
    * Use {@link #subscribeToROS1Topics(RosNodeInterface)} instead.
    **/
   @Deprecated
   public default void attachToRosMainNode(RosMainNode rosMainNode)
   {
      subscribeToROS1Topics(rosMainNode);
   }

   public default void unsubscribeFromROS1Topics(RosNodeInterface ros1Node)
   {
   }
}
