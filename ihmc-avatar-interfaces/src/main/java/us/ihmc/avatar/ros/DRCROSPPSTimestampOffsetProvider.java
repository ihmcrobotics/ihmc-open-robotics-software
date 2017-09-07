package us.ihmc.avatar.ros;

import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;

public interface DRCROSPPSTimestampOffsetProvider extends PPSTimestampOffsetProvider
{
   public void attachToRosMainNode(RosMainNode rosMainNode);
}
