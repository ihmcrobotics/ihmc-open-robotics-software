package us.ihmc.avatar.networkProcessor.time;


import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.perception.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosNodeInterface;

public class DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider extends AlwaysZeroOffsetPPSTimestampOffsetProvider implements DRCROSPPSTimestampOffsetProvider
{
   @Override
   public void subscribeToROS1Topics(RosNodeInterface ros1Node)
   {
   }
}
