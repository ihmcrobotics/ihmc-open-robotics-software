package us.ihmc.darpaRoboticsChallenge.networkProcessor.time;


import us.ihmc.darpaRoboticsChallenge.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.ihmcPerception.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;

public class DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider extends AlwaysZeroOffsetPPSTimestampOffsetProvider implements DRCROSPPSTimestampOffsetProvider
{
   @Override
   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
   }
}
