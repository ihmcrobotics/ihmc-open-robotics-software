package us.ihmc.atlas;


import us.ihmc.utilities.ros.RosMainNode;

public class AlwaysZeroOffsetPPSTimestampOffsetProvider implements PPSTimestampOffsetProvider
{
   @Override
   public long getCurrentTimestampOffset()
   {
      return 0;
   }

   @Override
   public long requestNewestRobotTimestamp()
   {
      return 0;
   }

   @Override
   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
      return;
   }
}
