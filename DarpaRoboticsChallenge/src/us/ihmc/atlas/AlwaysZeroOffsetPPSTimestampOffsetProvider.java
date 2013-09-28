package us.ihmc.atlas;


import us.ihmc.utilities.ros.RosMainNode;

public class AlwaysZeroOffsetPPSTimestampOffsetProvider implements PPSTimestampOffsetProvider
{
   public long getCurrentTimestampOffset()
   {
      return 0;
   }

   public long requestNewestRobotTimestamp()
   {
      return 0;
   }

   public long ajustTimeStampToRobotClock(long timeStamp)
   {
      return timeStamp;
   }

   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
      return;
   }
}
