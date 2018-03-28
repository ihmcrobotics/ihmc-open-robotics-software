package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;

public class StereoVisionPointCloudMessage extends Packet<StereoVisionPointCloudMessage>
{
   public long robotTimestamp;
   public TFloatArrayList pointCloud = new TFloatArrayList();
   public TIntArrayList colors = new TIntArrayList();

   public StereoVisionPointCloudMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(StereoVisionPointCloudMessage other)
   {
      robotTimestamp = other.robotTimestamp;
      MessageTools.copyData(other.pointCloud, pointCloud);
      MessageTools.copyData(other.colors, colors);
      setPacketInformation(other);
   }

   public void setRobotTimestamp(long robotTimestamp)
   {
      this.robotTimestamp = robotTimestamp;
   }

   public TIntArrayList getColors()
   {
      return colors;
   }

   @Override
   public boolean epsilonEquals(StereoVisionPointCloudMessage other, double epsilon)
   {
      if (pointCloud.size() != other.pointCloud.size())
         return false;
      for (int i = 0; i < pointCloud.size(); i++)
      {
         if (!MathTools.epsilonEquals(pointCloud.get(i), other.pointCloud.get(i), epsilon))
            return false;
      }

      return colors.equals(other.colors);
   }
}
