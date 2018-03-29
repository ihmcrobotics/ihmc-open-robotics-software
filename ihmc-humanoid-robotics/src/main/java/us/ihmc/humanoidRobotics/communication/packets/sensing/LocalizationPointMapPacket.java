package us.ihmc.humanoidRobotics.communication.packets.sensing;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

public class LocalizationPointMapPacket extends Packet<LocalizationPointMapPacket>
{
   public long timestamp;
   public TFloatArrayList localizationPointMap = new TFloatArrayList();

   public LocalizationPointMapPacket()
   {
      setDestination(PacketDestination.UI);
   }

   @Override
   public void set(LocalizationPointMapPacket other)
   {
      timestamp = other.timestamp;
      MessageTools.copyData(other.localizationPointMap, localizationPointMap);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(LocalizationPointMapPacket other, double epsilon)
   {
      if (timestamp != other.timestamp)
         return false;
      if (!MessageTools.epsilonEquals(localizationPointMap, other.localizationPointMap, epsilon))
         return false;
      return true;
   }

   @Override
   public String toString()
   {
      return "PointCloudWorldPacket [timestamp=" + timestamp + " points, localizationPointMap=" + localizationPointMap.size() / 3 + "]";
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

}
