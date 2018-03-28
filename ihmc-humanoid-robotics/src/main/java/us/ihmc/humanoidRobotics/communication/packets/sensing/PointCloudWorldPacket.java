package us.ihmc.humanoidRobotics.communication.packets.sensing;

import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;

@HighBandwidthPacket
public class PointCloudWorldPacket extends Packet<PointCloudWorldPacket>
{
   public long timestamp;

   public TFloatArrayList groundQuadTreeSupport = new TFloatArrayList();

   // Code is duplicated, probably gets replaced with locality hash
   public TFloatArrayList decayingWorldScan = new TFloatArrayList();

   public float defaultGroundHeight;

   public PointCloudWorldPacket()
   {
      this.setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.setDestination(PacketDestination.BROADCAST);
   }

   @Override
   public void set(PointCloudWorldPacket other)
   {
      this.timestamp = other.timestamp;
      MessageTools.copyData(other.groundQuadTreeSupport, this.groundQuadTreeSupport);
      MessageTools.copyData(other.decayingWorldScan, this.decayingWorldScan);
      this.defaultGroundHeight = other.defaultGroundHeight;
      this.setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(PointCloudWorldPacket other, double epsilon)
   {
      boolean ret = this.timestamp == other.timestamp;
      if (!MessageTools.epsilonEquals(this.groundQuadTreeSupport, other.groundQuadTreeSupport, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(this.decayingWorldScan, other.decayingWorldScan, epsilon))
         return false;
      ret &= this.defaultGroundHeight == other.defaultGroundHeight;

      return ret;
   }

   @Override
   public String toString()
   {
      String ret;

      try
      {
         ret = "PointCloudWorldPacket [timestamp=" + this.timestamp + ", groundQuadTreeSupport=" + this.groundQuadTreeSupport.size() / 3 + " points, decayingWorldScan="
               + this.decayingWorldScan.size() / 3 + " points, defaultGroundHeight=" + this.defaultGroundHeight + "]";
      }
      catch (NullPointerException e)
      {
         ret = this.getClass().getSimpleName();
      }

      return ret;
   }

   public long getTimestamp()
   {
      return this.timestamp;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

}
