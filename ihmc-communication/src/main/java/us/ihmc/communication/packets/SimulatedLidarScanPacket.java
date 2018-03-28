package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;

public class SimulatedLidarScanPacket extends Packet<SimulatedLidarScanPacket>
{
   public TFloatArrayList ranges = new TFloatArrayList();
   public int sensorId;
   public LidarScanParametersMessage params;

   public SimulatedLidarScanPacket()
   {
   }

   @Override
   public void set(SimulatedLidarScanPacket other)
   {
      MessageTools.copyData(other.ranges, ranges);
      sensorId = other.sensorId;
      params = other.params;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(SimulatedLidarScanPacket other, double epsilon)
   {
      if (!params.equals(other.params))
         return false;
      if (!MessageTools.epsilonEquals(ranges, other.ranges, epsilon))
         return false;

      return true;
   }

   public long getScanStartTime()
   {
      return params.getTimestamp();
   }

   public LidarScanParametersMessage getLidarScanParameters()
   {
      return params;
   }

   public TFloatArrayList getRanges()
   {
      return ranges;
   }

   public int getSensorId()
   {
      return sensorId;
   }

   public int size()
   {
      return ranges.size();
   }
}
