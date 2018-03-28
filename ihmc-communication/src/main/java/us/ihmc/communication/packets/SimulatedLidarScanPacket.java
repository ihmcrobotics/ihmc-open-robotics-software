package us.ihmc.communication.packets;

import gnu.trove.list.array.TFloatArrayList;

public class SimulatedLidarScanPacket extends Packet<SimulatedLidarScanPacket>
{
   public TFloatArrayList ranges = new TFloatArrayList();
   public int sensorId;
   public LidarScanParametersMessage lidarScanParameters;

   public SimulatedLidarScanPacket()
   {
   }

   @Override
   public void set(SimulatedLidarScanPacket other)
   {
      MessageTools.copyData(other.ranges, ranges);
      sensorId = other.sensorId;
      lidarScanParameters = other.lidarScanParameters;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(SimulatedLidarScanPacket other, double epsilon)
   {
      if (!lidarScanParameters.equals(other.lidarScanParameters))
         return false;
      if (!MessageTools.epsilonEquals(ranges, other.ranges, epsilon))
         return false;

      return true;
   }

   public LidarScanParametersMessage getLidarScanParameters()
   {
      return lidarScanParameters;
   }

   public TFloatArrayList getRanges()
   {
      return ranges;
   }

   public int getSensorId()
   {
      return sensorId;
   }
}
