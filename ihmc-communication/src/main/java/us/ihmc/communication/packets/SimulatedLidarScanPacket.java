package us.ihmc.communication.packets;

import java.util.Arrays;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.lidar.LidarScanParameters;

public class SimulatedLidarScanPacket extends Packet<SimulatedLidarScanPacket>
{
   public float[] ranges;
   public int sensorId;
   public LidarScanParameters params;

   public SimulatedLidarScanPacket()
   {
   }

   @Override
   public void set(SimulatedLidarScanPacket other)
   {
      ranges = Arrays.copyOf(other.ranges, other.ranges.length);
      sensorId = other.sensorId;
      params = other.params;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(SimulatedLidarScanPacket other, double epsilon)
   {
      boolean ret = true;
      for (int i = 0; i < ranges.length; i++)
      {
         ret &= MathTools.epsilonEquals(ranges[i], other.ranges[i], epsilon);
      }

      ret &= params.equals(other.params);

      return ret;
   }

   public long getScanStartTime()
   {
      return params.getTimestamp();
   }

   public LidarScanParameters getLidarScanParameters()
   {
      return params;
   }

   public float[] getRanges()
   {
      return ranges;
   }

   public int getSensorId()
   {
      return sensorId;
   }

   public int size()
   {
      return ranges.length;
   }
}
