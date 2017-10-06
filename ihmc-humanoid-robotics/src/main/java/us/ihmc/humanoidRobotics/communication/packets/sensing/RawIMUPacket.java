package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

// see multisense_ros.RawImuData

public class RawIMUPacket extends Packet<RawIMUPacket>
{
   public long timestampInNanoSecond;
   public Vector3D linearAcceleration;

   public RawIMUPacket()
   {
      linearAcceleration = new Vector3D();
      timestampInNanoSecond = 0;
   }

   @Override
   public boolean epsilonEquals(RawIMUPacket other, double epsilon)
   {
      boolean ret = MathTools.epsilonEquals(timestampInNanoSecond, other.timestampInNanoSecond, epsilon);
      ret &= linearAcceleration.epsilonEquals(other.linearAcceleration, epsilon);
      return ret;
   }

}
