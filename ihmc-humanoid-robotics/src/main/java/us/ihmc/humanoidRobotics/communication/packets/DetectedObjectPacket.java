package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;

public class DetectedObjectPacket extends Packet<DetectedObjectPacket>
{
   public Pose3D pose = new Pose3D();
   public int id;

   public DetectedObjectPacket()
   {
   }

   @Override
   public void set(DetectedObjectPacket other)
   {
      pose.set(other.pose);
      id = other.id;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(DetectedObjectPacket other, double epsilon)
   {
      return pose.epsilonEquals(other.pose, epsilon) && id == other.id;
   }

}
