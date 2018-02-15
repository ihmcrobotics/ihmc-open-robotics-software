package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class DetectedObjectPacket extends Packet<DetectedObjectPacket>
{
   public RigidBodyTransform pose;
   public int id;

   public DetectedObjectPacket()
   {
   }

   public DetectedObjectPacket(RigidBodyTransform pose, int id)
   {
      this.pose = pose;
      this.id = id;
   }

   @Override
   public void set(DetectedObjectPacket other)
   {
      pose = new RigidBodyTransform(other.pose);
      id = other.id;
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(DetectedObjectPacket other, double epsilon)
   {
      return pose.epsilonEquals(other.pose, epsilon) && id == other.id;
   }

}
