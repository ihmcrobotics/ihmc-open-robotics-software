package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;

public class DoorLocationPacket extends Packet<DoorLocationPacket>
{
   public Pose3D doorTransformToWorld = new Pose3D();

   public DoorLocationPacket()
   {

   }

   @Override
   public void set(DoorLocationPacket other)
   {
      doorTransformToWorld.set(other.doorTransformToWorld);
      setPacketInformation(other);
   }

   public Pose3D getDoorTransformToWorld()
   {
      return doorTransformToWorld;
   }

   public boolean epsilonEquals(DoorLocationPacket doorPacket, double epsilon)
   {
      return doorTransformToWorld.epsilonEquals(doorPacket.getDoorTransformToWorld(), epsilon);
   }
}
