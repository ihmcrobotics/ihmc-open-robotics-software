package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;

public class DoorLocationPacket extends Packet<DoorLocationPacket>
{
   public Pose3D doorTransformToWorld;

   public DoorLocationPacket()
   {

   }

   @Override
   public void set(DoorLocationPacket other)
   {
      doorTransformToWorld = new Pose3D(other.doorTransformToWorld);
      setPacketInformation(other);
   }

   public Pose3D getDoorTransformToWorld()
   {
      return doorTransformToWorld;
   }

   public boolean epsilonEquals(DoorLocationPacket doorPacket, double epsilon)
   {
      boolean transformEquals = doorTransformToWorld.epsilonEquals(doorPacket.getDoorTransformToWorld(), epsilon);

      return transformEquals;
   }
}
