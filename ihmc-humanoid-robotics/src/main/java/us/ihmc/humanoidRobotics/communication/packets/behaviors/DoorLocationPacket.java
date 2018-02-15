package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class DoorLocationPacket extends Packet<DoorLocationPacket>
{
   public RigidBodyTransform doorTransformToWorld;

   public DoorLocationPacket()
   {

   }

   @Override
   public void set(DoorLocationPacket other)
   {
      doorTransformToWorld = new RigidBodyTransform(other.doorTransformToWorld);
      setPacketInformation(other);
   }

   public RigidBodyTransform getValveTransformToWorld()
   {
      return doorTransformToWorld;
   }

   public boolean epsilonEquals(DoorLocationPacket doorPacket, double epsilon)
   {
      boolean transformEquals = doorTransformToWorld.epsilonEquals(doorPacket.getValveTransformToWorld(), epsilon);

      return transformEquals;
   }
}
