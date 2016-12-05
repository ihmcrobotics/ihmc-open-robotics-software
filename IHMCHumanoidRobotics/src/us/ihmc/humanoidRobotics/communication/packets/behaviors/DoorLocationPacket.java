package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class DoorLocationPacket extends Packet<DoorLocationPacket>
{
   public RigidBodyTransform doorTransformToWorld;
   
   
   public DoorLocationPacket()
   {
      
   }
   
   public DoorLocationPacket(RigidBodyTransform doorTransformToWorld)
   {
      this.doorTransformToWorld = doorTransformToWorld;
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
