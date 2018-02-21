package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.geometry.Pose3D;

public class ValveLocationPacket extends Packet<ValveLocationPacket>
{
   public Pose3D valvePoseInWorld;
   public double valveRadius;
   
   
   public ValveLocationPacket()
   {
      
   }

   @Override
   public void set(ValveLocationPacket other)
   {
      valvePoseInWorld = new Pose3D(other.valvePoseInWorld);
      valveRadius = other.valveRadius;
      setPacketInformation(other);
   }

   public Pose3D getValvePoseInWorld()
   {
      return valvePoseInWorld;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }
   
   public boolean epsilonEquals(ValveLocationPacket turnValvePacket, double epsilon)
   {
      boolean transformEquals = valvePoseInWorld.epsilonEquals(turnValvePacket.getValvePoseInWorld(), epsilon);
      boolean radiusEquals = valveRadius == turnValvePacket.getValveRadius();

      return transformEquals  && radiusEquals;
   }
}
