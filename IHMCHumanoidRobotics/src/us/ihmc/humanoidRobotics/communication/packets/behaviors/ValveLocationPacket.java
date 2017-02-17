package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ValveLocationPacket extends Packet<ValveLocationPacket>
{
   public RigidBodyTransform valveTransformToWorld;
   public double valveRadius;
   
   
   public ValveLocationPacket()
   {
      
   }
   
   public ValveLocationPacket(RigidBodyTransform valveTransformToWorld, double valveRadius)
   {
      this.valveTransformToWorld = valveTransformToWorld;
      this.valveRadius = valveRadius;
   }

   public RigidBodyTransform getValveTransformToWorld()
   {
      return valveTransformToWorld;
   }

   public double getValveRadius()
   {
      return valveRadius;
   }
   
   public boolean epsilonEquals(ValveLocationPacket turnValvePacket, double epsilon)
   {
      boolean transformEquals = valveTransformToWorld.epsilonEquals(turnValvePacket.getValveTransformToWorld(), epsilon);
      boolean radiusEquals = valveRadius == turnValvePacket.getValveRadius();

      return transformEquals  && radiusEquals;
   }
}
