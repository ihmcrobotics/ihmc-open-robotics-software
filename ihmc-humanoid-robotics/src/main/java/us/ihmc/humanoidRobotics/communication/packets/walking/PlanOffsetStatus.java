package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PlanOffsetStatus extends SettablePacket<PlanOffsetStatus>
{
   public Vector3D offsetVector = new Vector3D();

   public PlanOffsetStatus()
   {
      this.offsetVector.setToZero();
   }

   public Vector3D getOffsetVector()
   {
      return offsetVector;
   }

   @Override
   public boolean epsilonEquals(PlanOffsetStatus other, double epsilon)
   {
      return offsetVector.epsilonEquals(other.getOffsetVector(), epsilon);
   }

   @Override
   public void set(PlanOffsetStatus other)
   {
      offsetVector.set(other.getOffsetVector());
      setPacketInformation(other);
   }
}
