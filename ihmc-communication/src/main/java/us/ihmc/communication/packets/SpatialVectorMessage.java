package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Vector3D32;

public class SpatialVectorMessage extends Packet<SpatialVectorMessage>
{
   public Vector3D32 angularPart = new Vector3D32();
   public Vector3D32 linearPart = new Vector3D32();

   public SpatialVectorMessage()
   {
   }
   
   @Override
   public void set(SpatialVectorMessage other)
   {
      angularPart.set(other.angularPart);
      linearPart.set(other.linearPart);
   }

   @Override
   public boolean epsilonEquals(SpatialVectorMessage other, double epsilon)
   {
      return angularPart.epsilonEquals(other.angularPart, epsilon) && linearPart.epsilonEquals(other.linearPart, epsilon);
   }
}
