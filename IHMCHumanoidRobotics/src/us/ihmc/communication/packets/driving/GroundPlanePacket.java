package us.ihmc.communication.packets.driving;

import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.communication.packets.Packet;


public class GroundPlanePacket extends Packet<GroundPlanePacket>
{
   public Se3_F64 groundPlane;

   public GroundPlanePacket()
   {
   }

   public GroundPlanePacket(Se3_F64 groundPlane)
   {
      this.groundPlane = groundPlane;
   }

   public Se3_F64 getGroundPlane()
   {
      return groundPlane;
   }

   public boolean equals(Object other)
   {
      if (other instanceof GroundPlanePacket)
      {
         return epsilonEquals((GroundPlanePacket) other, 1e-6);
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(GroundPlanePacket other, double epsilon)
   {
      return other.getGroundPlane().getT().isIdentical(getGroundPlane().getT(), epsilon);
   }

   public GroundPlanePacket(Random random)
   {
      DenseMatrix64F rotation = CommonOps.identity(3, 3);
      Vector3D_F64 point = new Vector3D_F64(random.nextDouble(), random.nextDouble(), random.nextDouble());
      this.groundPlane = new Se3_F64(rotation, point);
   }
}
