package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;


public class FilteredPointCloudPacket extends AbstractPointCloudPacket
{
   public RigidBodyTransform transform;

   public FilteredPointCloudPacket(Random random)
   {
	   super(random);
	   transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
   }
   
   public FilteredPointCloudPacket()
   {
      super();
   }
   
   public FilteredPointCloudPacket(Point3D origin, ArrayList<Point3D> pointCloud, RigidBodyTransform transform, long timeStamp)
   {
      super(origin, pointCloud, timeStamp);
      this.transform = transform;
   }


   public boolean epsilonEquals(AbstractPointCloudPacket other, double epsilon)
   {
      if (!(other instanceof FilteredPointCloudPacket))
      {
         return false;
      }

      FilteredPointCloudPacket otherPacket = (FilteredPointCloudPacket) other;

      if (!super.epsilonEquals(otherPacket, epsilon))
      {
         return false;
      }

      if ((getTransform() != null) && (otherPacket.getTransform() == null))
      {
         return false;
      }
      else if ((getTransform() == null) && (otherPacket.getTransform() != null))
      {
         return false;
      }

      if (!getTransform().epsilonEquals(otherPacket.getTransform(), epsilon))
      {
         return false;
      }

      return true;
   }

   public RigidBodyTransform getTransform()
   {
      return transform;
   }

}
