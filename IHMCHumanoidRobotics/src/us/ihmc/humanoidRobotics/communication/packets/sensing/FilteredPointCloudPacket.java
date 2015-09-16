package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;


public class FilteredPointCloudPacket extends AbstractPointCloudPacket
{
   public RigidBodyTransform transform;

   public FilteredPointCloudPacket(Random random)
   {
	   super(random);
	   transform = RigidBodyTransform.generateRandomTransform(random);
   }
   
   public FilteredPointCloudPacket()
   {
      super();
   }
   
   public FilteredPointCloudPacket(Point3d origin, ArrayList<Point3d> pointCloud, RigidBodyTransform transform, long timeStamp)
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
