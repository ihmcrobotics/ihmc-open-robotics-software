package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;

public class BoundingBox3DMessage extends Packet<BoundingBox3DMessage>
{
   public Point3D minPoint = new Point3D();
   public Point3D maxPoint = new Point3D();

   public BoundingBox3DMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(BoundingBox3DMessage other)
   {
      minPoint.set(other.minPoint);
      maxPoint.set(other.maxPoint);
   }

   public Point3D getMinPoint()
   {
      return minPoint;
   }

   public Point3D getMaxPoint()
   {
      return maxPoint;
   }

   @Override
   public boolean epsilonEquals(BoundingBox3DMessage other, double epsilon)
   {
      return minPoint.epsilonEquals(other.minPoint, epsilon) && maxPoint.epsilonEquals(other.maxPoint, epsilon);
   }
}
