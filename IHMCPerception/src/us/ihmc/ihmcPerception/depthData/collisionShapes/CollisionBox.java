package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CollisionBox extends CollisionShape
{

   private final double xExtent;
   private final double yExtent;
   private final double zExtent;

   public CollisionBox(RigidBodyTransform pose, double xExtent, double yExtent, double zExtent)
   {
      super(pose);
      this.xExtent = xExtent;
      this.yExtent = yExtent;
      this.zExtent = zExtent;
   }

   public double getxExtent()
   {
      return xExtent;
   }

   public double getyExtent()
   {
      return yExtent;
   }

   public double getzExtent()
   {
      return zExtent;
   }

   @Override
   public boolean contains(Point3D point)
   {
      return (point.getX() >= -xExtent && point.getX() <= xExtent) && (point.getY() >= -yExtent && point.getY() <= yExtent) && (point.getZ() >= -zExtent && point.getZ() <= zExtent);
   }

}
