package us.ihmc.ihmcPerception.depthData.collisionShapes;

import us.ihmc.euclid.shape.Box3D;
import us.ihmc.euclid.shape.Shape3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CollisionBox extends CollisionShape
{
   private Box3D shape3D;

   private final double xExtent;
   private final double yExtent;
   private final double zExtent;

   /**
    * @param pose the pose of the center of the box in the parent frame. Used to define a reference frame with an origin
    *             at the center of the box and orientation aligned with x-y-z
    * @param xExtent half the length
    * @param yExtent half the width
    * @param zExtent half the height
    */
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

   /**
    * Checks if the point is contained within the box. Note that this point is assumed to be in the box origin frame.
    * @return whether or not the point is in the box
    */
   @Override
   public boolean contains(Point3D point)
   {
      return (point.getX() >= -xExtent && point.getX() <= xExtent) && (point.getY() >= -yExtent && point.getY() <= yExtent) && (point.getZ() >= -zExtent && point.getZ() <= zExtent);
   }

   @Override
   public Shape3D<?> getOrCreateShape3D()
   {
      if (shape3D == null)
      {
         shape3D = new Box3D(getPose(), 2.0 * xExtent, 2.0 * yExtent, 2.0 * zExtent);
      }
      return shape3D;
   }
}
