package us.ihmc.robotics.geometry;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Ray3d 
{
   private final Point3D point;
   private final Vector3D vector;
   
   public static Ray3d transformRay3d(Ray3d ray3d, RigidBodyTransform transformToHere)
   {
      Point3D transformedPoint = new Point3D();
      Vector3D transformedVector = new Vector3D();
      transformToHere.transform(ray3d.getPoint(), transformedPoint);
      transformToHere.transform(ray3d.getVector(), transformedVector);

      return new Ray3d(transformedPoint, transformedVector);
   }

   public Ray3d(Point3D point, Vector3D vector)
   {
      this.point = point;
      this.vector = vector;
   }

   public Point3D getPoint()
   {
      return point;
   }

   public Vector3D getVector()
   {
      return vector;
   }

   public void transformInPlace(RigidBodyTransform transformToHere)
   {
      this.set(transformRay3d(this, transformToHere));
   }

   public void set(Ray3d ray3d)
   {
      this.getPoint().set(ray3d.getPoint());
      this.getVector().set(ray3d.getVector());
   }

}
