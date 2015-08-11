package us.ihmc.robotics.geometry;

import org.apache.commons.lang3.tuple.ImmutablePair;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Ray3d 
{
   private final Point3d point;
   private final Vector3d vector;
   
   public static Ray3d transformRay3d(Ray3d ray3d, RigidBodyTransform transformToHere)
   {
      Point3d transformedPoint = new Point3d();
      Vector3d transformedVector = new Vector3d();
      transformToHere.transform(ray3d.getPoint(), transformedPoint);
      transformToHere.transform(ray3d.getVector(), transformedVector);

      return new Ray3d(transformedPoint, transformedVector);
   }

   public Ray3d(ImmutablePair<Point3d, Vector3d> pair)
   {
      this.point = pair.getLeft();
      this.vector = pair.getRight();
   }

   public Ray3d(Point3d point, Vector3d vector)
   {
      this.point = point;
      this.vector = vector;
   }

   public Point3d getPoint()
   {
      return point;
   }

   public Vector3d getVector()
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
