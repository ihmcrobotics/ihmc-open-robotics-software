package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;

public class Line3d implements GeometryObject<Line3d>
{
   private final TransformablePoint3d point;
   private final TransformableVector3d normalizedVector;
   
   public Line3d()
   {
      this.point = new TransformablePoint3d();
      this.normalizedVector = new TransformableVector3d();
   }
   
   public Line3d(Point3d point, Vector3d vector)
   {
      this.point = new TransformablePoint3d(point);
      this.normalizedVector = new TransformableVector3d(vector);
      normalize();
   }
   
   public Point3d getPoint()
   {
      return point;
   }
   
   public Vector3d getNormalizedVector()
   {
      return normalizedVector;
   }
   
   public void setPoint(Point3d point)
   {
      this.point.set(point);
   }
   
   public void setVector(Vector3d vector)
   {
      this.normalizedVector.set(vector);
      normalize();
   }
   
   private void normalize()
   {
      if (GeometryTools.isZero(normalizedVector, 1e-12))
      {
         normalizedVector.setToNaN();
      }
      
      normalizedVector.normalize();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      point.applyTransform(transform);
      normalizedVector.applyTransform(transform);
   }

   @Override
   public boolean epsilonEquals(Line3d other, double epsilon)
   {
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!normalizedVector.epsilonEquals(other.normalizedVector, epsilon))
         return false;

      return true;
   }

   @Override
   public void set(Line3d other)
   {
      point.set(other.point);
      normalizedVector.set(other.normalizedVector);
   }

   @Override
   public void setToZero()
   {
      point.setToZero();
      normalizedVector.setToZero();
   }

   @Override
   public void setToNaN()
   {
      point.setToNaN();
      normalizedVector.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      if (point.containsNaN())
         return true;
      if (normalizedVector.containsNaN())
         return true;

      return false;
   }
}
