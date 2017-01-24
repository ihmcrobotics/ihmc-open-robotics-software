package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.geometry.transformables.TransformableVector3d;

public class Line3d implements GeometryObject<Line3d>
{
   private final TransformablePoint3d point = new TransformablePoint3d();
   private final TransformableVector3d normalizedVector = new TransformableVector3d();

   public Line3d()
   {
   }

   public Line3d(Point3d point, Vector3d vector)
   {
      set(point, vector);
   }

   public Line3d(Point3d firstPointOnLine, Point3d secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
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

   public void set(Point3d point, Vector3d vector)
   {
      setPoint(point);
      setVector(vector);
   }

   public void set(Point3d firstPointOnLine, Point3d secondPointOnLine)
   {
      setPoint(firstPointOnLine);
      normalizedVector.sub(secondPointOnLine, firstPointOnLine);
      normalizedVector.normalize();
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

   public double distance(Point3d point)
   {
      return GeometryTools.distanceFromPointToLine(point, point, normalizedVector);
   }

   public double distance(Line3d otherLine)
   {
      return GeometryTools.distanceBetweenTwoLines(point, normalizedVector, otherLine.point, otherLine.normalizedVector);
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

   private void normalize()
   {
      if (GeometryTools.isZero(normalizedVector, 1e-12))
      {
         normalizedVector.setToNaN();
      }

      normalizedVector.normalize();
   }
}
