package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;

public class LineSegment3d implements GeometryObject<LineSegment3d>
{
   private final TransformablePoint3d firstEndpoint = new TransformablePoint3d();
   private final TransformablePoint3d secondEndpoint = new TransformablePoint3d();

   public LineSegment3d()
   {
   }

   public LineSegment3d(Point3d firstEndpoint, Point3d secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   @Override
   public void set(LineSegment3d lineSegment)
   {
      set(lineSegment.firstEndpoint, lineSegment.secondEndpoint);
   }

   public void set(Point3d firstEndpoint, Point3d secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   public void set(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      setFirstEndpoint(firstEndpointX, firstEndpointY, firstEndpointZ);
      setSecondEndpoint(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   public void setFirstEndpoint(Point3d firstEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
   }

   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY, firstEndpointZ);
   }

   public void setSecondEndpoint(Point3d secondEndpoint)
   {
      this.secondEndpoint.set(secondEndpoint);
   }

   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      secondEndpoint.set(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   @Override
   public void setToZero()
   {
      firstEndpoint.setToZero();
      secondEndpoint.setToZero();
   }

   @Override
   public void setToNaN()
   {
      firstEndpoint.setToNaN();
      secondEndpoint.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return firstEndpoint.containsNaN() || secondEndpoint.containsNaN();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(firstEndpoint);
      transform.transform(secondEndpoint);
   }

   public double length()
   {
      return firstEndpoint.distance(secondEndpoint);
   }

   public double distance(Point3d point)
   {
      return GeometryTools.distanceFromPointToLineSegment(point, firstEndpoint, secondEndpoint);
   }

   public Point3d orthogonalProjectionCopy(Point3d pointToProject)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint);
   }

   public boolean orthogonalProjection(Point3d pointToProject, Point3d projectionToPack)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint, projectionToPack);
   }

   public boolean isBetweenEndpoints(Point3d point3d, double epsilon)
   {
      return isBetweenEndpoints(point3d.getX(), point3d.getY(), point3d.getZ(), epsilon);
   }

   private boolean isBetweenEndpoints(double x, double y, double z, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y, z);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   public double percentageAlongLineSegment(Point3d point3d)
   {
      return percentageAlongLineSegment(point3d.getX(), point3d.getY(), point3d.getZ());
   }

   private double percentageAlongLineSegment(double x, double y, double z)
   {
      return GeometryTools.getPercentageAlongLineSegment(x, y, z, firstEndpoint.getX(), firstEndpoint.getY(), firstEndpoint.getZ(), secondEndpoint.getX(),
                                                         secondEndpoint.getY(), secondEndpoint.getZ());
   }

   public Vector3d getDirectionCopy()
   {
      Vector3d direction = new Vector3d(secondEndpoint);
      direction.sub(firstEndpoint);
      direction.normalize();

      return direction;
   }

   public Point3d getFirstEndpoint()
   {
      return firstEndpoint;
   }

   public Point3d getSecondEndpoint()
   {
      return secondEndpoint;
   }

   @Override
   public boolean epsilonEquals(LineSegment3d other, double epsilon)
   {
      // TODO Auto-generated method stub
      return false;
   }
}
