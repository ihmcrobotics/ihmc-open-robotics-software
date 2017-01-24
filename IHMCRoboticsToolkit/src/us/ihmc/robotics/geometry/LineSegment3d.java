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

   public void set(Point3d firstEndpoint, Vector3d fromFirstToSecondEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
      this.secondEndpoint.add(firstEndpoint, fromFirstToSecondEndpoint);
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

   public boolean firstEndpointContainsNaN()
   {
      return firstEndpoint.containsNaN();
   }

   public boolean secondEndpointContainsNaN()
   {
      return secondEndpoint.containsNaN();
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

   public double distance(LineSegment3d otherLineSegment)
   {
      return GeometryTools.distanceBetweenTwoLineSegments(firstEndpoint, secondEndpoint, otherLineSegment.firstEndpoint, otherLineSegment.secondEndpoint);
   }

   public Point3d orthogonalProjectionCopy(Point3d pointToProject)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint);
   }

   public boolean orthogonalProjection(Point3d pointToProject, Point3d projectionToPack)
   {
      return GeometryTools.getOrthogonalProjectionOnLineSegment(pointToProject, firstEndpoint, secondEndpoint, projectionToPack);
   }

   public void getPointAlongPercentageOfLineSegment(double percentage, Point3d pointToPack)
   {
      if (percentage < 0.0 || percentage > 1.0)
         throw new RuntimeException("Percentage must be between 0.0 and 1.0. Was: " + percentage);

      pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage);
   }

   public void getMidpoint(Point3d midpointToPack)
   {
      midpointToPack.interpolate(firstEndpoint, secondEndpoint, 0.5);
   }

   public void getDirection(boolean normalize, Vector3d directionToPack)
   {
      directionToPack.sub(secondEndpoint, firstEndpoint);
      if (normalize)
         directionToPack.normalize();
   }

   public Vector3d getDirectionCopy(boolean normalize)
   {
      Vector3d direction = new Vector3d();
      getDirection(normalize, direction);
      return direction;
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

   public Point3d getFirstEndpoint()
   {
      return firstEndpoint;
   }

   public Point3d getSecondEndpoint()
   {
      return secondEndpoint;
   }

   public void getLine(Line3d lineToPack)
   {
      lineToPack.set(firstEndpoint, secondEndpoint);
   }

   public Line3d getLineCopy()
   {
      return new Line3d(firstEndpoint, secondEndpoint);
   }

   @Override
   public boolean epsilonEquals(LineSegment3d other, double epsilon)
   {
      return firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon);
   }
}
