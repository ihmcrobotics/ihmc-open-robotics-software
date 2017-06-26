package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class Ellipsoid3d extends Shape3D<Ellipsoid3d>
{
   private final Vector3D radius = new Vector3D();

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius)
   {
      radius.set(xRadius, yRadius, zRadius);
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, Pose3D pose)
   {
      setPose(pose);
      radius.set(xRadius, yRadius, zRadius);
   }

   public Ellipsoid3d(double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      setPose(transform);
      radius.set(xRadius, yRadius, zRadius);
   }

   public Ellipsoid3d(Ellipsoid3d other)
   {
      setPose(other);
      radius.set(other.radius);
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || radius.containsNaN();
   }

   @Override
   public boolean epsilonEquals(Ellipsoid3d other, double epsilon)
   {
      return radius.epsilonEquals(other.radius, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double sumOfSquares = EuclidCoreTools.normSquared(x / radius.getX(), y / radius.getY(), z / radius.getZ());
      double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);

      if (sumOfSquares > 1.0e-10)
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(x, y, z);
            closestPointToPack.scale(scaleFactor);
         }

         if (normalToPack != null)
         {
            double xScale = 1.0 / (radius.getX() * radius.getX());
            double yScale = 1.0 / (radius.getY() * radius.getY());
            double zScale = 1.0 / (radius.getZ() * radius.getZ());

            normalToPack.set(x, y, z);
            normalToPack.scale(xScale, yScale, zScale);
            normalToPack.normalize();
         }

         return Math.sqrt(EuclidCoreTools.normSquared(x, y, z)) * (1.0 - scaleFactor);
      }
      else
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(0.0, 0.0, radius.getZ());
         }

         if (normalToPack != null)
         {
            normalToPack.set(0.0, 0.0, 1.0);
         }

         return z - radius.getZ();
      }
   }

   public void getCenter(Point3DBasics centerToPack)
   {
      getPosition(centerToPack);
   }

   public void getRadii(Vector3DBasics radiiToPack)
   {
      radiiToPack.set(radius);
   }

   public double getXRadius()
   {
      return radius.getX();
   }

   public double getYRadius()
   {
      return radius.getY();
   }

   public double getZRadius()
   {
      return radius.getZ();
   }

   public int intersectionWith(Line3D line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   public int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                               Point3DBasics secondIntersectionToPack)
   {
      double xLocal = TransformationTools.computeTransformedX(shapePose, true, pointOnLine);
      double yLocal = TransformationTools.computeTransformedY(shapePose, true, pointOnLine);
      double zLocal = TransformationTools.computeTransformedZ(shapePose, true, pointOnLine);

      double dxLocal = TransformationTools.computeTransformedX(shapePose, true, lineDirection);
      double dyLocal = TransformationTools.computeTransformedY(shapePose, true, lineDirection);
      double dzLocal = TransformationTools.computeTransformedZ(shapePose, true, lineDirection);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radius.getX(), radius.getY(), radius.getZ(), xLocal, yLocal,
                                                                                              zLocal, dxLocal, dyLocal, dzLocal, firstIntersectionToPack,
                                                                                              secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack, firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack, secondIntersectionToPack);
      return numberOfIntersections;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      double scaledX = x / (radius.getX() + epsilon);
      double scaledY = y / (radius.getY() + epsilon);
      double scaledZ = z / (radius.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   @Override
   public void set(Ellipsoid3d other)
   {
      if (this != other)
      {
         setPose(other);
         radius.set(other.radius);
      }
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius.setToNaN();
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      radius.setToZero();
   }

   public void setXRadius(double xRadius)
   {
      radius.setX(xRadius);
   }

   public void setYRadius(double yRadius)
   {
      radius.setY(yRadius);
   }

   public void setZRadius(double zRadius)
   {
      radius.setZ(zRadius);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", \npose = " + getPoseString() + "\n";
   }
}
