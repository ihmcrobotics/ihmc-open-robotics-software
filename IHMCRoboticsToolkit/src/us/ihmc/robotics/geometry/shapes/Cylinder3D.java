package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;

public class Cylinder3D extends Shape3D<Cylinder3D>
{
   private double radius;
   private double height;

   public Cylinder3D()
   {
      this(1.0, 0.5);
   }

   public Cylinder3D(Cylinder3D cylinder3d)
   {
      this(cylinder3d.radius, cylinder3d.height);
   }

   public Cylinder3D(double height, double radius)
   {
      this.height = height;
      this.radius = radius;
   }

   public Cylinder3D(RigidBodyTransform transform, double height, double radius)
   {
      setPose(transform);
      this.height = height;
      this.radius = radius;
   }

   public Cylinder3D(Pose3D pose, double height, double radius)
   {
      setPose(pose);
      this.height = height;
      this.radius = radius;
   }

   @Override
   public void set(Cylinder3D cylinder3d)
   {
      if (this != cylinder3d)
      {
         setPose(cylinder3d);
         height = cylinder3d.height;
         radius = cylinder3d.radius;
      }
   }

   public double getRadius()
   {
      return radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public double getHeight()
   {
      return height;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(height) || Double.isNaN(radius);
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      height = Double.NaN;
      radius = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      height = 0.0;
      radius = 0.0;
   }

   @Override
   public boolean epsilonEquals(Cylinder3D other, double epsilon)
   {
      return MathTools.epsilonEquals(height, other.height, epsilon) && MathTools.epsilonEquals(radius, other.radius, epsilon);
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

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(height, radius, xLocal, yLocal, zLocal, dxLocal, dyLocal, dzLocal,
                                                                                             firstIntersectionToPack, secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);
      return numberOfIntersections;
   }

   @Override
   protected boolean isInsideEpsilonShapeFrame(double x, double y, double z, double epsilon)
   {
      if (z < -epsilon || z > height + epsilon)
         return false;

      double radiusWithEpsilon = radius + epsilon;
      return EuclidCoreTools.normSquared(x, y) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      if (xyLengthSquared <= radius * radius)
      {
         if (z < 0.0)
         { // The query is directly below the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, 0.0);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, -1.0);
            return -z;
         }

         if (z > height)
         { // The query is directly above the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, height);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, 1.0);
            return z - height;
         }

         // The query is inside the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double dz = Math.min(z, height - z);
         double dr = radius - xyLength;

         if (dz < dr)
         {
            if (z == dz)
            { // Closer to the bottom face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, 0.0);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, -1.0);
               return -z;
            }
            else
            { // Closer to the top face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, height);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, 1.0);
               return z - height;
            }
         }
         else
         { // Closer to the cylinder part
            if (closestPointOnSurfaceToPack != null)
            {
               double xyScale = radius / xyLength;
               closestPointOnSurfaceToPack.set(x * xyScale, y * xyScale, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(x, y, 0.0);
               normalToPack.scale(1.0 / xyLength);
            }
            return xyLength - radius;
         }
      }
      else
      { // The projection of the query onto the xy-plane is outside of the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double xyLengthInv = 1.0 / xyLength;

         double xyClosestScale = radius * xyLengthInv;
         double xClosest = x * xyClosestScale;
         double yClosest = y * xyClosestScale;
         double zClosest = z;

         if (z < 0.0)
            zClosest = 0.0;
         else if (z > height)
            zClosest = height;

         if (zClosest != z)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double dx = x - xClosest;
            double dy = y - yClosest;
            double dz = z - zClosest;

            double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(dx, dy, dz);
               normalToPack.scale(1.0 / distance);
            }

            return distance;
         }
         else
         { // Closest point is on the cylinder.
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(x * xyLengthInv, y * xyLengthInv, 0.0);
            }

            return xyLength - radius;
         }
      }
   }

   @Override
   public String toString()
   {
      return "height = " + height + ", radius = " + radius + "\n";
   }

}
