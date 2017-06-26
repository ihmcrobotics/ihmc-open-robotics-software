package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

/**
 * The torus is built in the XY plane, around the z axis
 */
public class Torus3d extends Shape3D<Torus3d>
{
   private double radius;
   private double tubeRadius;

   private static final double SMALLEST_ALLOWABLE_THICKNESS = 5e-3;
   private static final double SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS = 1e-4;

   public Torus3d(Torus3d torus3d)
   {
      setPose(torus3d);
      this.radius = torus3d.radius;
      this.tubeRadius = torus3d.tubeRadius;

      checkRadiusAndThickness();
   }

   public Torus3d()
   {
      radius = 1.0;
      tubeRadius = 0.1;

      checkRadiusAndThickness();
   }

   public Torus3d(double radius, double thickness)
   {
      this.radius = radius;
      this.tubeRadius = thickness;

      checkRadiusAndThickness();
   }

   public Torus3d(RigidBodyTransform transform, double radius, double thickness)
   {
      setPose(transform);
      this.radius = radius;
      this.tubeRadius = thickness;

      checkRadiusAndThickness();
   }

   public Torus3d(Pose3D pose, double radius, double thickness)
   {
      setPose(pose);
      this.radius = radius;
      this.tubeRadius = thickness;

      checkRadiusAndThickness();
   }

   @Override
   public void set(Torus3d torus3d)
   {
      setPose(torus3d);
      this.radius = torus3d.radius;
      this.tubeRadius = torus3d.tubeRadius;
   }

   private void checkRadiusAndThickness()
   {
      if (radius - tubeRadius < SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS)
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): "
               + "Invalid dimensions: Difference between radius and thickness is too small. Enter new dimensions.");

      if (tubeRadius < SMALLEST_ALLOWABLE_THICKNESS)
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): " + "Invalid thickness: Torus is too thin. Enter new dimensions.");
   }

   public double getRadius()
   {
      return radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public double getThickness()
   {
      return tubeRadius;
   }

   public void setThickness(double thickness)
   {
      this.tubeRadius = thickness;
   }

   @Override
   public boolean epsilonEquals(Torus3d other, double epsilon)
   {
      return MathTools.epsilonEquals(radius, other.radius, epsilon) && MathTools.epsilonEquals(tubeRadius, other.tubeRadius, epsilon);
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      radius = 0.0;
      tubeRadius = 0.0;
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius = Double.NaN;
      tubeRadius = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(radius) || Double.isNaN(tubeRadius);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", thickness = " + tubeRadius + ", pose = " + getPoseString() + "\n";
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      double outerRadius = radius + tubeRadius + epsilon;
      double innerRadius = radius - tubeRadius - epsilon;

      if (xyLengthSquared > outerRadius * outerRadius || xyLengthSquared < innerRadius * innerRadius)
         return false;

      double xyScale = radius / Math.sqrt(xyLengthSquared);

      double closestTubeCenterX = x * xyScale;
      double closestTubeCenterY = y * xyScale;

      return EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, closestTubeCenterX, closestTubeCenterY, 0.0) <= tubeRadius + epsilon;
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double xyLengthSquared = normSquared(x, y);

      if (xyLengthSquared < 1.0e-12)
      {
         double xzLength = Math.sqrt(normSquared(radius, z));

         if (closestPointToPack != null)
         {
            double scale = tubeRadius / xzLength;
            double closestTubeCenterX = radius;
            double closestTubeCenterY = 0.0;

            double tubeCenterToSurfaceX = -radius * scale;
            double tubeCenterToSurfaceY = 0.0;
            double tubeCenterToSurfaceZ = z * scale;

            closestPointToPack.set(closestTubeCenterX, closestTubeCenterY, 0.0);
            closestPointToPack.add(tubeCenterToSurfaceX, tubeCenterToSurfaceY, tubeCenterToSurfaceZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(-radius, 0.0, z);
            normalToPack.scale(1.0 / xzLength);
         }

         return xzLength - tubeRadius;
      }
      else
      {
         double xyScale = radius / Math.sqrt(xyLengthSquared);

         double closestTubeCenterX = x * xyScale;
         double closestTubeCenterY = y * xyScale;

         double dx = x - closestTubeCenterX;
         double dy = y - closestTubeCenterY;
         double dz = z;

         double distance = Math.sqrt(normSquared(dx, dy, dz));

         double distanceInv = 1.0 / distance;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(dx, dy, dz);
            closestPointToPack.scale(tubeRadius * distanceInv);
            closestPointToPack.add(closestTubeCenterX, closestTubeCenterY, 0.0);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(distanceInv);
         }

         return distance - tubeRadius;
      }
   }
}
