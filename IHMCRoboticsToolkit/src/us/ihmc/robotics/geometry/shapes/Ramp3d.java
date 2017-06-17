package us.ihmc.robotics.geometry.shapes;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.GeometryTools;

/**
 * Ramp where the center of ramp start side is origin.
 * 0 to X
 * -Y/2 +Y/2
 * 0 to Z
 */
public class Ramp3d extends Shape3d<Ramp3d>
{
   private final Size3d size;
   
   private double rampLength;
   private double angleOfRampIncline;
   private final Plane3D rampPlane;

   private final Point3D temporaryPoint;

   public Ramp3d(Ramp3d ramp3d)
   {
      setPose(ramp3d);
      size = new Size3d(ramp3d.getLength(), ramp3d.getWidth(), ramp3d.getHeight());
      rampPlane = new Plane3D();
      temporaryPoint = new Point3D();

      updateRamp();
   }

   public Ramp3d(double width, double length, double height)
   {
      size = new Size3d(length, width, height);
      rampPlane = new Plane3D();
      temporaryPoint = new Point3D();

      updateRamp();
   }

   public Ramp3d(RigidBodyTransform transform, double width, double length, double height)
   {
      setPose(transform);
      size = new Size3d(length, width, height);
      rampPlane = new Plane3D();
      temporaryPoint = new Point3D();

      updateRamp();
   }

   public Ramp3d(Pose3D pose, double width, double length, double height)
   {
      setPose(pose);
      size = new Size3d(length, width, height);
      rampPlane = new Plane3D();
      temporaryPoint = new Point3D();

      updateRamp();
   }

   @Override
   public void set(Ramp3d ramp3d)
   {
      if (this != ramp3d)
      {
         setPose(ramp3d);
         size.set(ramp3d.size);

         updateRamp();
      }
   }

   private void updateRamp()
   {
      rampLength = Math.sqrt(size.getHeight() * size.getHeight() + size.getLength() * size.getLength());
      
      rampPlane.setToZero();
      rampPlane.setNormal(-size.getZ(), 0.0, size.getX());

      angleOfRampIncline = Math.atan(size.getHeight() / size.getLength());
   }

   public double getWidth()
   {
      return size.getWidth();
   }

   public void setWidth(double width)
   {
      size.setWidth(width);
      updateRamp();
   }

   public double getHeight()
   {
      return size.getHeight();
   }

   public void setHeight(double height)
   {
      size.setHeight(height);
      updateRamp();
   }

   public double getLength()
   {
      return size.getLength();
   }

   public void setLength(double length)
   {
      size.setLength(length);
      updateRamp();
   }

   public double getRampLength()
   {
      return rampLength;
   }

   public void getSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(rampPlane.getNormal());
      transformToWorld(surfaceNormalToPack);
   }

   public double getRampIncline()
   {
      return angleOfRampIncline;
   }

   @Override
   protected boolean checkIfInsideShapeFrame(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      boolean isInsideOrOnSurface = isInsideOrOnSurfaceShapeFrame(x, y, z, 0.0);
      
      closestPointToPack.set(x, y, z);
      
      if (!isInsideOrOnSurface)
      {
         orthogonalProjectionShapeFrame(closestPointToPack);
      }
      
      findClosestSurfaceNormal(closestPointToPack, normalToPack);
      
      return isInsideOrOnSurface;
   }
   
   private void findClosestSurfaceNormal(Point3DReadOnly point, Vector3DBasics normalToPack)
   {
      double distanceToNegativeYSide = Math.abs(size.getY() / 2.0 + point.getY());
      double distanceToPositiveYSide = Math.abs(size.getY() / 2.0 - point.getY());
      double distanceToFront = Math.abs(size.getX() - point.getX());
      double distanceToBottom = Math.abs(point.getZ());
      double distanceToRampPlane = Math.abs(rampPlane.distance(point));
      
      if (firstIsLeast(distanceToNegativeYSide, distanceToPositiveYSide, distanceToFront, distanceToBottom, distanceToRampPlane))
      {
         normalToPack.set(0.0, -1.0, 0.0);
      }
      else if (firstIsLeast(distanceToPositiveYSide, distanceToFront, distanceToBottom, distanceToRampPlane, distanceToNegativeYSide))
      {
         normalToPack.set(0.0, 1.0, 0.0);
      }
      else if (firstIsLeast(distanceToFront, distanceToBottom, distanceToRampPlane, distanceToNegativeYSide, distanceToPositiveYSide))
      {
         normalToPack.set(1.0, 0.0, 0.0);
      }
      else if (firstIsLeast(distanceToBottom, distanceToRampPlane, distanceToNegativeYSide, distanceToPositiveYSide, distanceToFront))
      {
         normalToPack.set(0.0, 0.0, -1.0);
      }
      else // distanceToRampPlane is least
      {
         normalToPack.set(rampPlane.getNormal());
      }
   }
   
   private boolean firstIsLeast(double least, double num1, double num2, double num3, double num4)
   {
      return least <= num1 && least <= num2 && least <= num3 && least <= num4;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      return MathTools.intervalContains(x, 0.0, size.getX(), epsilon * 2.0)
          && MathTools.intervalContains(y, -size.getY() / 2.0, size.getY() / 2.0, epsilon * 2.0)
          && MathTools.intervalContains(z, 0.0, size.getZ(), epsilon * 2.0)
          && rampPlane.isOnOrBelow(x, y, z, epsilon);
   }

   @Override
   protected double distanceShapeFrame(double x, double y, double z)
   {
      temporaryPoint.set(x, y, z);
      orthogonalProjectionShapeFrame(temporaryPoint);
      return EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, temporaryPoint);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(double x, double y, double z, Point3DBasics projectionToPack)
   {
      if (!isInsideOrOnSurfaceShapeFrame(x, y, z, Epsilons.ONE_BILLIONTH))
      {
         if (rampPlane.isOnOrAbove(x, y, z))
         {
            rampPlane.orthogonalProjection(x, y, z, projectionToPack);
         }
         else
         {
            projectionToPack.set(x, y, z);
         }
         
         GeometryTools.clipToBoundingBox(projectionToPack, 0.0, getLength(), -getWidth() / 2.0, getWidth() / 2.0, 0.0, getHeight());
      }
      else
      {
         double distanceToNegativeYSide = Math.abs(size.getY() / 2.0 + y);
         double distanceToPositiveYSide = Math.abs(size.getY() / 2.0 - y);
         double distanceToFront = Math.abs(size.getX() - x);
         double distanceToBottom = Math.abs(z);
         double distanceToRampPlane = Math.abs(rampPlane.distance(x, y, z));
         
         projectionToPack.set(x, y, z);

         if (firstIsLeast(distanceToNegativeYSide, distanceToPositiveYSide, distanceToFront, distanceToBottom, distanceToRampPlane))
         {
            projectionToPack.set(x, -size.getY() / 2.0, z);
         }
         else if (firstIsLeast(distanceToPositiveYSide, distanceToFront, distanceToBottom, distanceToRampPlane, distanceToNegativeYSide))
         {
            projectionToPack.set(x, size.getY() / 2.0, z);
         }
         else if (firstIsLeast(distanceToFront, distanceToBottom, distanceToRampPlane, distanceToNegativeYSide, distanceToPositiveYSide))
         {
            projectionToPack.set(size.getX(), y, z);
         }
         else if (firstIsLeast(distanceToBottom, distanceToRampPlane, distanceToNegativeYSide, distanceToPositiveYSide, distanceToFront))
         {
            projectionToPack.set(x, y, 0.0);
         }
         else // distanceToRampPlane is least
         {
            rampPlane.orthogonalProjection(x, y, z, projectionToPack);
         }
      }
   }

   @Override
   public boolean epsilonEquals(Ramp3d other, double epsilon)
   {
      return size.epsilonEquals(other.size, epsilon);
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      size.setToZero();
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      size.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || size.containsNaN();
   }

   @Override
   public String toString()
   {
      return "size = " + size + ", + transform = " + getPoseString() + "\n";
   }
}
