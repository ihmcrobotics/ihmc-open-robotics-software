package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.transformables.AbstractPose;
import us.ihmc.robotics.math.Epsilons;

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
   private final Plane3d rampPlane;

   private final Point3D temporaryPoint;

   public Ramp3d(Ramp3d ramp3d)
   {
      this(ramp3d, ramp3d.getWidth(), ramp3d.getLength(), ramp3d.getHeight());
   }

   public Ramp3d(double width, double length, double height)
   {
      size = new Size3d(length, width, height);
      rampPlane = new Plane3d();
      temporaryPoint = new Point3D();

      updateRamp();
   }

   public Ramp3d(RigidBodyTransform transform, double width, double length, double height)
   {
      setPose(transform);
      size = new Size3d(length, width, height);
      rampPlane = new Plane3d();
      temporaryPoint = new Point3D();

      updateRamp();
   }

   public Ramp3d(AbstractPose pose, double width, double length, double height)
   {
      setPose(pose);
      size = new Size3d(length, width, height);
      rampPlane = new Plane3d();
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
   public void applyTransform(Transform transform)
   {
      applyTransformToPose(transform);
      updateRamp();
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3DReadOnly pointInWorldToCheck, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      boolean isInsideOrOnSurface = isInsideOrOnSurfaceShapeFrame(pointInWorldToCheck, 0.0);
      
      closestPointToPack.set(pointInWorldToCheck);
      
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
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly pointToCheck, double epsilon)
   {
      return MathTools.isPreciselyBoundedByInclusive(0.0, size.getX(), pointToCheck.getX(), epsilon * 2.0)
          && MathTools.isPreciselyBoundedByInclusive(-size.getY() / 2.0, size.getY() / 2.0, pointToCheck.getY(), epsilon * 2.0)
          && MathTools.isPreciselyBoundedByInclusive(0.0, size.getZ(), pointToCheck.getZ(), epsilon * 2.0)
          && rampPlane.isOnOrBelow(pointToCheck, epsilon);
   }

   @Override
   protected double distanceShapeFrame(Point3DReadOnly point)
   {
      temporaryPoint.set(point);
      orthogonalProjectionShapeFrame(temporaryPoint);
      return temporaryPoint.distance(point);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack)
   {
      if (!isInsideOrOnSurfaceShapeFrame(pointToCheckAndPack, Epsilons.ONE_BILLIONTH))
      {
         if (rampPlane.isOnOrAbove(pointToCheckAndPack))
         {
            rampPlane.orthogonalProjection(pointToCheckAndPack);
         }
         
         GeometryTools.clipToBoundingBox(pointToCheckAndPack, 0.0, getLength(), -getWidth() / 2.0, getWidth() / 2.0, 0.0, getHeight());
      }
      else
      {
         double distanceToNegativeYSide = Math.abs(size.getY() / 2.0 + pointToCheckAndPack.getY());
         double distanceToPositiveYSide = Math.abs(size.getY() / 2.0 - pointToCheckAndPack.getY());
         double distanceToFront = Math.abs(size.getX() - pointToCheckAndPack.getX());
         double distanceToBottom = Math.abs(pointToCheckAndPack.getZ());
         double distanceToRampPlane = Math.abs(rampPlane.distance(pointToCheckAndPack));
         
         if (firstIsLeast(distanceToNegativeYSide, distanceToPositiveYSide, distanceToFront, distanceToBottom, distanceToRampPlane))
         {
            pointToCheckAndPack.setY(-size.getY() / 2.0);
         }
         else if (firstIsLeast(distanceToPositiveYSide, distanceToFront, distanceToBottom, distanceToRampPlane, distanceToNegativeYSide))
         {
            pointToCheckAndPack.setY(size.getY() / 2.0);
         }
         else if (firstIsLeast(distanceToFront, distanceToBottom, distanceToRampPlane, distanceToNegativeYSide, distanceToPositiveYSide))
         {
            pointToCheckAndPack.setX(size.getX());
         }
         else if (firstIsLeast(distanceToBottom, distanceToRampPlane, distanceToNegativeYSide, distanceToPositiveYSide, distanceToFront))
         {
            pointToCheckAndPack.setZ(0.0);
         }
         else // distanceToRampPlane is least
         {
            rampPlane.orthogonalProjection(pointToCheckAndPack);
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
      size.setToZero();
   }

   @Override
   public void setToNaN()
   {
      size.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return size.containsNaN();
   }

   @Override
   public String toString()
   {
      return "size = " + size + ", + transform = " + getPoseString() + "\n";
   }
}
