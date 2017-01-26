package us.ihmc.robotics.geometry.shapes;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class Cylinder3d extends Shape3d<Cylinder3d>
{
   private double radius;
   private double height;
   
   private final Point3d temporaryPoint;

   public static enum CylinderFaces {TOP, BOTTOM}

   public Cylinder3d()
   {
      this(1.0, 0.5);
   }
   
   public Cylinder3d(Cylinder3d cylinder3d)
   {
      this(cylinder3d.radius, cylinder3d.height);
   }

   public Cylinder3d(double height, double radius)
   {
      this.height = height;
      this.radius = radius;
      
      temporaryPoint = new Point3d();
   }

   public Cylinder3d(RigidBodyTransform transform, double height, double radius)
   {
      setTransform(transform);
      this.height = height;
      this.radius = radius;
      
      temporaryPoint = new Point3d();
   }

   @Override
   public void set(Cylinder3d cylinder3d)
   {
      if (this != cylinder3d)
      {
         setTransformFromShapeFrame(cylinder3d.getTransformToShapeFrameUnsafe());
         this.height = cylinder3d.height;
         this.radius = cylinder3d.radius;
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

   public Plane3d getPlane(CylinderFaces face)
   {
      Plane3d plane;
      switch (face)
      {
         case TOP :
            plane = new Plane3d(new Point3d(0.0, 0.0, height), new Vector3d(0.0, 0.0, 1.0));
            break;
         case BOTTOM :
            plane = new Plane3d(new Point3d(0.0, 0.0, 0.0), new Vector3d(0.0, 0.0, 1.0));
            break;
         default :
            throw(new RuntimeException("Unrecognized cylinder face"));
      }
      
      transformFromShapeFrame(plane);
      return plane;
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(height) || Double.isNaN(radius);
   }

   @Override
   public void setToNaN()
   {
      height = Double.NaN;
      radius = Double.NaN;
   }

   @Override
   public void setToZero()
   {
      height = 0.0;
      radius = 0.0;
   }

   @Override
   public boolean epsilonEquals(Cylinder3d other, double epsilon)
   {
      return MathTools.epsilonEquals(height, other.height, epsilon) && MathTools.epsilonEquals(radius, other.radius, epsilon);
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3d pointToCheck, Point3d closestPointOnSurfaceToPack, Vector3d normalToPack)
   {
      boolean insideOrOnSurfaceLocal = true;
      
      closestPointOnSurfaceToPack.set(0.0, 0.0, pointToCheck.getZ());
      double radialDistance = pointToCheck.distance(closestPointOnSurfaceToPack);
      
      closestPointOnSurfaceToPack.set(pointToCheck);
      double xyLengthSquared = closestPointOnSurfaceToPack.getX() * closestPointOnSurfaceToPack.getX()
            + closestPointOnSurfaceToPack.getY() * closestPointOnSurfaceToPack.getY();
      
      if (closestPointOnSurfaceToPack.getZ() < 0.0)
      {
         closestPointOnSurfaceToPack.setZ(0.0);
         insideOrOnSurfaceLocal = false;
      }
      else if (closestPointOnSurfaceToPack.getZ() > height)
      {
         closestPointOnSurfaceToPack.setZ(height);
         insideOrOnSurfaceLocal = false;
      }
      
      if (radialDistance > radius)
      {
         double xyLength = Math.sqrt(xyLengthSquared);
         double scale = radius / xyLength;
         
         closestPointOnSurfaceToPack.setX(closestPointOnSurfaceToPack.getX() * scale);
         closestPointOnSurfaceToPack.setY(closestPointOnSurfaceToPack.getY() * scale);
         insideOrOnSurfaceLocal = false;
      }
      
      if (insideOrOnSurfaceLocal)
      {
         double distanceSquaredToSide = radius * radius - xyLengthSquared;
         double distanceSquaredToTop = (height - pointToCheck.getZ()) * (height - pointToCheck.getZ());
         double distanceSquaredToBottom = pointToCheck.getZ() * pointToCheck.getZ();
         
         if ((distanceSquaredToSide < distanceSquaredToTop) && (distanceSquaredToSide < distanceSquaredToBottom))
         {
            double xyLength = Math.sqrt(xyLengthSquared);
            if (xyLength > 1e-10)
            {
               double scale = radius / xyLength;
      
               closestPointOnSurfaceToPack.setX(closestPointOnSurfaceToPack.getX() * scale);
               closestPointOnSurfaceToPack.setY(closestPointOnSurfaceToPack.getY() * scale);
               normalToPack.set(pointToCheck.getX(), pointToCheck.getY(), 0.0);
               normalToPack.normalize();
            }
            else
            {
               closestPointOnSurfaceToPack.setX(radius);
               closestPointOnSurfaceToPack.setY(0.0);
               normalToPack.set(1.0, 0.0, 0.0);
            }
         }
         else if (distanceSquaredToTop < distanceSquaredToBottom)
         {
            closestPointOnSurfaceToPack.setZ(height);
            normalToPack.set(0.0, 0.0, 1.0);
         }
         else
         {
            closestPointOnSurfaceToPack.setZ(0.0);
            normalToPack.set(0.0, 0.0, -1.0);
         }
      }
      else
      {
         normalToPack.set(pointToCheck);
         normalToPack.sub(closestPointOnSurfaceToPack);
         normalToPack.normalize();
      }
      
      return insideOrOnSurfaceLocal;
   }

   @Override
   protected double distanceShapeFrame(Point3d point)
   {
      temporaryPoint.set(point);
      orthogonalProjectionShapeFrame(temporaryPoint);
      return temporaryPoint.distance(point);
   }
      
   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3d pointToCheck, double epsilonToGrowObject)
   {
      temporaryPoint.set(pointToCheck);

      if (temporaryPoint.getZ() < 0.0 - epsilonToGrowObject)
         return false;
      if (temporaryPoint.getZ() > height + epsilonToGrowObject)
         return false;
      
      double radiusSquared = (radius + epsilonToGrowObject) * (radius + epsilonToGrowObject);
      double xySquared = temporaryPoint.getX() * temporaryPoint.getX() + temporaryPoint.getY() * temporaryPoint.getY();
      
      if (xySquared > radiusSquared)
         return false;
      
      return true;
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3d pointToCheckAndPack)
   {
      if (pointToCheckAndPack.getZ() < 0.0)
         pointToCheckAndPack.setZ(0.0);
      if (pointToCheckAndPack.getZ() > height)
         pointToCheckAndPack.setZ(height);
      
      double xyLengthSquared = pointToCheckAndPack.getX() * pointToCheckAndPack.getX() + pointToCheckAndPack.getY() * pointToCheckAndPack.getY();
      if (xyLengthSquared > radius * radius)
      {
         double xyLength = Math.sqrt(xyLengthSquared);
         double scale = radius / xyLength;
      
         pointToCheckAndPack.setX(pointToCheckAndPack.getX() * scale);
         pointToCheckAndPack.setY(pointToCheckAndPack.getY() * scale);
      }
   }

   @Override
   public String toString()
   {
      return "height = " + height + ", radius = " + radius + "\n";
   }
}
