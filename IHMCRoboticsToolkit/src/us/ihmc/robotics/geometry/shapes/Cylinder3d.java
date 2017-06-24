package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;

public class Cylinder3d extends Shape3d<Cylinder3d>
{
   private double radius;
   private double height;

   private final Point3D temporaryPoint = new Point3D();
   private final Vector3D temporaryVector = new Vector3D();

   public static enum CylinderFaces
   {
      TOP, BOTTOM
   }

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
   }

   public Cylinder3d(RigidBodyTransform transform, double height, double radius)
   {
      setPose(transform);
      this.height = height;
      this.radius = radius;
   }

   public Cylinder3d(Pose3D pose, double height, double radius)
   {
      setPose(pose);
      this.height = height;
      this.radius = radius;
   }

   @Override
   public void set(Cylinder3d cylinder3d)
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
   public boolean epsilonEquals(Cylinder3d other, double epsilon)
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
      transformToLocal(pointOnLine, temporaryPoint);
      transformToLocal(lineDirection, temporaryVector);
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(height, radius, temporaryPoint, temporaryVector,
                                                                                             firstIntersectionToPack, secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack, firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack, secondIntersectionToPack);
      return numberOfIntersections;
   }

   @Override
   protected boolean checkIfInsideShapeFrame(double x, double y, double z, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      boolean insideOrOnSurfaceLocal = true;

      closestPointOnSurfaceToPack.set(0.0, 0.0, z);
      double radialDistance = EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, closestPointOnSurfaceToPack);

      closestPointOnSurfaceToPack.set(x, y, z);
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
         double distanceSquaredToTop = (height - z) * (height - z);
         double distanceSquaredToBottom = z * z;

         if (distanceSquaredToSide < distanceSquaredToTop && distanceSquaredToSide < distanceSquaredToBottom)
         {
            double xyLength = Math.sqrt(xyLengthSquared);
            if (xyLength > 1e-10)
            {
               double scale = radius / xyLength;

               closestPointOnSurfaceToPack.setX(closestPointOnSurfaceToPack.getX() * scale);
               closestPointOnSurfaceToPack.setY(closestPointOnSurfaceToPack.getY() * scale);
               normalToPack.set(x, y, 0.0);
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
         normalToPack.set(x, y, z);
         normalToPack.sub(closestPointOnSurfaceToPack);
         normalToPack.normalize();
      }

      return insideOrOnSurfaceLocal;
   }

   @Override
   protected double distanceShapeFrame(double x, double y, double z)
   {
      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      if (xyLengthSquared <= radius * radius)
      {
         if (z < 0.0)
            return -z;
         else if (z > height)
            return z - height;
         else
         { // The query is inside the cylinder. Returning a negative distance.
            double dz = -Math.min(z, height - z);
            double dr = Math.sqrt(xyLengthSquared) - radius;
            return dz > dr ? dz : dr;
         }
      }
      else
      {
         double dz = Math.min(-z, z - height);
         double dr = Math.sqrt(xyLengthSquared) - radius;
         return Math.sqrt(EuclidCoreTools.normSquared(dr, dz));
      }
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilonToGrowObject)
   {
      if (z < -epsilonToGrowObject && z > height + epsilonToGrowObject)
         return false;

      double radiusSquared = (radius + epsilonToGrowObject) * (radius + epsilonToGrowObject);
      return EuclidCoreTools.normSquared(x, y) <= radiusSquared;
   }

   @Override
   protected void orthogonalProjectionShapeFrame(double x, double y, double z, Point3DBasics projectionToPack)
   {
      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      if (xyLengthSquared <= radius * radius)
      {
         if (z < 0.0)
            projectionToPack.set(x, y, 0.0);
         else if (z > height)
            projectionToPack.set(x, y, height);
      }
      else
      {
         double scale = radius / Math.sqrt(xyLengthSquared);

         x *= scale;
         y *= scale;
         z = MathTools.clamp(z, 0.0, height);

         projectionToPack.set(x, y, z);
      }
   }

   private final Point3D tempPointInWorldForProjectingToBottom = new Point3D();
   private final Vector3D tempVectorInWorldForProjectingToBottom = new Vector3D();

   /**
    * Projects the pointToProject to the bottom of a curved surface given the surfaceNormal defining
    * how the curved surface is contacting another surface, as when the two surfaces roll on each
    * other.
    *
    * @param surfaceNormal defining another surface in which this object is in rolling contact on
    *           the other surface.
    * @param pointToProject point to project to the bottom of the curved surface.
    * @return true if the point was projected. Otherwise false.
    */
   public boolean projectToBottomOfCurvedSurface(Vector3DReadOnly surfaceNormal, Point3DBasics pointToProject)
   {
      //TODO: Should this method be in Shape3d, or not even be here at all? Not sure...
      tempPointInWorldForProjectingToBottom.set(pointToProject);
      tempVectorInWorldForProjectingToBottom.set(surfaceNormal);
      transformToLocal(tempPointInWorldForProjectingToBottom);
      transformToLocal(tempVectorInWorldForProjectingToBottom);

      boolean wasRolling = projectToBottomOfCurvedSurfaceInShapeFrame(tempVectorInWorldForProjectingToBottom, tempPointInWorldForProjectingToBottom);
      transformToWorld(tempPointInWorldForProjectingToBottom);

      pointToProject.set(tempPointInWorldForProjectingToBottom);
      return wasRolling;
   }

   private final Vector2D tempVectorInShapeFrameForProjectingToBottom = new Vector2D();

   private boolean projectToBottomOfCurvedSurfaceInShapeFrame(Vector3DReadOnly normalInShapeFrame, Point3DBasics pointToRollInShapeFrame)
   {
      double x = normalInShapeFrame.getX();
      double y = normalInShapeFrame.getY();

      if (Math.abs(x) < 1e-7 && Math.abs(y) < 1e-7)
         return false;

      tempVectorInShapeFrameForProjectingToBottom.set(x, normalInShapeFrame.getY());
      tempVectorInShapeFrameForProjectingToBottom.normalize();
      tempVectorInShapeFrameForProjectingToBottom.scale(radius);

      pointToRollInShapeFrame.set(tempVectorInShapeFrameForProjectingToBottom.getX(), tempVectorInShapeFrameForProjectingToBottom.getY(),
                                  pointToRollInShapeFrame.getZ());
      return true;
   }

   @Override
   public String toString()
   {
      return "height = " + height + ", radius = " + radius + "\n";
   }

}
