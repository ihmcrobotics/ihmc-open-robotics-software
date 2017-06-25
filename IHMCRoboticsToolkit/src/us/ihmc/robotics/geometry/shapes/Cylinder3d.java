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
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
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
