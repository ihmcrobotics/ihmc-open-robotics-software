package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.transformables.Pose;

public class Cylinder3d extends Shape3d<Cylinder3d>
{
   private double radius;
   private double height;
   
   private final Point3D temporaryPoint = new Point3D();

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
   }

   public Cylinder3d(RigidBodyTransform transform, double height, double radius)
   {
      setPose(transform);
      this.height = height;
      this.radius = radius;
   }
   
   public Cylinder3d(Pose pose, double height, double radius)
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
            plane = new Plane3d(new Point3D(0.0, 0.0, height), new Vector3D(0.0, 0.0, 1.0));
            break;
         case BOTTOM :
            plane = new Plane3d(new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 1.0));
            break;
         default :
            throw(new RuntimeException("Unrecognized cylinder face"));
      }
      
      transformToWorld(plane);
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
   public void applyTransform(Transform transform)
   {
      applyTransformToPose(transform);
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
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
   protected double distanceShapeFrame(Point3DReadOnly point)
   {
      temporaryPoint.set(point);
      orthogonalProjectionShapeFrame(temporaryPoint);
      return temporaryPoint.distance(point);
   }
      
   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly pointToCheck, double epsilonToGrowObject)
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
   protected void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack)
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

   private final Point3D tempPointInWorldForProjectingToBottom = new Point3D();
   private final Vector3D tempVectorInWorldForProjectingToBottom = new Vector3D();
   
   /**
    * Projects the pointToProject to the bottom of a curved surface given the surfaceNormal defining how the curved surface is
    * contacting another surface, as when the two surfaces roll on each other. 
    * @param surfaceNormal defining another surface in which this object is in rolling contact on the other surface.
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

      if ((Math.abs(x) < 1e-7) && (Math.abs(y) < 1e-7))
         return false;

      tempVectorInShapeFrameForProjectingToBottom.set(x, normalInShapeFrame.getY());
      tempVectorInShapeFrameForProjectingToBottom.normalize();
      tempVectorInShapeFrameForProjectingToBottom.scale(radius);

      pointToRollInShapeFrame.set(tempVectorInShapeFrameForProjectingToBottom.getX(), tempVectorInShapeFrameForProjectingToBottom.getY(), pointToRollInShapeFrame.getZ());
      return true;
   }

   @Override
   public String toString()
   {
      return "height = " + height + ", radius = " + radius + "\n";
   }

}
