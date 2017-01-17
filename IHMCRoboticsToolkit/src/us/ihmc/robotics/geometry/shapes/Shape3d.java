package us.ihmc.robotics.geometry.shapes;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.transformables.Transformable;
import us.ihmc.robotics.math.Epsilons;

public abstract class Shape3d<S extends Shape3d<S>> implements GeometryObject<S>
{
   private final TwoWayRigidBodyTransform twoWayTransform;
   
   protected Shape3d()
   {
      twoWayTransform = new TwoWayRigidBodyTransform();
   }
   
   /**
    * Find the distance from the closest point on this shape to the given point. Returns 0.0 if the point is inside.
    *
    * @param point
    * @return distance from the point to this Shape3d.
    */
   public final double distance(Point3d point)
   {
      transformToShapeFrame(point);
      double distance = distanceShapeFrame(point);
      transformFromShapeFrame(point);
      return distance;
   }
   
   protected abstract double distanceShapeFrame(Point3d point);

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    *
    * @param pointToCheck
    * @return true if the point is inside or on the surface, false otherwise.
    */
   public final boolean isInsideOrOnSurface(Point3d pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, Epsilons.ONE_TRILLIONTH);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level. If epsilonToGrowObject is positive, then 
    * the object will be checked as being larger. If negative, then the object will be shrunk.
    *
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public final boolean isInsideOrOnSurface(Point3d pointToCheck, double epsilonToGrowObject)
   {
      transformToShapeFrame(pointToCheck);
      boolean isInsideOrOnSurface = isInsideOrOnSurfaceShapeFrame(pointToCheck, epsilonToGrowObject);
      transformFromShapeFrame(pointToCheck);
      return isInsideOrOnSurface;
   }
   
   protected abstract boolean isInsideOrOnSurfaceShapeFrame(Point3d pointToCheck, double epsilonToGrowObject);

   /**
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * If you wish to know the surface normal, then subtract the projected point from the original point.
    *
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting orthogonal point).
    */
   public final void orthogonalProjection(Point3d pointToCheckAndPack)
   {
      transformToShapeFrame(pointToCheckAndPack);
      orthogonalProjectionShapeFrame(pointToCheckAndPack);
      transformFromShapeFrame(pointToCheckAndPack);
   }
   
   protected abstract void orthogonalProjectionShapeFrame(Point3d pointToCheckAndPack);

   /**
    * Returns true if inside the Shape3d. If inside, must pack the intersection and normal. If not inside, packing those is optional.
    * But if they are not packed when outside, then they should be set to NaN. If they are set to NaN and you really do wish to see
    * where they would project to, then call orthogonalProjection.
    *
    * @param pointToCheck
    * @param intersectionToPack
    * @param normalToPack
    * @return true if the point is inside, false otherwise.
    */
   public final boolean checkIfInside(Point3d pointToCheck, Point3d closestPointOnSurfaceToPack, Vector3d normalToPack)
   {
      transformToShapeFrame(pointToCheck);
      boolean isInside = checkIfInsideShapeFrame(pointToCheck, closestPointOnSurfaceToPack, normalToPack);
      transformFromShapeFrame(pointToCheck);
      if (closestPointOnSurfaceToPack != null)
      {
         transformFromShapeFrame(closestPointOnSurfaceToPack);
      }
      if (normalToPack != null)
      {
         transformFromShapeFrame(normalToPack);
      }
      return isInside;
   }
   
   protected abstract boolean checkIfInsideShapeFrame(Point3d pointToCheck, Point3d closestPointOnSurfaceToPack, Vector3d normalToPack);
   
   // Transform getters
   
   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(getTransformFromShapeFrameUnsafe());
   }
   
   public RigidBodyTransform getTransformUnsafe()
   {
      return getTransformFromShapeFrameUnsafe();
   }

   public void getPosition(Point3d positionToPack)
   {
      getTransformFromShapeFrameUnsafe().getTranslation(positionToPack);
   }
   
   public void getOrientation(Matrix3d orientationToPack)
   {
      getTransformFromShapeFrameUnsafe().getRotation(orientationToPack);
   }
   
   public void getOrientation(Quat4d orientationToPack)
   {
      getTransformFromShapeFrameUnsafe().getRotation(orientationToPack);
   }
   
   // Transform setters
   
   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      RigidBodyTransform fromShape = getTransformFromShapeFrameUnsafe();
      fromShape.multiply(transform);
      setTransformFromShapeFrame(fromShape);
   }

   public void setPosition(Point3d point)
   {
      RigidBodyTransform fromShape = getTransformFromShapeFrameUnsafe();
      fromShape.setTranslation(point);
      setTransformFromShapeFrame(fromShape);
   }

   public void setPosition(double x, double y, double z)
   {
      RigidBodyTransform fromShape = getTransformFromShapeFrameUnsafe();
      fromShape.setTranslation(x, y, z);
      setTransformFromShapeFrame(fromShape);
   }
   
   public void setOrientation(Quat4d orientation)
   {
      RigidBodyTransform fromShape = getTransformFromShapeFrameUnsafe();
      fromShape.setRotation(orientation);
      setTransformFromShapeFrame(fromShape);
   }
   
   public void setOrientation(Matrix3d orientation)
   {
      RigidBodyTransform fromShape = getTransformFromShapeFrameUnsafe();
      fromShape.setRotation(orientation);
      setTransformFromShapeFrame(fromShape);
   }
   
   public void setPose(Point3d position, Quat4d orientation)
   {
      RigidBodyTransform fromShape = getTransformFromShapeFrameUnsafe();
      fromShape.set(orientation, position);
      setTransformFromShapeFrame(fromShape);
   }
   
   public void setTransform(RigidBodyTransform transform)
   {
      setTransformFromShapeFrame(transform);
   }
   
   // Two way transform boilerplate

   public void setTransformToShapeFrame(RigidBodyTransform transform)
   {
      twoWayTransform.setForwardTransform(transform);
   }
   
   public void setTransformFromShapeFrame(RigidBodyTransform transform)
   {
      twoWayTransform.setBackwardTransform(transform);
   }
   
   public RigidBodyTransform getTransformToShapeFrameUnsafe()
   {
      return twoWayTransform.getForwardTransformUnsafe();
   }
   
   public RigidBodyTransform getTransformFromShapeFrameUnsafe()
   {
      return twoWayTransform.getBackwardTransformUnsafe();
   }
   
   public void ensureBaseTransformsUpToDate()
   {
      twoWayTransform.ensureTransformsUpToDate();
   }
   
   public void transformToShapeFrame(Transformable transformable)
   {
      twoWayTransform.transformForward(transformable);
   }

   public void transformToShapeFrame(Vector3f vector)
   {
      twoWayTransform.transformForward(vector);
   }

   public void transformToShapeFrame(Vector4f vector)
   {
      twoWayTransform.transformForward(vector);
   }

   public void transformToShapeFrame(Vector3d vector)
   {
      twoWayTransform.transformForward(vector);
   }

   public void transformToShapeFrame(Vector4d vector)
   {
      twoWayTransform.transformForward(vector);
   }

   public void transformToShapeFrame(Vector3f vectorIn, Vector3f vectorOut)
   {
      twoWayTransform.transformForward(vectorIn, vectorOut);
   }
   
   public void transformToShapeFrame(Vector4f vectorIn, Vector4f vectorOut)
   {
      twoWayTransform.transformForward(vectorIn, vectorOut);
   }
   
   public void transformToShapeFrame(Vector3d vectorIn, Vector3d vectorOut)
   {
      twoWayTransform.transformForward(vectorIn, vectorOut);
   }
   
   public void transformToShapeFrame(Vector4d vectorIn, Vector4d vectorOut)
   {
      twoWayTransform.transformForward(vectorIn, vectorOut);
   }

   public void transformToShapeFrame(Point3f point)
   {
      twoWayTransform.transformForward(point);
   }
   
   public void transformToShapeFrame(Point3d point)
   {
      twoWayTransform.transformForward(point);
   }
   
   public void transformToShapeFrame(Point3f pointIn, Point3f pointOut)
   {
      twoWayTransform.transformForward(pointIn, pointOut);
   }
   
   public void transformToShapeFrame(Point3d pointIn, Point3d pointOut)
   {
      twoWayTransform.transformForward(pointIn, pointOut);
   }
   
   public void transformFromShapeFrame(Transformable transformable)
   {
      twoWayTransform.transformBackward(transformable);
   }

   public void transformFromShapeFrame(Vector3f vector)
   {
      twoWayTransform.transformBackward(vector);
   }

   public void transformFromShapeFrame(Vector4f vector)
   {
      twoWayTransform.transformBackward(vector);
   }

   public void transformFromShapeFrame(Vector3d vector)
   {
      twoWayTransform.transformBackward(vector);
   }

   public void transformFromShapeFrame(Vector4d vector)
   {
      twoWayTransform.transformBackward(vector);
   }

   public void transformFromShapeFrame(Vector3f vectorIn, Vector3f vectorOut)
   {
      twoWayTransform.transformBackward(vectorIn, vectorOut);
   }
   
   public void transformFromShapeFrame(Vector4f vectorIn, Vector4f vectorOut)
   {
      twoWayTransform.transformBackward(vectorIn, vectorOut);
   }
   
   public void transformFromShapeFrame(Vector3d vectorIn, Vector3d vectorOut)
   {
      twoWayTransform.transformBackward(vectorIn, vectorOut);
   }
   
   public void transformFromShapeFrame(Vector4d vectorIn, Vector4d vectorOut)
   {
      twoWayTransform.transformBackward(vectorIn, vectorOut);
   }

   public void transformFromShapeFrame(Point3f point)
   {
      twoWayTransform.transformBackward(point);
   }
   
   public void transformFromShapeFrame(Point3d point)
   {
      twoWayTransform.transformBackward(point);
   }
   
   public void transformFromShapeFrame(Point3f pointIn, Point3f pointOut)
   {
      twoWayTransform.transformBackward(pointIn, pointOut);
   }
   
   public void transformFromShapeFrame(Point3d pointIn, Point3d pointOut)
   {
      twoWayTransform.transformBackward(pointIn, pointOut);
   }
}
