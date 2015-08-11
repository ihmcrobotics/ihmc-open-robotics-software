package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public interface Shape3d
{
   /**
    * Apply the given transform to translate and rotate this shape.
    *
    * @param transform
    */
   public abstract void applyTransform(RigidBodyTransform transform);

   /**
    * Find the distance from the closest point on this shape to the given point. Returns 0.0 if the point is inside.
    *
    * @param point
    * @return distance from the point to this Shape3d.
    */
   public abstract double distance(Point3d point);

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    *
    * @param pointToCheck
    * @return true if the point is inside or on the surface, false otherwise.
    */
   public abstract boolean isInsideOrOnSurface(Point3d pointToCheck);

   /**
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level. If epsilonToGrowObject is positive, then 
    * the object will be checked as being larger. If negative, then the object will be shrunk.
    *
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public abstract boolean isInsideOrOnSurface(Point3d pointToCheck, double epsilonToGrowObject);

   /**
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * If you wish to know the surface normal, then subtract the projected point from the original point.
    *
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting orthogonal point).
    */
   public abstract void orthogonalProjection(Point3d pointToCheckAndPack);

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
   public abstract boolean checkIfInside(Point3d pointToCheck, Point3d closestPointOnSurfaceToPack, Vector3d normalToPack);

}
