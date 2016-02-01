package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public abstract class FrameShape3d extends ReferenceFrameHolder
{
   /**
    * Apply the given transform to translate and rotate this shape.
    * @param transform
    */
   public abstract void applyTransform(RigidBodyTransform transform);

   /**
    * Find the distance from the closest point on this shape to the given point. 
    * @param point
    * @return
    */
   public abstract double distance(FramePoint point);

   /**
    * Find the closest point on the surface of this shape to the given point as well as the surface normal at that point.
    * 
    * @param closestPointToPack  out parameter packed with the resulting closest point on the shape
    * @param normalToPack  out parameter packed with the resulting normal vector
    * @param pointInWorldToCheck
    */
   public abstract void getClosestPointAndNormalAt(FramePoint closestPointToPack, FrameVector normalToPack, FramePoint pointInWorldToCheck);

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    * @param pointToCheck
    * @return
    */
   public abstract boolean isInsideOrOnSurface(FramePoint pointToCheck);

   /**
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level.
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public abstract boolean isInsideOrOnSurface(FramePoint pointToCheck, double epsilon);

   /**
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * 
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting ortho point).
    */
   public abstract void orthogonalProjection(FramePoint pointToCheckAndPack);

}