package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.geometry.AbstractFrameObject;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class FrameShape3d<F extends FrameShape3d<F, G>, G extends Shape3d<G>> extends AbstractFrameObject<F, G>
{
   public FrameShape3d(G shape3d)
   {
      super(ReferenceFrame.getWorldFrame(), shape3d);
   }

   public FrameShape3d(ReferenceFrame referenceFrame, G shape3d)
   {
      super(referenceFrame, shape3d);
   }

   /**
    * Find the distance from the closest point on this shape to the given point. 
    * @param point
    * @return
    */
   public final double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      
      return getGeometryObject().distance(point.getPoint());
   }
   
   /**
    * Find the closest point on the surface of this shape to the given point as well as the surface normal at that point.
    * 
    * @param closestPointToPack  out parameter packed with the resulting closest point on the shape
    * @param normalToPack  out parameter packed with the resulting normal vector
    * @param pointInWorldToCheck
    */
   public final void getClosestPointAndNormalAt(FramePoint closestPointToPack, FrameVector normalToPack, FramePoint pointInWorldToCheck)
   {
      checkReferenceFrameMatch(pointInWorldToCheck);
      closestPointToPack.setToZero(referenceFrame);
      normalToPack.setToZero(referenceFrame);
      
      getGeometryObject().checkIfInside(pointInWorldToCheck.getPoint(), closestPointToPack.getPoint(), normalToPack.getVector());
   }
   
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
   public final boolean checkIfInside(Point3DBasics pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      return getGeometryObject().checkIfInside(pointToCheck, closestPointOnSurfaceToPack, normalToPack);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    * @param pointToCheck
    * @return
    */
   public final boolean isInsideOrOnSurface(FramePoint pointToCheck)
   {
      checkReferenceFrameMatch(pointToCheck);
      
      return getGeometryObject().isInsideOrOnSurface(pointToCheck.getPoint());
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level.
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public final boolean isInsideOrOnSurface(FramePoint pointToCheck, double epsilon)
   {
      checkReferenceFrameMatch(pointToCheck);
      
      return getGeometryObject().isInsideOrOnSurface(pointToCheck.getPoint(), epsilon);
   }

   /**
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * 
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting ortho point).
    */
   public final void orthogonalProjection(FramePoint pointToCheckAndPack)
   {
      checkReferenceFrameMatch(pointToCheckAndPack);
      
      getGeometryObject().orthogonalProjection(pointToCheckAndPack.getPoint());
   }
}