package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public abstract class FrameShape3d<F extends FrameShape3d<F, G>, G extends Shape3DBasics & GeometryObject<G>> extends FrameGeometryObject<F, G>
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
   public final double distance(FramePoint3D point)
   {
      checkReferenceFrameMatch(point);
      
      return getGeometryObject().distance(point);
   }
   
   /**
    * Find the closest point on the surface of this shape to the given point as well as the surface normal at that point.
    * 
    * @param closestPointToPack  out parameter packed with the resulting closest point on the shape
    * @param normalToPack  out parameter packed with the resulting normal vector
    * @param pointInWorldToCheck
    */
   public final void getClosestPointAndNormalAt(FramePoint3D closestPointToPack, FrameVector3D normalToPack, FramePoint3D pointInWorldToCheck)
   {
      checkReferenceFrameMatch(pointInWorldToCheck);
      closestPointToPack.setToZero(referenceFrame);
      normalToPack.setToZero(referenceFrame);
      
      getGeometryObject().evaluatePoint3DCollision(pointInWorldToCheck, closestPointToPack, normalToPack);
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
      return getGeometryObject().evaluatePoint3DCollision(pointToCheck, closestPointOnSurfaceToPack, normalToPack);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape.
    * @param pointToCheck
    * @return
    */
   public final boolean isInsideOrOnSurface(FramePoint3D pointToCheck)
   {
      checkReferenceFrameMatch(pointToCheck);
      
      return getGeometryObject().isPointInside(pointToCheck);
   }

   /**
    * Determine whether the given point is on or inside the surface of this shape,
    * within a given tolerance or error level.
    * @param pointToCheck
    * @param epsilon
    * @return
    */
   public final boolean isInsideOrOnSurface(FramePoint3D pointToCheck, double epsilon)
   {
      checkReferenceFrameMatch(pointToCheck);
      
      return getGeometryObject().isPointInside(pointToCheck, epsilon);
   }

   /**
    * Find the closest point on the surface of this shape to the given point.
    * If the given point is on or inside the shape, then it is not changed.
    * 
    * @param pointToCheckAndPack both an input parameter (the point to check),
    *          and an output parameter (packed with the resulting ortho point).
    */
   public final void orthogonalProjection(FramePoint3D pointToCheckAndPack)
   {
      checkReferenceFrameMatch(pointToCheckAndPack);
      
      getGeometryObject().orthogonalProjection(pointToCheckAndPack);
   }
}