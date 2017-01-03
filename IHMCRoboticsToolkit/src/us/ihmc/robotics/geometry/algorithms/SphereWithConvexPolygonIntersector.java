package us.ihmc.robotics.geometry.algorithms;

import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.shapes.FrameSphere3d;
import us.ihmc.robotics.geometry.transformables.TransformablePoint2d;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SphereWithConvexPolygonIntersector
{
   private final TransformablePoint3d closestPolygonIntersectionPoint;
   private final TransformablePoint2d closestPolygonIntersectionPoint2d;
   private final FramePoint closestPointOnPolygon;

   public SphereWithConvexPolygonIntersector()
   {
      closestPolygonIntersectionPoint = new TransformablePoint3d();
      closestPolygonIntersectionPoint2d = new TransformablePoint2d();
      closestPointOnPolygon = new FramePoint();
   }

   /**
    * All math in polygon frame.
    */
   public boolean checkIfIntersectionExists(FrameSphere3d sphere, FrameConvexPolygon2d polygon)
   {
      sphere.changeFrame(polygon.getReferenceFrame());
      
      sphere.getCenter(closestPolygonIntersectionPoint);
      
      closestPolygonIntersectionPoint2d.set(closestPolygonIntersectionPoint.getX(), closestPolygonIntersectionPoint.getY());
      ConvexPolygon2dCalculator.orthogonalProjection(closestPolygonIntersectionPoint2d, polygon.getConvexPolygon2d());
      closestPolygonIntersectionPoint.set(closestPolygonIntersectionPoint2d.getX(), closestPolygonIntersectionPoint2d.getY(), 0.0);
      
      boolean isInsideOrOnSurface = sphere.getSphere3d().isInsideOrOnSurface(closestPolygonIntersectionPoint);
      
      closestPointOnPolygon.setIncludingFrame(polygon.getReferenceFrame(), closestPolygonIntersectionPoint);
      closestPointOnPolygon.changeFrame(ReferenceFrame.getWorldFrame());
      
      return isInsideOrOnSurface;
   }

   public FramePoint getClosestPointOnPolygon()
   {
      return closestPointOnPolygon;
   }
}
