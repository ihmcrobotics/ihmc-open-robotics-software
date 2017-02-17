package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SmartCMPProjectorTwo extends CMPProjector
{
   private final BooleanYoVariable cmpProjectedAlongRay, cmpProjectedToPushTowardFinalDesiredICP, cmpProjectedToVertex;
   private final FrameLine2d icpToCMPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2D(), new Point2D(1.0, 0.0));
   private final FrameVector2d finalDesiredICPToICPDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());
   private final FrameLine2d rayFromICPAwayFromFinalDesiredICP = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2D(), new Point2D(1.0, 0.0));
   private final FramePoint2d finalDesiredICPLocation = new FramePoint2d();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BoundingBox2d tempBoundingBox = new BoundingBox2d();
   private final FramePoint2d intersection1 = new FramePoint2d();
   private final FramePoint2d intersection2 = new FramePoint2d();
   private final BooleanYoVariable cmpWasProjected = new BooleanYoVariable("CmpWasProjected", registry);

   public SmartCMPProjectorTwo(YoVariableRegistry parentRegistry)
   {
      cmpProjectedAlongRay = new BooleanYoVariable("cmpProjectedAlongRay", registry);
      cmpProjectedToPushTowardFinalDesiredICP = new BooleanYoVariable("cmpProjectedToPushTowardFinalDesiredICP", registry);
      cmpProjectedToVertex = new BooleanYoVariable("cmpProjectedToVertex", registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   /**
    * Project the CMP to the support polygon by moving it along the ray from ICP through CMP.
    * Where that ray intersects the support polygon, will be a good point since that point will still
    * push the ICP in the same direction as before, just with a different velocity.
    * If there is no intersection, then that means that both ICP and CMP are outside the support polygon
    * and we can no longer push the ICP in the originally intended direction.
    * Therefore we try to direct the ICP towards the final desired ICP location (almost equivalent to the next footstep location).
    */
   @Override
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, FramePoint2d finalDesiredCapturePoint,
         FramePoint2d desiredCMP)
   {
      ReferenceFrame returnFrame = desiredCMP.getReferenceFrame();

      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());
      if (finalDesiredCapturePoint != null)
      {
         finalDesiredCapturePoint.changeFrame(supportPolygon.getReferenceFrame());
      }
      capturePoint.changeFrame(supportPolygon.getReferenceFrame());

      projectCMPIntoSupportPolygonIfOutsideLocal(capturePoint, supportPolygon, finalDesiredCapturePoint, desiredCMP);

      desiredCMP.changeFrame(returnFrame);
      capturePoint.changeFrame(returnFrame);
   }

   private void projectCMPIntoSupportPolygonIfOutsideLocal(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2d finalDesiredCapturePoint, FramePoint2d desiredCMP)
   {
      cmpProjectedAlongRay.set(false);
      cmpProjectedToPushTowardFinalDesiredICP.set(false);
      cmpProjectedToVertex.set(false);

      supportPolygon.getBoundingBox(tempBoundingBox);
      double diagonalLengthSquared = tempBoundingBox.getDiagonalLengthSquared();

      if (diagonalLengthSquared < 0.01 * 0.01)
      {
         supportPolygon.getCentroid(desiredCMP);
         return;
      }

      if (supportPolygon.isPointInside(desiredCMP))
         return;

      icpToCMPLine.setIncludingFrame(capturePoint, desiredCMP);
      int intersections = supportPolygon.intersectionWithRay(icpToCMPLine, intersection1, intersection2);

      if (intersections == 1)
      {
         cmpProjectedAlongRay.set(true);
         desiredCMP.set(intersection1);
         return;
      }
      else if (intersections == 2)
      {
         cmpProjectedAlongRay.set(true);
         desiredCMP.set(findClosestIntersection(desiredCMP, intersection1, intersection2));
         return;
      }

      if (finalDesiredCapturePoint != null)
      {
         finalDesiredICPLocation.setIncludingFrame(finalDesiredCapturePoint);
         rayFromICPAwayFromFinalDesiredICP.setIncludingFrame(finalDesiredICPLocation, capturePoint);

         finalDesiredICPToICPDirection.setIncludingFrame(capturePoint);
         finalDesiredICPToICPDirection.sub(finalDesiredICPLocation);
         rayFromICPAwayFromFinalDesiredICP.setIncludingFrame(capturePoint, finalDesiredICPToICPDirection);

         FramePoint2d[] finalDesiredICPToICPIntersections = supportPolygon.intersectionWith(rayFromICPAwayFromFinalDesiredICP);

         if (finalDesiredICPToICPIntersections != null && finalDesiredICPToICPIntersections.length > 1)
         {
            cmpProjectedToPushTowardFinalDesiredICP.set(true);
            FramePoint2d closestIntersection = findClosestIntersection(capturePoint, finalDesiredICPToICPIntersections);
            desiredCMP.set(closestIntersection);
            return;
         }

         cmpProjectedToVertex.set(true);
         boolean success = supportPolygon.getClosestPointWithRay(desiredCMP, rayFromICPAwayFromFinalDesiredICP);
         if (!success)
            supportPolygon.getClosestVertex(desiredCMP, capturePoint);
         return;
      }

      supportPolygon.orthogonalProjection(desiredCMP);
   }

   private FramePoint2d findClosestIntersection(FramePoint2d closestToPoint, FramePoint2d... potentialIntersections)
   {
      FramePoint2d closestIntersection = null;
      double closestDistanceSquared = Double.POSITIVE_INFINITY;
      for (FramePoint2d framePoint2d : potentialIntersections)
      {
         double distanceSquared = framePoint2d.distanceSquared(closestToPoint);
         if (distanceSquared < closestDistanceSquared)
         {
            closestIntersection = framePoint2d;
            closestDistanceSquared = distanceSquared;
         }
      }

      return closestIntersection;
   }

   @Override
   public boolean getWasCMPProjected()
   {
      if (cmpWasProjected.getBooleanValue())
         return true;

      return (cmpProjectedAlongRay.getBooleanValue() || cmpProjectedToVertex.getBooleanValue());
   }

   public void setCMPEdgeProjectionInside(double cmpEdgeProjectionInside)
   {
   }

   public void setMinICPToCMPProjection(double minICPToCMPProjection)
   {
   }

}
