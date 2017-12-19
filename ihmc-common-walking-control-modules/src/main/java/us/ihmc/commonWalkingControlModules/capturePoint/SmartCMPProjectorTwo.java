package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;

public class SmartCMPProjectorTwo extends CMPProjector
{
   private final YoBoolean cmpProjectedAlongRay, cmpProjectedToPushTowardFinalDesiredICP, cmpProjectedToVertex;
   private final FrameLine2d icpToCMPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2D(), new Point2D(1.0, 0.0));
   private final FrameVector2D finalDesiredICPToICPDirection = new FrameVector2D(ReferenceFrame.getWorldFrame());
   private final FrameLine2d rayFromICPAwayFromFinalDesiredICP = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2D(), new Point2D(1.0, 0.0));
   private final FramePoint2D finalDesiredICPLocation = new FramePoint2D();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BoundingBox2D tempBoundingBox = new BoundingBox2D();
   private final FramePoint2D intersection1 = new FramePoint2D();
   private final FramePoint2D intersection2 = new FramePoint2D();
   private final YoBoolean cmpWasProjected = new YoBoolean("CmpWasProjected", registry);

   public SmartCMPProjectorTwo(YoVariableRegistry parentRegistry)
   {
      cmpProjectedAlongRay = new YoBoolean("cmpProjectedAlongRay", registry);
      cmpProjectedToPushTowardFinalDesiredICP = new YoBoolean("cmpProjectedToPushTowardFinalDesiredICP", registry);
      cmpProjectedToVertex = new YoBoolean("cmpProjectedToVertex", registry);

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
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2D capturePoint, FrameConvexPolygon2d supportPolygon, FramePoint2D finalDesiredCapturePoint,
         FramePoint2D desiredCMP)
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

   private void projectCMPIntoSupportPolygonIfOutsideLocal(FramePoint2D capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2D finalDesiredCapturePoint, FramePoint2D desiredCMP)
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

         FramePoint2D[] finalDesiredICPToICPIntersections = supportPolygon.intersectionWith(rayFromICPAwayFromFinalDesiredICP);

         if (finalDesiredICPToICPIntersections != null && finalDesiredICPToICPIntersections.length > 1)
         {
            cmpProjectedToPushTowardFinalDesiredICP.set(true);
            FramePoint2D closestIntersection = findClosestIntersection(capturePoint, finalDesiredICPToICPIntersections);
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

   private FramePoint2D findClosestIntersection(FramePoint2D closestToPoint, FramePoint2D... potentialIntersections)
   {
      FramePoint2D closestIntersection = null;
      double closestDistanceSquared = Double.POSITIVE_INFINITY;
      for (FramePoint2D framePoint2d : potentialIntersections)
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
