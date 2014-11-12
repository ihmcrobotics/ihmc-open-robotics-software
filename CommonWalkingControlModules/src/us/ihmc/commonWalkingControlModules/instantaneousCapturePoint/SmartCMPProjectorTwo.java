package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Point2d;

import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;


public class SmartCMPProjectorTwo
{
   private final BooleanYoVariable cmpProjectedAlongRay, cmpProjectedToPushTowardSwingFoot, cmpProjectedToVertex;
   private final FrameLine2d icpToCMPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2d(), new Point2d(1.0, 0.0));
   private final FrameLine2d swingSoleToICPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2d(), new Point2d(1.0, 0.0));
   private final FramePoint2d swingSoleLocation = new FramePoint2d();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SmartCMPProjectorTwo(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      cmpProjectedAlongRay = new BooleanYoVariable("cmpProjectedAlongRay", registry);
      cmpProjectedToPushTowardSwingFoot = new BooleanYoVariable("cmpProjectedToPushTowardSwingFoot", registry);
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
    * Therefore the best to do without thinking about where to step is to just project the CMP into the foot polygon.
    */
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, ReferenceFrame swingSoleFrame, FramePoint2d desiredCMP)
   {
      ReferenceFrame returnFrame = desiredCMP.getReferenceFrame();

      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());
      capturePoint.changeFrame(supportPolygon.getReferenceFrame());

      projectCMPIntoSupportPolygonIfOutsideLocal(capturePoint, supportPolygon, swingSoleFrame, desiredCMP);

      desiredCMP.changeFrame(returnFrame);
      capturePoint.changeFrame(returnFrame);
   }

   
   
   private void projectCMPIntoSupportPolygonIfOutsideLocal(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, ReferenceFrame swingSoleFrame, FramePoint2d desiredCMP)
   {
      cmpProjectedAlongRay.set(false);
      cmpProjectedToPushTowardSwingFoot.set(false);
      cmpProjectedToVertex.set(false);

      if (supportPolygon.isPointInside(desiredCMP))
         return;

      icpToCMPLine.setIncludingFrame(capturePoint, desiredCMP);
      FramePoint2d[] icpToCMPIntersections = supportPolygon.intersectionWithRay(icpToCMPLine);

      if ((icpToCMPIntersections != null) && (icpToCMPIntersections.length > 0))
      {
         cmpProjectedAlongRay.set(true);

         FramePoint2d closestIntersection = findClosestIntersection(icpToCMPIntersections, desiredCMP);
         desiredCMP.set(closestIntersection);
         return;
      }

      if (swingSoleFrame != null)
      {
         swingSoleLocation.setToZero(swingSoleFrame);
         swingSoleLocation.changeFrameAndProjectToXYPlane(supportPolygon.getReferenceFrame());
         swingSoleToICPLine.setIncludingFrame(swingSoleLocation, capturePoint);

         FramePoint2d[] swingSoleToICPIntersections = supportPolygon.intersectionWith(swingSoleToICPLine);

         if ((swingSoleToICPIntersections != null) && (swingSoleToICPIntersections.length >= 0))
         {
            cmpProjectedToPushTowardSwingFoot.set(true);

            FramePoint2d closestIntersection = findClosestIntersection(swingSoleToICPIntersections, capturePoint);
            desiredCMP.set(closestIntersection);
            return;
         }

         FramePoint2d closestVertex = supportPolygon.getClosestVertexCopy(swingSoleToICPLine);
         desiredCMP.set(closestVertex);

         cmpProjectedToVertex.set(true);
         return;
      }
      
      supportPolygon.orthogonalProjection(desiredCMP);
   }

   public FramePoint2d findClosestIntersection(FramePoint2d[] potentialIntersections, FramePoint2d closestToPoint)
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

   public boolean getWasCMPProjected()
   {
      return (cmpProjectedAlongRay.getBooleanValue() || cmpProjectedToVertex.getBooleanValue());
   }


   public void setCMPEdgeProjectionInside(double cmpEdgeProjectionInside)
   {      
   }


   public void setMinICPToCMPProjection(double minICPToCMPProjection)
   {      
   }

}
