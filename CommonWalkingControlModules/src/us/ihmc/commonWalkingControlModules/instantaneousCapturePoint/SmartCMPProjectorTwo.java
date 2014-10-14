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
   private final BooleanYoVariable cmpProjectedAlongRay, cmpProjectedOrthogonally;
   private final FrameLine2d icpToCMPLine = new FrameLine2d(ReferenceFrame.getWorldFrame(), new Point2d(), new Point2d(1.0, 0.0));

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SmartCMPProjectorTwo(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      cmpProjectedAlongRay = new BooleanYoVariable("cmpProjectedAlongRay", registry);
      cmpProjectedOrthogonally = new BooleanYoVariable("cmpProjectedOrthogonally", registry);


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
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, FramePoint2d desiredCMP)
   {
      ReferenceFrame returnFrame = desiredCMP.getReferenceFrame();

      desiredCMP.changeFrame(supportPolygon.getReferenceFrame());
      capturePoint.changeFrame(supportPolygon.getReferenceFrame());

      projectCMPIntoSupportPolygonIfOutsideLocal(capturePoint, supportPolygon, desiredCMP);

      desiredCMP.changeFrame(returnFrame);
      capturePoint.changeFrame(returnFrame);
   }

   private void projectCMPIntoSupportPolygonIfOutsideLocal(FramePoint2d capturePoint, FrameConvexPolygon2d supportPolygon, FramePoint2d desiredCMP)
   {
      cmpProjectedAlongRay.set(false);
      cmpProjectedOrthogonally.set(false);

      if (supportPolygon.isPointInside(desiredCMP))
         return;

      icpToCMPLine.setAndChangeFrame(capturePoint, desiredCMP);
      FramePoint2d[] icpToCMPIntersections = supportPolygon.intersectionWithRay(icpToCMPLine);

      if ((icpToCMPIntersections == null) || (icpToCMPIntersections.length == 0))
      {
         supportPolygon.orthogonalProjection(desiredCMP);
         cmpProjectedOrthogonally.set(true);

         return;
      }

      FramePoint2d closestIntersection = null;
      double closestDistanceSquared = Double.POSITIVE_INFINITY;
      for (FramePoint2d framePoint2d : icpToCMPIntersections)
      {
         double distanceSquared = framePoint2d.distanceSquared(desiredCMP);
         if (distanceSquared < closestDistanceSquared)
         {
            closestIntersection = framePoint2d;
            closestDistanceSquared = distanceSquared;
         }
      }

      cmpProjectedAlongRay.set(true);
      desiredCMP.set(closestIntersection);
   }



   public boolean getWasCMPProjected()
   {
      return (cmpProjectedAlongRay.getBooleanValue() || cmpProjectedOrthogonally.getBooleanValue());
   }


   public void setCMPEdgeProjectionInside(double cmpEdgeProjectionInside)
   {
      // TODO Auto-generated method stub
      
   }


   public void setMinICPToCMPProjection(double minICPToCMPProjection)
   {
      // TODO Auto-generated method stub
      
   }

}
