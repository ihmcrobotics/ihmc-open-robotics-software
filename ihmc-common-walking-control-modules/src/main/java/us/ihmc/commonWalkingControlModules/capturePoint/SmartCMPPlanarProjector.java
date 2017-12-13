package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class SmartCMPPlanarProjector extends CMPProjector
{
   private final YoBoolean cmpProjected;

   private final FramePoint2D projectedCMP = new FramePoint2D(ReferenceFrame.getWorldFrame());

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public SmartCMPPlanarProjector(YoVariableRegistry parentRegistry)
   {
      cmpProjected = new YoBoolean("cmpProjected", registry);

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   @Override
   public void projectCMPIntoSupportPolygonIfOutside(FramePoint2D capturePoint, FrameConvexPolygon2d supportPolygon,
         FramePoint2D finalDesiredCapturePoint, FramePoint2D desiredCMPToPack)
   {
      ReferenceFrame returnFrame = desiredCMPToPack.getReferenceFrame();

      desiredCMPToPack.changeFrame(supportPolygon.getReferenceFrame());
      capturePoint.changeFrame(supportPolygon.getReferenceFrame());

      projectedCMP.setToZero(supportPolygon.getReferenceFrame());
      projectedCMP.setX(desiredCMPToPack.getX());
      projectedCMP.setY(supportPolygon.getCentroid().getY());

      projectCMPIntoSupportPolygonIfOutsideLocal(supportPolygon, desiredCMPToPack);

      desiredCMPToPack.changeFrame(returnFrame);
      capturePoint.changeFrame(returnFrame);
   }

   private void projectCMPIntoSupportPolygonIfOutsideLocal(FrameConvexPolygon2d supportPolygon, FramePoint2D desiredCMPToPack)
   {
      cmpProjected.set(false);

      if (supportPolygon.getArea() < 1.0e-3)
      {
         supportPolygon.getCentroid(desiredCMPToPack);
         return;
      }

      if (supportPolygon.isPointInside(projectedCMP))
      {
         return;
      }
      else
      {
         supportPolygon.getClosestVertex(projectedCMP, projectedCMP);
         cmpProjected.set(true);
      }

      desiredCMPToPack.setX(projectedCMP.getX());
   }

   @Override
   public boolean getWasCMPProjected()
   {
      return (cmpProjected.getBooleanValue());
   }
}
