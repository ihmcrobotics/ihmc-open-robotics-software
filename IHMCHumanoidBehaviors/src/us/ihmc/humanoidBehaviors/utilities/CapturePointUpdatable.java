package us.ihmc.humanoidBehaviors.utilities;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubsrciber;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class CapturePointUpdatable implements Updatable
{
   private final YoFramePoint2d yoCapturePoint;
   private final YoFramePoint2d yoDesiredCapturePoint;
   private final YoFrameConvexPolygon2d yoSupportPolygon;
   
   private final CapturabilityBasedStatusSubsrciber capturabilityBasedStatusSubsrciber;

   public CapturePointUpdatable(CapturabilityBasedStatusSubsrciber capturabilityBasedStatusSubsrciber,
         YoFramePoint2d yoCapturePoint, YoFramePoint2d yoDesiredCapturePoint, YoFrameConvexPolygon2d yoSupportPolygon,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.capturabilityBasedStatusSubsrciber = capturabilityBasedStatusSubsrciber;

      this.yoCapturePoint = yoCapturePoint;
      this.yoDesiredCapturePoint = yoDesiredCapturePoint;
      this.yoSupportPolygon = yoSupportPolygon;

      YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", capturePointViz.createArtifact());
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, YoAppearance.Yellow(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", desiredCapturePointViz.createArtifact());

      YoArtifactPolygon supportPolygonViz = new YoArtifactPolygon("Combined Polygon", yoSupportPolygon, Color.pink, false);
      yoGraphicsListRegistry.registerArtifact("Capturability", supportPolygonViz);
   }

   @Override
   public void update(double time)
   {
      FramePoint2d capturePoint = capturabilityBasedStatusSubsrciber.getCapturePoint();
      if (capturePoint !=null)
      {
         yoCapturePoint.set(capturePoint);
      }

      FramePoint2d desiredCapturePoint = capturabilityBasedStatusSubsrciber.getDesiredCapturePoint();
      if (desiredCapturePoint != null)
      {
         yoDesiredCapturePoint.set(desiredCapturePoint);
      }
      
      FrameConvexPolygon2d supportPolygon = capturabilityBasedStatusSubsrciber.getSupportPolygon();
      if (supportPolygon != null)
      {
         yoSupportPolygon.setFrameConvexPolygon2d(supportPolygon);
      }
   }
}
