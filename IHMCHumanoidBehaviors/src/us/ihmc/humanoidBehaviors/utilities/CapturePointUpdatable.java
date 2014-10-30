package us.ihmc.humanoidBehaviors.utilities;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubsrciber;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class CapturePointUpdatable implements Updatable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("desiredCapturePoint", worldFrame, registry);
   private final YoFramePoint2d yoCapturePoint = new YoFramePoint2d("capturePoint", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoSupportPolygon = new YoFrameConvexPolygon2d("supportPolygon", "", worldFrame, 30, registry);
   private final EnumYoVariable<RobotSide> yoSupportLeg = new EnumYoVariable<>("supportLeg", registry, RobotSide.class, true);
   private final BooleanYoVariable yoDoubleSupport = new BooleanYoVariable("doubleSupport", registry);
   
   private final CapturabilityBasedStatusSubsrciber capturabilityBasedStatusSubsrciber;

   public CapturePointUpdatable(CapturabilityBasedStatusSubsrciber capturabilityBasedStatusSubsrciber, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.capturabilityBasedStatusSubsrciber = capturabilityBasedStatusSubsrciber;

      YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", capturePointViz.createArtifact());
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, YoAppearance.Yellow(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", desiredCapturePointViz.createArtifact());

      YoArtifactPolygon supportPolygonViz = new YoArtifactPolygon("Combined Polygon", yoSupportPolygon, Color.pink, false);
      yoGraphicsListRegistry.registerArtifact("Capturability", supportPolygonViz);

      parentRegistry.addChild(registry);
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

      RobotSide supportLeg = capturabilityBasedStatusSubsrciber.getSupportLeg();
      if (supportLeg != null)
      {
         yoSupportLeg.set(supportLeg);
      }

      Boolean isInDoubleSupport = capturabilityBasedStatusSubsrciber.IsInDoubleSupport();
      if (isInDoubleSupport != null)
      {
         yoDoubleSupport.set(isInDoubleSupport);
         yoSupportLeg.set(null);
      }
   }

   public YoFramePoint2d getYoDesiredCapturePoint()
   {
      return yoDesiredCapturePoint;
   }

   public YoFramePoint2d getYoCapturePoint()
   {
      return yoCapturePoint;
   }

   public YoFrameConvexPolygon2d getYoSupportPolygon()
   {
      return yoSupportPolygon;
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return yoSupportLeg;
   }

   public BooleanYoVariable getYoDoubleSupport()
   {
      return yoDoubleSupport;
   }
}
