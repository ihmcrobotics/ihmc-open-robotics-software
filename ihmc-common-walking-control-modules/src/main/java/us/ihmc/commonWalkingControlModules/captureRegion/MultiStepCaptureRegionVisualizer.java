package us.ihmc.commonWalkingControlModules.captureRegion;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;

public class MultiStepCaptureRegionVisualizer
{
   private static final double size = 0.005;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultiStepCaptureRegionCalculator captureRegionCalculator;
   private final Runnable updateCallback;

   private final YoFrameConvexPolygon2D yoOneStepRegion;
   private final YoFramePoint2D firstPointToExtrude = new YoFramePoint2D("firstPointToExtrude", worldFrame, registry);
   private final YoFramePoint2D secondPointToExtrude = new YoFramePoint2D("secondPointToExtrude", worldFrame, registry);
   private final YoFramePoint2D extrudedFirstPoint = new YoFramePoint2D("extrudedFirstPoint", worldFrame, registry);
   private final YoFramePoint2D extrudedSecondPoint = new YoFramePoint2D("extrudedSecondPoint", worldFrame, registry);
   private final YoFramePoint2D extrusionCenter = new YoFramePoint2D("extrusionCenter", worldFrame, registry);
   private final YoFramePoint2D extrusionCenterEnd = new YoFramePoint2D("extrusionCenterEnd", worldFrame, registry);

   public MultiStepCaptureRegionVisualizer(MultiStepCaptureRegionCalculator captureRegionCalculator,
                                           Runnable updateCallback,
                                           YoRegistry parentRegistry,
                                           YoGraphicsListRegistry graphicsListRegistry)
   {
      this.captureRegionCalculator = captureRegionCalculator;
      this.updateCallback = updateCallback;

      yoOneStepRegion = new YoFrameConvexPolygon2D("oneStepRegion", worldFrame, 10, registry);

      YoArtifactPolygon oneStepRegionGraphic = new YoArtifactPolygon("oneStepRegionnnnn", yoOneStepRegion, Color.green, false);
      YoGraphicPosition firstPointToExtrudeGraphic = new YoGraphicPosition("firstPointToExtrudeGraphic", firstPointToExtrude, size, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition secondtPointToExtrudeGraphic = new YoGraphicPosition("secondPointToExtrudeGraphic", secondPointToExtrude, size, YoAppearance.Blue(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition extrudedFirstPointGraphic = new YoGraphicPosition("extrudedFirstPointGraphic", extrudedFirstPoint, size, YoAppearance.Red(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition extrudedSecondPointGraphic = new YoGraphicPosition("extrudedSecondPointGraphic", extrudedSecondPoint, size, YoAppearance.Red(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition extrusionCenterGraphic = new YoGraphicPosition("extrusionCenterGraphic", extrusionCenter, size, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);

      YoArtifactLineSegment2d extrusionArrow = new YoArtifactLineSegment2d("extrusionArrowGraphic", extrusionCenter, extrusionCenterEnd, Color.BLUE);

      graphicsListRegistry.registerArtifact("test2", oneStepRegionGraphic);
      graphicsListRegistry.registerArtifact("test2", firstPointToExtrudeGraphic.createArtifact());
      graphicsListRegistry.registerArtifact("test2", secondtPointToExtrudeGraphic.createArtifact());
      graphicsListRegistry.registerArtifact("test2", extrudedFirstPointGraphic.createArtifact());
      graphicsListRegistry.registerArtifact("test2", extrudedSecondPointGraphic.createArtifact());
      graphicsListRegistry.registerArtifact("test2", extrusionCenterGraphic.createArtifact());
      graphicsListRegistry.registerArtifact("test2", extrusionArrow);

      parentRegistry.addChild(registry);
   }

   public void updateInputs(FrameConvexPolygon2DReadOnly oneStepCaptureRegion)
   {
      yoOneStepRegion.setMatchingFrame(oneStepCaptureRegion, false);

      updateCallback.run();
   }

   public void visualizeProcess(FramePoint2DReadOnly extrudedFirstPoint,
                                FramePoint2DReadOnly extrudedSecondPoint,
                                FrameLineSegment2DReadOnly edgeToExtrude,
                                FrameConvexPolygon2DReadOnly oneStepCaptureRegion,
                                int currentVertex)
   {
      firstPointToExtrude.setMatchingFrame(edgeToExtrude.getFirstEndpoint());
      secondPointToExtrude.setMatchingFrame(edgeToExtrude.getSecondEndpoint());
      FrameVector2D perp = new FrameVector2D(edgeToExtrude.getReferenceFrame());
      edgeToExtrude.perpendicular(true, perp);
      perp.changeFrame(worldFrame);

      extrusionCenter.setMatchingFrame(edgeToExtrude.midpoint());
      extrusionCenterEnd.set(extrusionCenter);
      extrusionCenterEnd.add(perp);

      this.extrudedFirstPoint.setMatchingFrame(extrudedFirstPoint);
      this.extrudedSecondPoint.setMatchingFrame(extrudedSecondPoint);

      updateCallback.run();
   }
}
