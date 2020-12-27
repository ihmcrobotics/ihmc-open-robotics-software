package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class SegmentPointViewer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double size = 0.0075;
   private static final int maxPoints = 20;

   private final List<YoFramePoint3D> dcmStartPoints = new ArrayList<>();
   private final List<YoFramePoint3D> dcmEndPoints = new ArrayList<>();
   private final List<YoFramePoint3D> comStartPoints = new ArrayList<>();
   private final List<YoFramePoint3D> comEndPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpStartPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpEndPoints = new ArrayList<>();

   private static final String name = "Corner Points";

   public SegmentPointViewer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(false, true, registry, yoGraphicsListRegistry);
   }

   public SegmentPointViewer(boolean artifactsOnly, boolean viewCoM, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (int i = 0; i < maxPoints; i++)
      {
         YoFramePoint3D dcmStartPoint = new YoFramePoint3D("dcmStartPoint" + i, worldFrame, registry);
         YoFramePoint3D dcmEndPoint = new YoFramePoint3D("dcmEndPoint" + i, worldFrame, registry);
         YoFramePoint3D vrpStartPoint = new YoFramePoint3D("vrpStartPoint" + i, worldFrame, registry);
         YoFramePoint3D vrpEndPoint = new YoFramePoint3D("vrpEndPoint" + i, worldFrame, registry);

         dcmStartPoint.setToNaN();
         dcmEndPoint.setToNaN();
         vrpStartPoint.setToNaN();
         vrpEndPoint.setToNaN();

         dcmStartPoints.add(dcmStartPoint);
         dcmEndPoints.add(dcmEndPoint);
         vrpStartPoints.add(vrpStartPoint);
         vrpEndPoints.add(vrpEndPoint);

         YoGraphicPosition dcmStartPointGraphic = new YoGraphicPosition("dcmStartPoint" + i, dcmEndPoint, size, YoAppearance.Blue(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition dcmEndPointGraphic = new YoGraphicPosition("dcmEndPoint" + i, dcmEndPoint, size, YoAppearance.Blue(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition vrpStartPointGraphic = new YoGraphicPosition("vrpStartPoint" + i, vrpStartPoint, size, YoAppearance.Green(), GraphicType.BALL);
         YoGraphicPosition vrpEndPointGraphic = new YoGraphicPosition("vrpEndPoint" + i, vrpEndPoint, size, YoAppearance.Green(), GraphicType.SOLID_BALL);

         if (!artifactsOnly)
         {
            yoGraphicsListRegistry.registerYoGraphic(name, dcmStartPointGraphic);
            yoGraphicsListRegistry.registerYoGraphic(name, dcmEndPointGraphic);
            yoGraphicsListRegistry.registerYoGraphic(name, vrpStartPointGraphic);
            yoGraphicsListRegistry.registerYoGraphic(name, vrpEndPointGraphic);
         }

         yoGraphicsListRegistry.registerArtifact(name, dcmStartPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, dcmEndPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, vrpStartPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, vrpEndPointGraphic.createArtifact());

         if (viewCoM)
         {
            YoFramePoint3D comStartPoint = new YoFramePoint3D("comStartPoint" + i, worldFrame, registry);
            YoFramePoint3D comEndPoint = new YoFramePoint3D("comEndPoint" + i, worldFrame, registry);
            comStartPoint.setToNaN();
            comEndPoint.setToNaN();
            comStartPoints.add(comStartPoint);
            comEndPoints.add(comEndPoint);
            YoGraphicPosition comStartPointGraphic = new YoGraphicPosition("comStartPoint" + i, comStartPoint, size, YoAppearance.Black(), GraphicType.BALL);
            YoGraphicPosition comEndPointGraphic = new YoGraphicPosition("comEndPoint" + i, comStartPoint, size, YoAppearance.Black(), GraphicType.SOLID_BALL);
            yoGraphicsListRegistry.registerArtifact(name, comStartPointGraphic.createArtifact());
            yoGraphicsListRegistry.registerArtifact(name, comEndPointGraphic.createArtifact());

            if (!artifactsOnly)
            {
               yoGraphicsListRegistry.registerYoGraphic(name, comStartPointGraphic);
               yoGraphicsListRegistry.registerYoGraphic(name, comEndPointGraphic);
            }
         }


      }
   }

   public void updateDCMCornerPoints(List<LineSegment3D> dcmCornerPoints)
   {
      int i = 0;
      int size = Math.min(dcmCornerPoints.size(), this.dcmStartPoints.size());

      for (; i < size; i++)
      {
         this.dcmStartPoints.get(i).set(dcmCornerPoints.get(i).getFirstEndpoint());
         this.dcmEndPoints.get(i).set(dcmCornerPoints.get(i).getSecondEndpoint());
      }

      for (; i < this.dcmStartPoints.size(); i++)
      {
         this.dcmStartPoints.get(i).setToNaN();
         this.dcmEndPoints.get(i).setToNaN();
      }
   }

   public void updateCoMCornerPoints(List<LineSegment3D> comCornerPoints)
   {
      int i = 0;
      int size = Math.min(comCornerPoints.size(), this.comStartPoints.size());

      for (; i < size; i++)
      {
         this.comStartPoints.get(i).set(comCornerPoints.get(i).getFirstEndpoint());
         this.comEndPoints.get(i).set(comCornerPoints.get(i).getSecondEndpoint());
      }

      for (; i < this.comStartPoints.size(); i++)
      {
         this.comStartPoints.get(i).setToNaN();
         this.comEndPoints.get(i).setToNaN();
      }
   }

   public void updateVRPWaypoints(List<LineSegment3D> vrpSegments)
   {
      int i = 0;
      int size = Math.min(vrpSegments.size(), this.vrpStartPoints.size());

      for (; i < size; i++)
      {
         vrpStartPoints.get(i).set(vrpSegments.get(i).getFirstEndpoint());
         vrpEndPoints.get(i).set(vrpSegments.get(i).getSecondEndpoint());
      }

      for (; i < vrpStartPoints.size(); i++)
      {
         vrpStartPoints.get(i).setToNaN();
         vrpEndPoints.get(i).setToNaN();
      }

   }
}
