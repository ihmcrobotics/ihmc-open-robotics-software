package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CornerPointViewer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double size = 0.01;
   private static final int maxPoints = 20;

   private final List<YoFramePoint3D> dcmCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> comCornerPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpStartPoints = new ArrayList<>();
   private final List<YoFramePoint3D> vrpEndPoints = new ArrayList<>();

   private static final String name = "Corner Points";

   public CornerPointViewer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (int i = 0; i < maxPoints; i++)
      {
         YoFramePoint3D dcmCornerPoint = new YoFramePoint3D("dcmCornerPoint" + i, worldFrame, registry);
         YoFramePoint3D comCornerPoint = new YoFramePoint3D("comCornerPoint" + i, worldFrame, registry);
         YoFramePoint3D vrpStartPoint = new YoFramePoint3D("vrpStartPoint" + i, worldFrame, registry);
         YoFramePoint3D vrpEndPoint = new YoFramePoint3D("vrpEndPoint" + i, worldFrame, registry);

         dcmCornerPoint.setToNaN();
         comCornerPoint.setToNaN();
         vrpStartPoint.setToNaN();
         vrpEndPoint.setToNaN();

         dcmCornerPoints.add(dcmCornerPoint);
         comCornerPoints.add(comCornerPoint);
         vrpStartPoints.add(vrpStartPoint);
         vrpEndPoints.add(vrpEndPoint);

         YoGraphicPosition dcmCornerPointGraphic = new YoGraphicPosition("dcmCornerPoint" + i, dcmCornerPoint, size, YoAppearance.Blue(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition comCornerPointGraphic = new YoGraphicPosition("comCornerPoint" + i, comCornerPoint, size, YoAppearance.Black(), GraphicType.SOLID_BALL);
         YoGraphicPosition vrpStartPointGraphic = new YoGraphicPosition("vrpStartPoint" + i, vrpStartPoint, size, YoAppearance.Green(), GraphicType.BALL);
         YoGraphicPosition vrpEndPointGraphic = new YoGraphicPosition("vrpEndPoint" + i, vrpEndPoint, size, YoAppearance.Green(), GraphicType.SOLID_BALL);

         yoGraphicsListRegistry.registerYoGraphic(name, dcmCornerPointGraphic);
         yoGraphicsListRegistry.registerYoGraphic(name, comCornerPointGraphic);
         yoGraphicsListRegistry.registerYoGraphic(name, vrpStartPointGraphic);
         yoGraphicsListRegistry.registerYoGraphic(name, vrpEndPointGraphic);

         yoGraphicsListRegistry.registerArtifact(name, dcmCornerPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, comCornerPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, vrpStartPointGraphic.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, vrpEndPointGraphic.createArtifact());
      }
   }

   public void updateDCMCornerPoints(List<FramePoint3D> dcmCornerPoints)
   {
      for (int i = 0; i < this.dcmCornerPoints.size(); i++)
         this.dcmCornerPoints.get(i).setToNaN();

      for (int i = 0; i < dcmCornerPoints.size(); i++)
         this.dcmCornerPoints.get(i).set(dcmCornerPoints.get(i));
   }

   public void updateCoMCornerPoints(List<FramePoint3D> comCornerPoints)
   {
      for (int i = 0; i < this.comCornerPoints.size(); i++)
         this.comCornerPoints.get(i).setToNaN();

      for (int i = 0; i < comCornerPoints.size(); i++)
         this.comCornerPoints.get(i).set(comCornerPoints.get(i));
   }

   public void updateVRPWaypoints(List<LineSegment3D> vrpSegments)
   {
      for (int i = 0; i < vrpStartPoints.size(); i++)
      {
         vrpStartPoints.get(i).setToNaN();
         vrpEndPoints.get(i).setToNaN();
      }

      for (int i = 0; i < vrpSegments.size(); i++)
      {
         vrpStartPoints.get(i).set(vrpSegments.get(i).getFirstEndpoint());
         vrpEndPoints.get(i).set(vrpSegments.get(i).getSecondEndpoint());
      }
   }
}
