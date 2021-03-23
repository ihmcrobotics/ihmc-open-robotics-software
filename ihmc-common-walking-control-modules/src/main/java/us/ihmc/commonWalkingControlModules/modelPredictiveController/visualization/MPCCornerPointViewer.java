package us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.LinearMPCTrajectoryHandler;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLongTime;

public class MPCCornerPointViewer
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

   public MPCCornerPointViewer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(false, true, registry, yoGraphicsListRegistry);
   }

   public MPCCornerPointViewer(boolean artifactsOnly, boolean viewCoM, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
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

         YoGraphicPosition dcmStartPointGraphic = new YoGraphicPosition("dcmStartPoint" + i,
                                                                        dcmEndPoint,
                                                                        size,
                                                                        YoAppearance.Blue(),
                                                                        GraphicType.BALL_WITH_CROSS);
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

   public void updateCornerPoints(LinearMPCTrajectoryHandler trajectoryHandler, List<? extends ContactStateProvider> contactSequence)
   {
      int segmentId = 0;
      for (; segmentId < Math.min(contactSequence.size(), comStartPoints.size()); segmentId++)
      {
         double startTime = contactSequence.get(segmentId).getTimeInterval().getStartTime();
         double endTime = Math.min(contactSequence.get(segmentId).getTimeInterval().getEndTime(), startTime + sufficientlyLongTime);

         startTime += 1e-6;
         endTime -= 1e-6;

         trajectoryHandler.compute(startTime);
         comStartPoints.get(segmentId).set(trajectoryHandler.getDesiredCoMPosition());
         dcmStartPoints.get(segmentId).set(trajectoryHandler.getDesiredDCMPosition());
         vrpStartPoints.get(segmentId).set(trajectoryHandler.getDesiredVRPPosition());

         trajectoryHandler.compute(endTime);
         comEndPoints.get(segmentId).set(trajectoryHandler.getDesiredCoMPosition());
         dcmEndPoints.get(segmentId).set(trajectoryHandler.getDesiredDCMPosition());
         vrpEndPoints.get(segmentId).set(trajectoryHandler.getDesiredVRPPosition());
      }

      for (; segmentId < dcmStartPoints.size(); segmentId++)
      {
         comStartPoints.get(segmentId).setToNaN();
         comEndPoints.get(segmentId).setToNaN();
         dcmStartPoints.get(segmentId).setToNaN();
         dcmEndPoints.get(segmentId).setToNaN();
         vrpStartPoints.get(segmentId).setToNaN();
         vrpEndPoints.get(segmentId).setToNaN();
      }
   }
}
