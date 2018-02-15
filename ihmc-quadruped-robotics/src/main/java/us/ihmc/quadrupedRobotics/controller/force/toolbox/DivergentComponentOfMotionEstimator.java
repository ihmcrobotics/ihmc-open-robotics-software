package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;

public class DivergentComponentOfMotionEstimator
{
   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint yoDcmPositionEstimate = new YoFramePoint("dcmPositionEstimate", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", ReferenceFrame.getWorldFrame(), registry);

   public DivergentComponentOfMotionEstimator(ReferenceFrame comZUpFrame, LinearInvertedPendulumModel lipModel, YoVariableRegistry parentRegistry,
                                              YoGraphicsListRegistry graphicsListRegistry)
   {
      this(comZUpFrame, lipModel, "", parentRegistry, graphicsListRegistry);
   }

   public DivergentComponentOfMotionEstimator(ReferenceFrame comZUpFrame, LinearInvertedPendulumModel lipModel, String nameSuffix, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.comZUpFrame = comZUpFrame;
      this.lipModel = lipModel;

      parentRegistry.addChild(registry);

      if (graphicsListRegistry != null)
         createGraphics(nameSuffix, graphicsListRegistry);
   }

   private void createGraphics(String suffix, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition("icpPositionEstimate" + suffix , yoIcpPositionEstimate, 0.01, Blue(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      yoGraphicsList.add(yoIcpPositionEstimateViz);
      artifactList.add(yoIcpPositionEstimateViz.createArtifact());
      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      graphicsListRegistry.registerArtifactList(artifactList);
   }

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return lipModel;
   }

   public void compute(FramePoint3D dcmPositionEstimate, FrameVector3D comVelocityEstimate)
   {
      ReferenceFrame dcmPositionEstimateFrame = dcmPositionEstimate.getReferenceFrame();
      ReferenceFrame comVelocityEstimateFrame = comVelocityEstimate.getReferenceFrame();
      dcmPositionEstimate.changeFrame(comZUpFrame);
      comVelocityEstimate.changeFrame(comZUpFrame);

      dcmPositionEstimate.set(comVelocityEstimate);
      dcmPositionEstimate.scale(1.0 / lipModel.getNaturalFrequency());
      yoDcmPositionEstimate.setAndMatchFrame(dcmPositionEstimate);
      yoIcpPositionEstimate.set(yoDcmPositionEstimate);
      yoIcpPositionEstimate.add(0, 0, -lipModel.getComHeight());

      dcmPositionEstimate.changeFrame(dcmPositionEstimateFrame);
      comVelocityEstimate.changeFrame(comVelocityEstimateFrame);
   }
}
