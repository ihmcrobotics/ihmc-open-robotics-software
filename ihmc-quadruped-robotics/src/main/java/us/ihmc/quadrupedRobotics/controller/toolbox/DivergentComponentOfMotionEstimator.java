package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Blue;

public class DivergentComponentOfMotionEstimator
{
   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint3D yoDcmPositionEstimate = new YoFramePoint3D("dcmPositionEstimate", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoIcpPositionEstimate = new YoFramePoint3D("icpPositionEstimate", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D centerOfMass = new YoFramePoint3D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);

   private final FramePoint3D tempPoint = new FramePoint3D();

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

      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition("Capture Point" + suffix , yoIcpPositionEstimate, 0.01, Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition dcmPositionEstimateViz = new YoGraphicPosition("Divergent Component of Motion" + suffix , yoDcmPositionEstimate, 0.01, Blue(), GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition comEstimateViz = new YoGraphicPosition("Center of Mass" + suffix , centerOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);

      yoGraphicsList.add(yoIcpPositionEstimateViz);
      yoGraphicsList.add(dcmPositionEstimateViz);
      yoGraphicsList.add(comEstimateViz);

      artifactList.add(yoIcpPositionEstimateViz.createArtifact());
      artifactList.add(comEstimateViz.createArtifact());

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      graphicsListRegistry.registerArtifactList(artifactList);
   }

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return lipModel;
   }


   public void compute(FrameVector3D comVelocityEstimate)
   {
      compute(tempPoint, comVelocityEstimate);
   }
   public void compute(FramePoint3D dcmPositionEstimateToPack, FrameVector3D comVelocityEstimate)
   {
      ReferenceFrame dcmPositionEstimateFrame = dcmPositionEstimateToPack.getReferenceFrame();
      ReferenceFrame comVelocityEstimateFrame = comVelocityEstimate.getReferenceFrame();
      dcmPositionEstimateToPack.changeFrame(comZUpFrame);
      comVelocityEstimate.changeFrame(comZUpFrame);

      dcmPositionEstimateToPack.set(comVelocityEstimate);
      dcmPositionEstimateToPack.scale(1.0 / lipModel.getNaturalFrequency());
      yoDcmPositionEstimate.setMatchingFrame(dcmPositionEstimateToPack);
      yoIcpPositionEstimate.set(yoDcmPositionEstimate);
      yoIcpPositionEstimate.subZ(lipModel.getLipmHeight());
      centerOfMass.setFromReferenceFrame(comZUpFrame);

      dcmPositionEstimateToPack.changeFrame(dcmPositionEstimateFrame);
      comVelocityEstimate.changeFrame(comVelocityEstimateFrame);
   }

   public void getDCMPositionEstimate(FramePoint3D dcmPositionEstimateToPack)
   {
      dcmPositionEstimateToPack.setIncludingFrame(yoDcmPositionEstimate);
   }

   public FramePoint3DReadOnly getDCMPositionEstimate()
   {
      return yoDcmPositionEstimate;
   }
}
