package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.ParabolicCartesianTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DivergentComponentOfMotionEstimator
{
   private final ReferenceFrame comZUpFrame;
   private final LinearInvertedPendulumModel lipModel;

   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
   ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

   YoFramePoint yoDcmPositionEstimate = new YoFramePoint("dcmPositionEstimate", ReferenceFrame.getWorldFrame(), registry);
   YoFramePoint yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", ReferenceFrame.getWorldFrame(), registry);

   public DivergentComponentOfMotionEstimator(ReferenceFrame comZUpFrame, LinearInvertedPendulumModel lipModel, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.comZUpFrame = comZUpFrame;
      this.lipModel = lipModel;

      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition("icpPositionEstimate", yoIcpPositionEstimate, 0.025, YoAppearance.Magenta());
      yoGraphicsList.add(yoIcpPositionEstimateViz);
      artifactList.add(yoIcpPositionEstimateViz.createArtifact());
      parentRegistry.addChild(registry);
      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      graphicsListRegistry.registerArtifactList(artifactList);
   }

   public LinearInvertedPendulumModel getLinearInvertedPendulumModel()
   {
      return lipModel;
   }

   public void compute(FramePoint dcmPositionEstimate, FrameVector comVelocityEstimate)
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
