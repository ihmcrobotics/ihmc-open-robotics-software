package us.ihmc.footstepPlanning.baselinePlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SamplingPoseTrajectoryVisualizer
{
   private final PoseTrajectoryGenerator poseTrajectory;
   private final int sampleCount;

   private final BagOfBalls sampleViz;
   private final YoFramePoint3D headPosition;
   private final YoFrameYawPitchRoll headOrientation;
   private final YoFrameVector3D headVelocity;

   private final FramePoint3D tempSamplePosition = new FramePoint3D();

   public SamplingPoseTrajectoryVisualizer(String prefix, PoseTrajectoryGenerator poseTrajectory, ReferenceFrame frame, int sampleCount, double drawSize,
                                           boolean displayVelocityVector, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.poseTrajectory = poseTrajectory;
      this.sampleCount = sampleCount;

      List<AppearanceDefinition> appearances = new ArrayList<>(sampleCount);
      for (int i = 0; i < sampleCount; i++)
      {
         appearances.add(new YoAppearanceRGBColor((double) i / sampleCount, 0.0, (double) (sampleCount - i) / sampleCount, 0.0));
      }
      this.sampleViz = new BagOfBalls(drawSize, prefix + "SampleViz", appearances, registry, graphicsListRegistry);

      this.headPosition = new YoFramePoint3D("headPosition", frame, registry);
      this.headOrientation = new YoFrameYawPitchRoll("headOrientation", frame, registry);
      this.headVelocity = new YoFrameVector3D("headVelocity", frame, registry);

      YoGraphicCoordinateSystem headFrameViz = new YoGraphicCoordinateSystem("headFrameFix", headPosition, headOrientation, drawSize * 10.0);
      YoGraphicVector headVelocityViz = new YoGraphicVector("headVelocityViz", headPosition, headVelocity, drawSize * 100, YoAppearance.Chartreuse());

      YoGraphicsList graphicsList = new YoGraphicsList(prefix + "TrajectoryHead");
      graphicsList.add(headFrameViz);
      if (displayVelocityVector)
      {
         graphicsList.add(headVelocityViz);
      }

      graphicsListRegistry.registerYoGraphicsList(graphicsList);
   }

   public void redraw(double t0, double tf)
   {
      double dt = (tf - t0) / (sampleCount - 1);
      for (int sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++)
      {
         poseTrajectory.compute(t0 + sampleIdx * dt);
         sampleViz.setBall(poseTrajectory.getPosition(), sampleIdx);
      }

      poseTrajectory.compute(tf);
      headPosition.set(poseTrajectory.getPosition());
      headOrientation.set(poseTrajectory.getOrientation());
      headVelocity.set(poseTrajectory.getVelocity());
   }

   public void hide()
   {
      for (int sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++)
      {
         headPosition.set(0.0, 0.0, -100.0);
         tempSamplePosition.set(0.0, 0.0, -100.0);
         sampleViz.setBall(tempSamplePosition, sampleIdx);
      }
   }
}
