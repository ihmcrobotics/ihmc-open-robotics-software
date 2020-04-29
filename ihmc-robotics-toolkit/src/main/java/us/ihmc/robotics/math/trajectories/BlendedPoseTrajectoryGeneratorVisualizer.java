package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.Axis3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.YoGraphicTrajectory3D.TrajectoryColorType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class BlendedPoseTrajectoryGeneratorVisualizer
{
   private final YoGraphicTrajectory3D trajectoryViz;

   public BlendedPoseTrajectoryGeneratorVisualizer(String namePrefix,
                                                   BlendedPoseTrajectoryGenerator trajectory,
                                                   YoVariableRegistry registry,
                                                   YoGraphicsListRegistry graphicsListRegistry)
   {

      trajectoryViz = new YoGraphicTrajectory3D(namePrefix + "Trajectory",
                                                null,
                                                trajectory.getPositionTrajectoryGenerator(),
                                                () -> 1.0,
                                                0.01,
                                                25,
                                                8,
                                                registry);
      graphicsListRegistry.registerYoGraphic(namePrefix + "Trajectory", trajectoryViz);

      trajectoryViz.setColorType(TrajectoryColorType.ACCELERATION_BASED);
   }

   public void visualize()
   {
      if (trajectoryViz == null)
         return;

      trajectoryViz.showGraphic();
      trajectoryViz.update();
   }

   public void showVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.showGraphic();
   }

   public void hideVisualization()
   {
      if (trajectoryViz == null)
         return;
      trajectoryViz.hideGraphic();
   }
}
