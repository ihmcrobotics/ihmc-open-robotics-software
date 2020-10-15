package us.ihmc.robotics.math.trajectories;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.YoGraphicTrajectory3D.TrajectoryColorType;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BlendedPositionTrajectoryGeneratorVisualizer
{
   private final YoGraphicTrajectory3D trajectoryViz;

   public BlendedPositionTrajectoryGeneratorVisualizer(String namePrefix,
                                                       BlendedPoseTrajectoryGenerator trajectory,
                                                       DoubleProvider swingDuration,
                                                       YoRegistry registry,
                                                       YoGraphicsListRegistry graphicsListRegistry)
   {
      this(namePrefix, trajectory.getPositionTrajectoryGenerator(), swingDuration, registry, graphicsListRegistry);
   }

   public BlendedPositionTrajectoryGeneratorVisualizer(String namePrefix,
                                                       BlendedPositionTrajectoryGenerator trajectory,
                                                       DoubleProvider swingDuration,
                                                       YoRegistry registry,
                                                       YoGraphicsListRegistry graphicsListRegistry)
   {
      trajectoryViz = new YoGraphicTrajectory3D(namePrefix + "BlendedTrajectory",
                                                null,
                                                trajectory,
                                                swingDuration,
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
