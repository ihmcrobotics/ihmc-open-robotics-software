package us.ihmc.robotics.math.trajectories;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FixedFrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.yoVariables.YoGraphicTrajectory3D;
import us.ihmc.robotics.math.trajectories.yoVariables.YoGraphicTrajectory3D.TrajectoryColorType;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BlendedPositionTrajectoryGeneratorVisualizer
{
   private static final boolean useBagOfBalls = true;

   private static final int numberOfBalls = 25;
   private final DoubleProvider swingDuration;
   private final BlendedWaypointPositionTrajectoryGenerator trajectory;
   private final BagOfBalls trajectoryViz;
   private final BagOfBalls waypointViz;
   private final YoGraphicTrajectory3D splineViz;

   public BlendedPositionTrajectoryGeneratorVisualizer(String namePrefix,
                                                       BlendedPoseTrajectoryGenerator trajectory,
                                                       DoubleProvider swingDuration,
                                                       YoRegistry registry,
                                                       YoGraphicsListRegistry graphicsListRegistry)
   {
      this(namePrefix, trajectory.getPositionTrajectoryGenerator(), swingDuration, registry, graphicsListRegistry);
   }

   public BlendedPositionTrajectoryGeneratorVisualizer(String namePrefix,
                                                       BlendedWaypointPositionTrajectoryGenerator trajectory,
                                                       DoubleProvider swingDuration,
                                                       YoRegistry registry,
                                                       YoGraphicsListRegistry graphicsListRegistry)
   {
      this.trajectory = trajectory;
      this.swingDuration = swingDuration;
      if (useBagOfBalls)
      {
         trajectoryViz = new BagOfBalls(numberOfBalls, 0.01, namePrefix + "BlendTrajectory", registry, graphicsListRegistry);
         waypointViz = new BagOfBalls(numberOfBalls, 0.025, namePrefix + "BlendWaypoints", YoAppearance.Green(), registry, graphicsListRegistry);
         splineViz = null;
      }
      else
      {
         trajectoryViz = null;
         waypointViz = null;
         splineViz = new YoGraphicTrajectory3D(namePrefix + "BlendedTrajectory", null, trajectory, swingDuration, 0.01, 25, 8, registry);
         graphicsListRegistry.registerYoGraphic(namePrefix + "Trajectory", splineViz);

         splineViz.setColorType(TrajectoryColorType.ACCELERATION_BASED);
      }
   }

   public void visualize()
   {
      if (useBagOfBalls)
      {
         trajectoryViz.reset();
         waypointViz.reset();

         MultipleWaypointsPositionTrajectoryGenerator blendTrajecotry = trajectory.getBlendTrajectory();
         for (int i = 0; i < blendTrajecotry.getCurrentNumberOfWaypoints(); i++)
         {
            FixedFrameEuclideanTrajectoryPointBasics blendWaypont = blendTrajecotry.getWaypoint(i);
            trajectory.compute(blendWaypont.getTime());
            waypointViz.setBall(trajectory.getPosition());
         }

         double dt = swingDuration.getValue() / numberOfBalls;
         double time = 0.0;
         for (int i = 0; i < numberOfBalls; i++)
         {
            trajectory.compute(time);
            trajectoryViz.setBall(trajectory.getPosition());

            time += dt;
         }
      }
      else
      {
         splineViz.showGraphic();
         splineViz.update();
      }
   }

   public void showVisualization()
   {
      if (!useBagOfBalls)
         splineViz.showGraphic();
   }

   public void hideVisualization()
   {
      if (useBagOfBalls)
      {
         trajectoryViz.reset();
         trajectoryViz.hideAll();

         waypointViz.reset();
         waypointViz.hideAll();
      }
      else
      {
         splineViz.hideGraphic();
      }
   }
}
