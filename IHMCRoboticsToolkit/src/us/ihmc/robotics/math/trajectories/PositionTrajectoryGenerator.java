package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator, PositionProvider
{
   public abstract void getVelocity(FrameVector velocityToPack);

   public abstract void getAcceleration(FrameVector accelerationToPack);

   public abstract void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack);

   public abstract void showVisualization();

   public abstract void hideVisualization();
}
