package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator, PositionProvider
{
   public abstract void getVelocity(FrameVector3D velocityToPack);

   public abstract void getAcceleration(FrameVector3D accelerationToPack);

   public default void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public abstract void showVisualization();

   public abstract void hideVisualization();
}
