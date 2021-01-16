package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;

public interface FramePositionTrajectoryGenerator extends TrajectoryGenerator, FramePositionProvider
{
   FrameVector3DReadOnly getVelocity();

   FrameVector3DReadOnly getAcceleration();

   default void getLinearData(FramePoint3DBasics positionToPack, FrameVector3DBasics velocityToPack, FrameVector3DBasics accelerationToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
      velocityToPack.setIncludingFrame(getVelocity());
      accelerationToPack.setIncludingFrame(getAcceleration());
   }

   default void getLinearData(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics velocityToPack, FixedFrameVector3DBasics accelerationToPack)
   {
      positionToPack.setMatchingFrame(getPosition());
      velocityToPack.setMatchingFrame(getVelocity());
      accelerationToPack.setMatchingFrame(getAcceleration());
   }

   void showVisualization();

   void hideVisualization();
}
