package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;

public interface FramePositionTrajectoryGenerator extends FramePositionProvider, PositionTrajectoryGenerator
{
   @Override
   FrameVector3DReadOnly getVelocity();

   @Override
   FrameVector3DReadOnly getAcceleration();

   default void getLinearData(FramePoint3DBasics positionToPack, FrameVector3DBasics velocityToPack, FrameVector3DBasics accelerationToPack)
   {
      positionToPack.setReferenceFrame(getReferenceFrame());
      velocityToPack.setReferenceFrame(getReferenceFrame());
      accelerationToPack.setReferenceFrame(getReferenceFrame());
      PositionTrajectoryGenerator.super.getLinearData(positionToPack, velocityToPack, accelerationToPack);
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
