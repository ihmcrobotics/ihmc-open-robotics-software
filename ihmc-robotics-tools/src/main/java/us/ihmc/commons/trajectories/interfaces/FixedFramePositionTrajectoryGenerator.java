package us.ihmc.commons.trajectories.interfaces;

import us.ihmc.commons.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

public interface FixedFramePositionTrajectoryGenerator extends PositionTrajectoryGenerator, ReferenceFrameHolder
{
   @Override
   default ReferenceFrame getReferenceFrame()
   {
      return getPosition().getReferenceFrame();
   }

   @Override
   FramePoint3DReadOnly getPosition();

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
}
