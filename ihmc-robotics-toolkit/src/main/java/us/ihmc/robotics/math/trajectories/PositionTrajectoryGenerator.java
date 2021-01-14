package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public interface PositionTrajectoryGenerator extends TrajectoryGenerator, PositionProvider
{
   default void getVelocity(FrameVector3DBasics velocityToPack)
   {
      velocityToPack.setReferenceFrame(this.getReferenceFrame());
      getVelocity((FixedFrameVector3DBasics) velocityToPack);
   }

   default void getVelocity(FixedFrameVector3DBasics velocityToPack)
   {
      velocityToPack.setMatchingFrame(getVelocity());
   }

   FrameVector3DReadOnly getVelocity();

   default void getAcceleration(FrameVector3DBasics accelerationToPack)
   {
      accelerationToPack.setReferenceFrame(this.getReferenceFrame());
      getAcceleration((FixedFrameVector3DBasics) accelerationToPack);
   }

   default void getAcceleration(FixedFrameVector3DBasics accelerationToPack)
   {
      accelerationToPack.setMatchingFrame(getAcceleration());
   }

   FrameVector3DReadOnly getAcceleration();

   default void getLinearData(FramePoint3DBasics positionToPack, FrameVector3DBasics velocityToPack, FrameVector3DBasics accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   default void getLinearData(FixedFramePoint3DBasics positionToPack, FixedFrameVector3DBasics velocityToPack, FixedFrameVector3DBasics accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   void showVisualization();

   void hideVisualization();
}
