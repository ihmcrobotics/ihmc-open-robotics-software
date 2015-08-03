package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.math.trajectories.Finishable;


public interface InstantaneousCapturePointTrajectory extends Finishable
{

   public abstract void initialize(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, double moveTime, double omega0, double amountToBeInside, RobotSide supportSide, double time);

   public abstract void getCurrentDesiredICPPositionAndVelocity(FramePoint2d desiredPosition, FrameVector2d desiredVelocity, double omega0, double currentTime);

   public abstract double getStartTime();
      
   public abstract void reset();

   public abstract double getEstimatedTimeRemainingForState(double time);

}