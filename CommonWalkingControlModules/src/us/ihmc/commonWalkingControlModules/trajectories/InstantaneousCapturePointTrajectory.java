package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.util.trajectory.Finishable;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;

public interface InstantaneousCapturePointTrajectory extends Finishable
{

   public abstract void initialize(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, double moveTime, double omega0, double amountToBeInside);

   public abstract void pack(FramePoint2d desiredPosition, FrameVector2d desiredVelocity, double omega0);

   public abstract double getMoveTime();

   public abstract void reset();

}