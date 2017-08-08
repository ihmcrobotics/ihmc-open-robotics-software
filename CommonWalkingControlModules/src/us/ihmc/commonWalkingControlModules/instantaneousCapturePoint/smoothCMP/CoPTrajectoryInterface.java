package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3DInterface;

public interface CoPTrajectoryInterface extends SegmentedFrameTrajectory3DInterface
{
   public void update(double timeInState, FramePoint desiredCoPToPack);
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack);
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack, FrameVector desiredCoPAccelerationToPack);
   public void setSegment(double initialTime, double finalTime, FramePoint initialPosition, FramePoint finalPosition);
}
