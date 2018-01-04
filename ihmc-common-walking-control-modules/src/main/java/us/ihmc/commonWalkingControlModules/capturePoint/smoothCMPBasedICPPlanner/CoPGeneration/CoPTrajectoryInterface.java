package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3DInterface;

public interface CoPTrajectoryInterface extends SegmentedFrameTrajectory3DInterface
{
   public void update(double timeInState, FramePoint3D desiredCoPToPack);
   public void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack);
   public void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack, FrameVector3D desiredCoPAccelerationToPack);
   public void setNextSegment(double initialTime, double finalTime, FramePoint3D initialPosition, FramePoint3D finalPosition);
}
