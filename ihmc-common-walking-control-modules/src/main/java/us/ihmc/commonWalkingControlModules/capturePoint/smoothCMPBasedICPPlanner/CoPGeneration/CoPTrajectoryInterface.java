package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3DInterface;

public interface CoPTrajectoryInterface extends SegmentedFrameTrajectory3DInterface
{
   void update(double timeInState, FramePoint3D desiredCoPToPack);
   void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack);
   void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack, FrameVector3D desiredCoPAccelerationToPack);
   void setNextSegment(double initialTime, double finalTime, FramePoint3DReadOnly initialPosition, FramePoint3DReadOnly finalPosition);
}
