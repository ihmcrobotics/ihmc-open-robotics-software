package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3DInterface;

public interface AngularMomentumTrajectoryInterface extends SegmentedFrameTrajectory3DInterface
{
   void update(double timeInState, FrameVector3D desiredAngularMomentumToPack);
   void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack);
   void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack);
   void set(FrameTrajectory3D computedAngularMomentumTrajectory);
   void set(double t0, double tFinal, FramePoint3D z0, FramePoint3D zf);
}
