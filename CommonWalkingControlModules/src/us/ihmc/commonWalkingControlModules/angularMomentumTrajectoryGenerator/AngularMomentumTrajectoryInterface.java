package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3DInterface;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;

public interface AngularMomentumTrajectoryInterface extends SegmentedFrameTrajectory3DInterface
{
   void update(double timeInState, FrameVector3D desiredAngularMomentumToPack);
   void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack);
   void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack);
   void set(YoFrameTrajectory3D computedAngularMomentumTrajectory);
   void set(double t0, double tFinal, FramePoint3D z0, FramePoint3D zf);
}
