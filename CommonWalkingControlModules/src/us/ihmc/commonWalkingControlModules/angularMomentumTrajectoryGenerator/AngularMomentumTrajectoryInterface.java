package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3DInterface;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;

public interface AngularMomentumTrajectoryInterface extends SegmentedFrameTrajectory3DInterface
{
   void update(double timeInState, FrameVector desiredAngularMomentumToPack);
   void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack);
   void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack, FrameVector desiredRotatumToPack);
   void set(YoFrameTrajectory3D computedAngularMomentumTrajectory);
   void set(double t0, double tFinal, FramePoint z0, FramePoint zf);
}
