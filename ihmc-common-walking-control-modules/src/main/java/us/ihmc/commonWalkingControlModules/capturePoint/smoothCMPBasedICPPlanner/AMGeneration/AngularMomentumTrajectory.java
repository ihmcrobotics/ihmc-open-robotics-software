package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class AngularMomentumTrajectory extends SegmentedFrameTrajectory3D implements AngularMomentumTrajectoryInterface
{
   public AngularMomentumTrajectory(int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(maxNumberOfSegments, maxNumberOfCoefficients);
   }

   @Override
   public void reset()
   {
      super.reset();
   }

   @Override
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack)
   {
      update(timeInState);
      desiredAngularMomentumToPack.setIncludingFrame(currentSegment.getFramePosition());
   }

   @Override
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack)
   {
      update(timeInState, desiredAngularMomentumToPack);
      desiredTorqueToPack.setIncludingFrame(currentSegment.getFrameVelocity());
   }

   @Override
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack)
   {
      update(timeInState, desiredAngularMomentumToPack, desiredTorqueToPack);
      desiredRotatumToPack.setIncludingFrame(currentSegment.getFrameAcceleration());
   }

   @Override
   public void set(FrameTrajectory3D computedAngularMomentumTrajectory)
   {
      FrameTrajectory3D segment = segments.add();
      segment.set(computedAngularMomentumTrajectory);
   }

   public void set(double t0, double tFinal, FramePoint3D z0, FramePoint3D zf)
   {
      FrameTrajectory3D segment = segments.add();
      segment.setLinear(t0, tFinal, z0, zf);
   }
}
