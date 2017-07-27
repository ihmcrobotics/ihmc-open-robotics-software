package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface AngularMomentumTrajectoryInterface
{
   public void reset();
   public void update(double timeInState);
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack);
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack);
   public List<YoFrameTrajectory3D> getPolynomials();
   public int getNumberOfSegments();
}
