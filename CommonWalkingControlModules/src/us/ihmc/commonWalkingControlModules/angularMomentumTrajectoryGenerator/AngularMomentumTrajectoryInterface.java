package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.robotics.geometry.FrameVector;

public interface AngularMomentumTrajectoryInterface
{
   public void reset();
   public void update(double timeInState);
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack);
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack);
   public List<YoFrameTrajectory3D> getPolynomials();
   public int getNumberOfSegments();
}
