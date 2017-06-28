package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;

public interface AngularMomentumTrajectory
{
   public void reset();
   public void update(double timeInState);
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack);
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack);
   public List<YoFramePolynomial3D> getPolynomials();
   public int getNumberOfSegments();
}
