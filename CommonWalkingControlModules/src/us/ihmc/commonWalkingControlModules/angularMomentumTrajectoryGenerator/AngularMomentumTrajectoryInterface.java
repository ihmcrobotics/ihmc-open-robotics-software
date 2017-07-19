package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.List;

import us.ihmc.robotics.geometry.FrameVector;

public interface AngularMomentumTrajectoryInterface
{
   void reset();
   void update(double timeInState);
   void update(double timeInState, FrameVector desiredAngularMomentumToPack);
   void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack);
   void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack, FrameVector desiredRotatumToPack);
   List<YoFrameTrajectory3D> getPolynomials();
   int getNumberOfSegments();
}
