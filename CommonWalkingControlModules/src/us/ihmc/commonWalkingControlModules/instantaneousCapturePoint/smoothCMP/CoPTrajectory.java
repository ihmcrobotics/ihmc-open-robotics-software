package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;

import java.util.List;

public interface CoPTrajectory
{
   public void reset();
   public void update(double timeInState);
   public void update(double timeInState, FramePoint desiredCoPToPack);
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack);
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack, FrameVector desiredCoPAccelerationToPack);

   public List<YoFramePolynomial3D> getPolynomials();
   public int getNumberOfSegments();
}
