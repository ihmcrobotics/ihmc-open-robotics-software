package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;

public interface CoPTrajectoryInterface
{
   public void reset();
   public void update(double timeInState);
   public void update(double timeInState, FramePoint desiredCoPToPack);
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack);
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack, FrameVector desiredCoPAccelerationToPack);
   
   public void setSegment(CoPSplineType segmentInterpolationOrder, double initialTime, double finalTime, FramePoint initialPosition, FramePoint finalPosition);
   public List<YoFrameTrajectory3D> getPolynomials();
   public int getNumberOfSegments();
}
