package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface CoPTrajectoryInterface
{
   public void reset();
   public void update(double timeInState);
   public void update(double timeInState, FramePoint3D desiredCoPToPack);
   public void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack);
   public void update(double timeInState, FramePoint3D desiredCoPToPack, FrameVector3D desiredCoPVelocityToPack, FrameVector3D desiredCoPAccelerationToPack);
   
   public void setSegment(CoPSplineType segmentInterpolationOrder, double initialTime, double finalTime, FramePoint3D initialPosition, FramePoint3D finalPosition);
   public List<YoFrameTrajectory3D> getPolynomials();
   public int getNumberOfSegments();
}
