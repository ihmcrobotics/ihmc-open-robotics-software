package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

public interface QuadrupedCoMTrajectoryPlannerInterface
{
   void initialize();

   void setNominalCoMHeight(double comHeight);

   void setInitialState(double initialTime, FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity, FramePoint3DReadOnly copPosition)

   void computeSetpoints(double currentTime, List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact);

   FramePoint3DReadOnly getDesiredDCMPosition();

   FrameVector3DReadOnly getDesiredDCMVelocity();

   FramePoint3DReadOnly getDesiredCoMPosition();

   FrameVector3DReadOnly getDesiredCoMVelocity();

   FrameVector3DReadOnly getDesiredCoMAcceleration();

   FramePoint3DReadOnly getDesiredVRPPosition();

   FramePoint3DReadOnly getDesiredECMPPosition();
}
