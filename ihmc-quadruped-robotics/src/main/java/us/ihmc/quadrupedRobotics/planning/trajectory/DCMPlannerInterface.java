package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

public interface DCMPlannerInterface
{
//   void initializeForStanding();

//   void initializeForStepping(double currentTime, QuadrantDependentList<YoEnum<ContactState>> currentContactStates, List<? extends QuadrupedTimedStep> stepSequence,
//                              FramePoint3DReadOnly currentDCMPosition, FrameVector3DReadOnly currentDCMVelocity);

   void setInitialState(double initialTime, FramePoint3DReadOnly currentDCMPosition, FrameVector3DReadOnly currentDCMVelocity);

//   void initialize();

   void setHoldCurrentDesiredPosition(boolean holdPosition);

   void computeSetpoints(double currentTime, QuadrantDependentList<YoEnum<ContactState>> currentContactStates, List<? extends QuadrupedTimedStep> stepSequence);

   void getFinalDCMPosition(FixedFramePoint3DBasics finalDesiredDCMToPack);

   FramePoint3DReadOnly getDesiredDCMPosition();

   FrameVector3DReadOnly getDesiredDCMVelocity();

   FramePoint3DReadOnly getDesiredCoMPosition();

   FrameVector3DReadOnly getDesiredCoMVelocity();

   FrameVector3DReadOnly getDesiredCoMAcceleration();

   FramePoint3DReadOnly getDesiredVRPPosition();

   FramePoint3DReadOnly getDesiredECMPPosition();
}
