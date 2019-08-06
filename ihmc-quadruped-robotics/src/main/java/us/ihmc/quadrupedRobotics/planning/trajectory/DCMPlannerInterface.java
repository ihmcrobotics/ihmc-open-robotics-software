package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.variable.YoEnum;

public interface DCMPlannerInterface
{
   void clearStepSequence();

   void addStepToSequence(QuadrupedTimedStep step);

   void initializeForStanding();

   void initializeForStepping(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FramePoint3DReadOnly currentDCMPosition,
                              FrameVector3DReadOnly currentDCMVelocity);

   void beganStep();

   void completedStep();

   void setHoldCurrentDesiredPosition(boolean holdPosition);

   void computeSetpoints(QuadrantDependentList<YoEnum<ContactState>> currentContactStates);

   void getFinalDCMPosition(FixedFramePoint3DBasics finalDesiredDCMToPack);

   FramePoint3DReadOnly getDesiredDCMPosition();

   FrameVector3DReadOnly getDesiredDCMVelocity();

   FramePoint3DReadOnly getDesiredCoMPosition();

   FrameVector3DReadOnly getDesiredCoMVelocity();

   FrameVector3DReadOnly getDesiredCoMAcceleration();

   FramePoint3DReadOnly getDesiredVRPPosition();

   FramePoint3DReadOnly getDesiredECMPPosition();
}
