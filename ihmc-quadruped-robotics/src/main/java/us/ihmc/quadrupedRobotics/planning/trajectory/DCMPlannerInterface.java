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

   void computeDcmSetpoints(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                            FixedFrameVector3DBasics desiredDCMVelocityToPack);

   void getDesiredECMPPosition(FramePoint3DBasics eCMPPositionToPack);

   void getFinalDCMPosition(FixedFramePoint3DBasics finalDesiredDCMToPack);
}
