package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

public interface DCMPlannerInterface
{
   void clearStepSequence();

   void setCoMHeight(double comHeight);

   void addStepToSequence(QuadrupedTimedStep step);

   void initializeForStanding();

   void initializeForStepping(QuadrantDependentList<ContactState> currentContactStates, FramePoint3DReadOnly dcmPosition);

   void computeDcmSetpoints(QuadrantDependentList<ContactState> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                   FixedFrameVector3DBasics desiredDCMVelocityToPack);

   void getFinalDesiredDCM(FixedFramePoint3DBasics finalDesiredDCMToPack);

   double getFinalTime();
}
