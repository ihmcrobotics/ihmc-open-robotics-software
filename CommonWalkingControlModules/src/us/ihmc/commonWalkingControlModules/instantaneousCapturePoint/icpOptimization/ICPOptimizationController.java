package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPOptimizationController
{
   public void setFootstepWeights(double forwardWeight, double lateralWeight);

   public void setFeedbackWeights(double forwardWeight, double lateralWeight);

   public void setDynamicRelaxationWeight(double relaxationWeight);

   public void clearPlan();

   public void setFinalTransferDuration(double finalTransferDuration);

   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   public void initializeForStanding(double initialTime);

   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0);

   public void initializeForSingleSupport(double initialTime, RobotSide supportSide, double omega0);

   public void setBeginningOfStateICP(FramePoint2d beginningOfStateICP, FrameVector2d beginningOfStateICPVelocity);

   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP, double omega0);

   public int getNumberOfFootstepsToConsider();

   public void getDesiredCMP(FramePoint2d desiredCMPToPack);

   public void getFootstepSolution(int footstepIndex, FramePoint2d footstepSolutionToPack);

   public boolean wasFootstepAdjusted();

   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);

   public boolean useAngularMomentum();
}
