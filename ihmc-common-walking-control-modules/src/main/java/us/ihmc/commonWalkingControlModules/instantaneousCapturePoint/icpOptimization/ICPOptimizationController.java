package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPOptimizationController
{
   public void setFootstepWeights(double forwardWeight, double lateralWeight);
   public void setFeedbackWeights(double forwardWeight, double lateralWeight);
   public void clearPlan();
   public void setTransferDuration(int stepNumber, double duration);
   public void setTransferSplitFraction(int stepNumber, double splitFraction);
   public void setSwingDuration(int stepNumber, double duration);
   public void setSwingSplitFraction(int stepNumber, double splitFraction);
   public void setFinalTransferDuration(double finalTransferDuration);
   public void setFinalTransferSplitFraction(double finalTransferSplitFraction);
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing);
   public void initializeForStanding(double initialTime);
   public void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0);
   public void initializeForSingleSupport(double initialTime, RobotSide transferToSide, double omega0);

   public int getNumberOfFootstepsToConsider();
   public void getDesiredCMP(FramePoint2D desiredCMP);
   public void getFootstepSolution(int footstepIndex, FramePoint2D footstepSolutionToPack);
   public boolean wasFootstepAdjusted();
   public boolean useAngularMomentum();

   public void compute(double currentTime, FramePoint2D desiredICP, FrameVector2D desiredICPVelocity, FramePoint2D perfectCMP, FramePoint2D currentICP, double omega0);

   public void setFinalTransferSplitFractionToDefault();
   public void setReferenceICPVelocity(FrameVector2D referenceICPVelocity);
   public double getOptimizedTimeRemaining();
   public void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);
}
