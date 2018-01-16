package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ICPOptimizationControllerInterface
{
   void clearPlan();

   void setTransferDuration(double duration);
   void setSwingDuration(double duration);
   void setNextTransferDuration(double duration);

   void setFinalTransferDuration(double finalTransferDuration);

   void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon);

   void addFootstepToPlan(Footstep footstep, FootstepTiming timing);

   void initializeForStanding(double initialTime);
   void initializeForTransfer(double initialTime, RobotSide transferToSide, double omega0);
   void initializeForSingleSupport(double initialTime, RobotSide transferToSide, double omega0);

   void getDesiredCMP(FramePoint2D desiredCMP);
   void getFootstepSolution(FramePoint2D footstepSolutionToPack);
   boolean wasFootstepAdjusted();
   boolean useAngularMomentum();
   boolean useStepAdjustment();

   void compute(double currentTime, FramePoint2D desiredICP, FrameVector2D desiredICPVelocity, FramePoint2D perfectCMP, FramePoint2D currentICP, double omega0);

   void submitRemainingTimeInSwingUnderDisturbance(double remainingTimeForSwing);
}
