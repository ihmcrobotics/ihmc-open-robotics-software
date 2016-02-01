package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.robotics.robotSide.RobotSide;

public interface StanceSubController
{
   public abstract void doLoadingPreSwingA(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doLoadingPreSwingB(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doLoadingPreSwingC(LegTorques legTorquesToPackForStanceSide, RobotSide loadingLeg, double timeInState);

   public abstract void doEarlyStance(LegTorques legTorquesToPackForStanceSide, double timeInState);

   public abstract void doLateStance(LegTorques legTorquesToPackForStanceSide, double timeInState);

   public abstract void doTerminalStance(LegTorques legTorquesToPackForStanceSide, double timeInState);

   public abstract void doStartWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doStopWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doUnloadLegToTransferIntoWalking(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doLoadingForSingleLegBalance(LowerBodyTorques lowerBodyTorques, RobotSide upcomingSupportSide, double timeInCurrentState);
   
   public abstract void doSingleLegBalance(LegTorques legTorquesToPack, RobotSide supportLeg, double timeInCurrentState);
   

   public abstract boolean isDoneWithLoadingPreSwingA(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneWithLoadingPreSwingB(RobotSide loadingLeg, double timeInState);

   public abstract boolean isReadyToStartStopWalkingDoubleSupport(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneUnloadLegToTransferIntoWalking(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneLoadingForSingleLegBalance(RobotSide upcomingSupportSide, double timeInCurrentState);
   

   public abstract void doTransitionIntoLoadingPreSwingA(RobotSide loadingLeg);

   public abstract void doTransitionIntoLoadingPreSwingB(RobotSide loadingLeg);

   public abstract void doTransitionIntoLoadingPreSwingC(RobotSide loadingLeg);

   public abstract void doTransitionIntoEarlyStance(RobotSide stanceSide);

   public abstract void doTransitionIntoLateStance(RobotSide stanceSide);

   public abstract void doTransitionIntoTerminalStance(RobotSide stanceSide);

   public abstract void doTransitionIntoStartStopWalkingDoubleSupport(RobotSide stanceSide);

   public abstract void doTransitionIntoUnloadLegToTransferIntoWalking(RobotSide stanceSide);
   
   public abstract void doTransitionIntoLoadingForSingleLegBalance(RobotSide upcomingSupportSide);
   
   public abstract void doTransitionIntoSingleLegBalance(RobotSide supportLeg, double[] desiredYawPitchRoll);
   
   public abstract void doTransitionOutOfLoadingPreSwingA(RobotSide loadingLeg);

   public abstract void doTransitionOutOfLoadingPreSwingB(RobotSide loadingLeg);

   public abstract void doTransitionOutOfLoadingPreSwingC(RobotSide loadingLeg);

   public abstract void doTransitionOutOfEarlyStance(RobotSide stanceSide);

   public abstract void doTransitionOutOfLateStance(RobotSide stanceSide);

   public abstract void doTransitionOutOfTerminalStance(RobotSide stanceSide);

   public abstract void doTransitionOutOfStartStopWalkingDoubleSupport(RobotSide stanceSide);

   public abstract void doTransitionOutOfUnloadLegToTransferIntoWalking(RobotSide stanceSide);

   public abstract void doTransitionOutOfLoadingForSingleLegBalance(RobotSide upcomingSupportSide);
   
   public abstract void doTransitionOutOfSingleLegBalance(RobotSide supportLeg);

   
   public abstract boolean canWeStopNow();

   public abstract void initialize();

   public abstract boolean needToTakeAStep(RobotSide supportLeg);
}
