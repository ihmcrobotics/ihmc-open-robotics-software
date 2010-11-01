package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;

public interface StanceSubController
{
   public abstract void doLoadingPreSwingA(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doLoadingPreSwingB(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doLoadingPreSwingC(LegTorques legTorquesToPackForStanceSide, RobotSide loadingLeg, double timeInState);

   public abstract void doEarlyStance(LegTorques legTorquesToPackForStanceSide, double timeInState);

   public abstract void doLateStance(LegTorques legTorquesToPackForStanceSide, double timeInState);

   public abstract void doTerminalStance(LegTorques legTorquesToPackForLoadingLeg, double timeInState);

   public abstract void doStartStopWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract void doUnloadLegToTransferIntoWalking(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneWithLoadingPreSwingA(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneWithLoadingPreSwingB(RobotSide loadingLeg, double timeInState);

   // TODO: I think we can delete this method (pdn)
   public abstract boolean isDoneStartStopWalkingDoubleSupport(RobotSide loadingLeg, double timeInState);

   public abstract boolean isDoneUnloadLegToTransferIntoWalking(RobotSide loadingLeg, double timeInState);

   public abstract void doTransitionIntoLoadingPreSwingA(RobotSide loadingLeg);

   public abstract void doTransitionIntoLoadingPreSwingB(RobotSide loadingLeg);

   public abstract void doTransitionIntoLoadingPreSwingC(RobotSide loadingLeg);

   public abstract void doTransitionIntoEarlyStance(RobotSide stanceSide);

   public abstract void doTransitionIntoLateStance(RobotSide stanceSide);

   public abstract void doTransitionIntoTerminalStance(RobotSide stanceSide);

   public abstract void doTransitionIntoStartStopWalkingDoubleSupport(RobotSide stanceSide);

   public abstract void doTransitionIntoUnloadLegToTransferIntoWalking(RobotSide stanceSide);

   public abstract void doTransitionOutOfLoadingPreSwingA(RobotSide loadingLeg);

   public abstract void doTransitionOutOfLoadingPreSwingB(RobotSide loadingLeg);

   public abstract void doTransitionOutOfLoadingPreSwingC(RobotSide loadingLeg);

   public abstract void doTransitionOutOfEarlyStance(RobotSide stanceSide);

   public abstract void doTransitionOutOfLateStance(RobotSide stanceSide);

   public abstract void doTransitionOutOfTerminalStance(RobotSide stanceSide);

   public abstract void doTransitionOutOfStartStopWalkingDoubleSupport(RobotSide stanceSide);

   public abstract void doTransitionOutOfUnloadLegToTransferIntoWalking(RobotSide stanceSide);

   public abstract RobotSide getUpcomingSupportLegForStartingToWalkFromDoubleSupport();

   public abstract boolean canWeStopNowStanceSubController();

   public abstract void doDoubleSupportStanceControl(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg);

}
