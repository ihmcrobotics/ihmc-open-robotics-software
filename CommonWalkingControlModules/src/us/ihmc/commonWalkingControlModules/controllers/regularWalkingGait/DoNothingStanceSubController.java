package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.robotics.robotSide.RobotSide;

public class DoNothingStanceSubController implements StanceSubController
{
   public void doLoadingPreSwingA(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();

   }


   public void doLoadingPreSwingB(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();

   }


   public void doLoadingPreSwingC(LegTorques legTorquesToPackForStanceSide, RobotSide loadingLeg, double timeInState)
   {
      legTorquesToPackForStanceSide.setTorquesToZero();

   }


   public void doEarlyStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      legTorquesToPackForStanceSide.setTorquesToZero();

   }


   public void doLateStance(LegTorques legTorquesToPackForStanceSide, double timeInState)
   {
      legTorquesToPackForStanceSide.setTorquesToZero();

   }


   public void doTerminalStance(LegTorques legTorquesToPackForLoadingLeg, double timeInState)
   {
      legTorquesToPackForLoadingLeg.setTorquesToZero();
   }


   public void doStartWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();

   }


   public void doUnloadLegToTransferIntoWalking(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();

   }

   public void doLoadingForSingleLegBalance(LowerBodyTorques lowerBodyTorques, RobotSide upcomingSupportSide, double timeInCurrentState)
   {
      lowerBodyTorques.setLowerBodyTorquesToZero();
   }



   public void doSingleLegBalance(LegTorques legTorquesToPack, RobotSide supportLeg, double timeInCurrentState)
   {
      legTorquesToPack.setTorquesToZero();
   }

   public boolean isDoneWithLoadingPreSwingA(RobotSide loadingLeg, double timeInState)
   {
      return false;
   }


   public boolean isDoneWithLoadingPreSwingB(RobotSide loadingLeg, double timeInState)
   {
      return false;
   }


   public boolean isReadyToStartStopWalkingDoubleSupport(RobotSide loadingLeg, double timeInState)
   {
      return false;
   }


   public boolean isDoneUnloadLegToTransferIntoWalking(RobotSide loadingLeg, double timeInState)
   {
      return false;
   }

   public boolean isDoneLoadingForSingleLegBalance(RobotSide upcomingSupportSide, double timeInCurrentState)
   {
      return false;
   }

   public void doTransitionIntoLoadingPreSwingA(RobotSide loadingLeg)
   {
   }


   public void doTransitionIntoLoadingPreSwingB(RobotSide loadingLeg)
   {
   }


   public void doTransitionIntoLoadingPreSwingC(RobotSide loadingLeg)
   {
   }


   public void doTransitionIntoEarlyStance(RobotSide stanceSide)
   {
   }


   public void doTransitionIntoLateStance(RobotSide stanceSide)
   {
   }


   public void doTransitionIntoTerminalStance(RobotSide stanceSide)
   {
   }


   public void doTransitionIntoStartStopWalkingDoubleSupport(RobotSide stanceSide)
   {
   }


   public void doTransitionIntoUnloadLegToTransferIntoWalking(RobotSide stanceSide)
   {
   }

   public void doTransitionIntoLoadingForSingleLegBalance(RobotSide upcomingSupportSide)
   {
   }

   public void doTransitionIntoSingleLegBalance(RobotSide supportLeg, double[] desiredYawPitchRoll)
   {
   }


   public void doTransitionOutOfLoadingPreSwingA(RobotSide loadingLeg)
   {
   }


   public void doTransitionOutOfLoadingPreSwingB(RobotSide loadingLeg)
   {
   }


   public void doTransitionOutOfLoadingPreSwingC(RobotSide loadingLeg)
   {
   }


   public void doTransitionOutOfEarlyStance(RobotSide stanceSide)
   {
   }


   public void doTransitionOutOfLateStance(RobotSide stanceSide)
   {
   }


   public void doTransitionOutOfTerminalStance(RobotSide stanceSide)
   {
   }


   public void doTransitionOutOfStartStopWalkingDoubleSupport(RobotSide stanceSide)
   {
   }


   public void doTransitionOutOfUnloadLegToTransferIntoWalking(RobotSide stanceSide)
   {
   }

   public void doTransitionOutOfLoadingForSingleLegBalance(RobotSide upcomingSupportSide)
   {
   }


   public void doTransitionOutOfSingleLegBalance(RobotSide supportLeg)
   {
   }

   public boolean canWeStopNow()
   {
      return false;
   }


   public void doStopWalkingDoubleSupport(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, double timeInState)
   {
      lowerBodyTorquesToPack.setLowerBodyTorquesToZero();

   }


   public void initialize()
   {      
   }
   
   public boolean needToTakeAStep(RobotSide supportLeg)
   {
      return false;
   }
}
