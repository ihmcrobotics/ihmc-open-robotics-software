package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.SplitFractionCalculatorParametersReadOnly;

public abstract interface ICPPlannerParameters extends ICPTrajectoryPlannerParameters, CoPPlannerParameters
{

   double getSwingDurationShiftFraction();

   boolean adjustCoPPlanForSingleSupportContinuity();

   boolean adjustEveryCoPPlanForDoubleSupportContinuity();

   boolean adjustInitialCoPPlanForDoubleSupportContinuity();

   boolean adjustCoPPlanForStandingContinuity();

   boolean doContinuousReplanningForStanding();

   boolean doContinuousReplanningForTransfer();

   boolean doContinuousReplanningForSwing();

   CoPSplineType getOrderOfCoPInterpolation();

   CoPPointName[] getTransferCoPPointsToPlan();

   CoPPointName[] getSwingCoPPointsToPlan();

   boolean planSwingAngularMomentum();

   boolean planTransferAngularMomentum();

   AngularMomentumEstimationParameters getAngularMomentumEstimationParameters();

   default SplitFractionCalculatorParametersReadOnly getSplitFractionCalculatorParameters()
   {
      return new DefaultSplitFractionCalculatorParameters();
   }
}
