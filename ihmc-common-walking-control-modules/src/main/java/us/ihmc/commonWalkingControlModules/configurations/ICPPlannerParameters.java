package us.ihmc.commonWalkingControlModules.configurations;

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
}
