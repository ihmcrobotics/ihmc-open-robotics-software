package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOffsetTrajectoryWhileWalking;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public interface  PushRecoveryControllerParameters
{
   default int getMaxStepsToGenerateForRecovery()
   {
      return 2;
   }
   default double getFinalTransferDurationForRecovery()
   {
      return 0.25;
   }

   default double getTransferDurationAfterRecovery()
   {
      return 1.0;
   }

   /**
    * This is the minimum transfer time that the controller will allow when adjusting transfer times to achieve certain step
    * times in footstep plans.
    */
   default double getRecoveryTransferDuration()
   {
      return 0.1;
   }

   /**
    * This is the minimum swing time that the controller will allow when adjusting transfer times to achieve certain step
    * times in footstep plans.
    */
   default double getMinimumRecoverySwingDuration()
   {
      return 0.3;
   }

   default double getMaximumRecoverySwingDuration()
   {
      return 1.0;
   }

   /**
    * Returns the {@link MomentumOptimizationSettings} for this robot. These parameters define the weights
    * given to the objectives of the walking controller in the QP.
    */
   MomentumOptimizationSettings getMomentumOptimizationSettings();


   /**
    * Determines whether or not to attempt to directly control the height.
    * If true, the height will be controlled directly via a command to the controller core. This can be
    * a linear momentum z command or a feedback control command for the pelvis.
    * If false, the height will be controlled inside the nullspace of other objectives by trying to achieve
    * the desired privileged configuration in the legs.
    * @return boolean (true = control height, false = do not control height but leave it up to the optimization)
    */
   default boolean enableHeightFeedbackControl()
   {
      return true;
   }

   double getMaxStepLength();

   default double getMaxBackwardsStepLength()
   {
      return getMaxStepLength();
   }

   double getMinStepWidth();

   default double getMaxStepWidth()
   {
      return getMaxStepLength();
   }

   default double getMaxICPErrorBeforeSingleSupportForwardX()
   {
      return 0.035;
   }

   default double getMaxICPErrorBeforeSingleSupportBackwardX()
   {
      return 0.035;
   }

   default double getMaxICPErrorBeforeSingleSupportInnerY()
   {
      return 0.015;
   }

   default double getMaxICPErrorBeforeSingleSupportOuterY()
   {
      return 0.015;
   }

   default double getPreferredStepWidth()
   {
      return 0.3;
   }

   default double getMaxAllowedFinalStepXOffset()
   {
      return 0.1;
   }
}
