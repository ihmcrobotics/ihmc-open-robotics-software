package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOffsetTrajectoryWhileWalking;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public abstract class PushRecoveryControllerParameters
{

   /**
    * This is the minimum transfer time that the controller will allow when adjusting transfer times to achieve certain step
    * times in footstep plans.
    */
   public double getMinimumTransferTime()
   {
      return 0.1;
   }

   /**
    * This is the minimum swing time that the controller will allow when adjusting transfer times to achieve certain step
    * times in footstep plans.
    */
   public double getMinimumSwingTime()
   {
      return 0.6;
   }

   /**
    * Returns the {@link MomentumOptimizationSettings} for this robot. These parameters define the weights
    * given to the objectives of the walking controller in the QP.
    */
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return null;
   }


   /**
    * When true, some of the tracking performance will be degraded to reduce the generated angular momentum rate around
    * the vertical axis during swing only.
    * Useful when the robot has heavy legs and tends to slips during swing.
    * @return
    */
   public boolean minimizeAngularMomentumRateZDuringSwing()
   {
      return false;
   }

   public boolean minimizeAngularMomentumRateZDuringTransfer()
   {
      return false;
   }

   /**
    * Determines whether or not to attempt to directly control the height.
    * If true, the height will be controlled directly via a command to the controller core. This can be
    * a linear momentum z command or a feedback control command for the pelvis.
    * If false, the height will be controlled inside the nullspace of other objectives by trying to achieve
    * the desired privileged configuration in the legs.
    * @return boolean (true = control height, false = do not control height but leave it up to the optimization)
    */
   public boolean enableHeightFeedbackControl()
   {
      return true;
   }

   /**
    * Returns parameters related to stepping such as maximum step length etc.
    */
   public SteppingParameters getSteppingParameters()
   {
      return null;
   }
}
