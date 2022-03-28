package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;

public class ValkyrieStepAdjustmentParameters extends StepAdjustmentParameters
{
   private final boolean useStepAdjustment;
   private final RobotTarget target;

   public ValkyrieStepAdjustmentParameters(RobotTarget target)
   {
      this.target = target;
      useStepAdjustment = (target == RobotTarget.REAL_ROBOT) ? false : true;
   }

   /**
    * Specifies the amount of ICP error (the 2D distance in XY from desired to current) that is required for the controller to consider step adjustment.
    */
   public double getMinICPErrorForStepAdjustment()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.04 : super.getMinICPErrorForStepAdjustment();
   }

   /** {@inheritDoc} */
   @Override
   public boolean allowStepAdjustment()
   {
      return useStepAdjustment;
   }

   /** {@inheritDoc} */
   @Override
   public double getAdjustmentDeadband()
   {
      return 0.02;
   }

}
