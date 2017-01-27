package us.ihmc.atlas.StepAdjustmentVisualizers.forwardStepAndPush;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.StepAdjustmentVisualizers.StepAdjustmentDemo;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

import javax.vecmath.Vector3d;

public class SlowNoAdjustment
{
   private static String script = "scripts/stepAdjustment_forwardWalkingSlow.xml";
   private static double simulationTime = 10.0;

   public SlowNoAdjustment()
   {
   }

   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap())
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true;
               }

               @Override
               public double getMinimumSwingTimeForDisturbanceRecovery()
               {
                  return 0.6;
               }
            };
         }
      };

      StepAdjustmentDemo stepAdjustmentDemo = new StepAdjustmentDemo(robotModel, script);

      double swingTime = stepAdjustmentDemo.getSwingTime();

      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);
      double percentWeight = 0.31;
      double magnitude = percentWeight * stepAdjustmentDemo.getTotalMass() * 9.81;
      double duration = 0.1 * swingTime;

      // push timing:
      StateTransitionCondition pushCondition = stepAdjustmentDemo.getSingleSupportStartCondition(RobotSide.RIGHT);
      stepAdjustmentDemo.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = stepAdjustmentDemo.simulateAndBlockAndCatchExceptions(simulationTime);
   }
}