package us.ihmc.atlas.StepAdjustmentVisualizers;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;

import javax.vecmath.Vector3d;

public class OutwardPush
{
   private static String script = "scripts/stepAdjustment_forwardWalkingFast.xml";
   private static double simulationTime = 10.0;

   public OutwardPush()
   {
   }

   public static void main(String[] args)
   {
      StepAdjustmentDemo stepAdjustmentDemo = new StepAdjustmentDemo(script);

      double swingTime = stepAdjustmentDemo.getSwingTime();

      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double percentWeight = 0.7;
      double magnitude = percentWeight * stepAdjustmentDemo.getTotalMass() * 9.81;
      double duration = 0.1 * swingTime;

      // push timing:
      StateTransitionCondition pushCondition = stepAdjustmentDemo.getSingleSupportStartCondition(RobotSide.RIGHT);
      stepAdjustmentDemo.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = stepAdjustmentDemo.simulateAndBlockAndCatchExceptions(simulationTime);
   }
}