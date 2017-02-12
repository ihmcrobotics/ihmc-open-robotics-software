package us.ihmc.atlas.StepAdjustmentVisualizers;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.io.printing.PrintTools;

import javax.vecmath.Vector3d;

public class AtlasStepAdjustmentDemo
{
   private static StepScriptType stepScriptType = StepScriptType.FORWARD_FAST;
   private static TestType testType = TestType.FEEDBACK_ONLY;
   private static PushDirection pushDirection = PushDirection.BACKWARD_45;

   private static String forwardFastScript = "scripts/stepAdjustment_forwardWalkingFast.xml";
   private static String forwardSlowScript = "scripts/stepAdjustment_forwardWalkingSlow.xml";
   private static String stationaryFastScript = "scripts/stepAdjustment_stationaryWalkingFast.xml";
   private static String stationarySlowScript = "scripts/stepAdjustment_stationaryWalkingSlow.xml";

   private static double simulationTime = 15.0;

   private final StepAdjustmentDemoHelper stepAdjustmentDemo;

   public AtlasStepAdjustmentDemo(StepScriptType stepScriptType, TestType testType, PushDirection pushDirection)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap())
            {
               @Override
               public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
               {
                  switch (testType)
                  {
                  case FEEDBACK_ONLY:
                  case ADJUSTMENT_ONLY:
                     return false;
                  default:
                     return true;
                  }
               }

               @Override
               public boolean useOptimizationBasedICPController()
               {
                  switch (testType)
                  {
                  case FEEDBACK_ONLY:
                  case SPEED_UP_ONLY:
                     return false;
                  default:
                     return true;
                  }
               }

               @Override
               public double getMinimumSwingTimeForDisturbanceRecovery()
               {
                  switch(stepScriptType)
                  {
                  case FORWARD_FAST:
                  case STATIONARY_FAST:
                     return 0.5;
                  default: // FORWARD_SLOW
                     return 0.6;
                  }
               }
            };
         }
      };

      String script;
      Vector3d forceDirection;
      double percentWeight;

      switch(pushDirection)
      {
      case FORWARD:
         forceDirection = new Vector3d(1.0, 0.0, 0.0);

         switch (stepScriptType)
         {
         case FORWARD_SLOW:
            script = forwardSlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.96;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.49;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.56;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.44;
               break;
            }
            break;
         case STATIONARY_FAST:
            script = stationaryFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.92;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 1.63;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.47;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.47;
               break;
            }
            break;
         case STATIONARY_SLOW:
            script = stationarySlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.73;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.65;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.48;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.47;
               break;
            }
            break;
         default: // FORWARD_FAST
            script = forwardFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.6;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.86;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.62;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.39;
               break;
            }
            break;
         }
         break;
      case BACKWARD:
         forceDirection = new Vector3d(-1.0, 0.0, 0.0);

         switch (stepScriptType)
         {
         case FORWARD_SLOW:
            script = forwardSlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.9;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.68;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.65;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.65;
               break;
            }
            break;
         case STATIONARY_FAST:
            script = stationaryFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.27;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 1.06;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.48;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.48;
               break;
            }
            break;
         case STATIONARY_SLOW:
            script = stationarySlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.66;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.55;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.48;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.48;
               break;
            }
            break;
         default: // FORWARD_FAST
            script = forwardFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.85;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 1.54;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.86;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.86;
               break;
            }
            break;
         }
         break;
      case FORWARD_45:
         double angle = Math.PI / 4.0;
         forceDirection = new Vector3d(Math.cos(angle), -Math.sin(angle), 0.0);

         switch (stepScriptType)
         {
         case FORWARD_SLOW:
            script = forwardSlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.98;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.41;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.69;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.34;
               break;
            }
            break;
         case STATIONARY_FAST:
            script = stationaryFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.76;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 1.33;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.55;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.32;
               break;
            }
            break;
         case STATIONARY_SLOW:
            script = stationarySlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.17;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.42;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.68;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.33;
               break;
            }
            break;
         default: // FORWARD_FAST
            script = forwardFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.47;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.92;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.46;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.3;
               break;
            }
            break;
         }
         break;
      case BACKWARD_45:
         angle = Math.PI / 4.0;
         forceDirection = new Vector3d(-Math.cos(angle), -Math.sin(angle), 0.0);

         switch (stepScriptType)
         {
         case FORWARD_SLOW:
            script = forwardSlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.82;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.47;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.47;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.38;
               break;
            }
            break;
         case STATIONARY_FAST:
            script = stationaryFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.81;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 1.65;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.6;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.36;
               break;
            }
            break;
         case STATIONARY_SLOW:
            script = stationarySlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.14;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.54;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.61;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.37;
               break;
            }
            break;
         default: // FORWARD_FAST
            script = forwardFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.46;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 1.12;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.48;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.33;
               break;
            }
            break;
         }
         break;
      default: // OUTWARD
         forceDirection = new Vector3d(0.0, -1.0, 0.0);

         switch (stepScriptType)
         {
         case FORWARD_SLOW:
            script = forwardSlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.76;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.31;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.47;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.25;
               break;
            }
            break;
         case STATIONARY_FAST:
            script = stationaryFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.72;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.85;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.41;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.26;
               break;
            }
            break;
         case STATIONARY_SLOW:
            script = stationarySlowScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 0.97;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.29;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.52;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.25;
               break;
            }
            break;
         default: // FORWARD_FAST
            script = forwardFastScript;

            switch(testType)
            {
            case BIG_ADJUSTMENT:
               percentWeight = 1.42;
               break;
            case ADJUSTMENT_ONLY:
               percentWeight = 0.83;
               break;
            case SPEED_UP_ONLY:
               percentWeight = 0.36;
               break;
            default: // doesn't allow speed up or step adjustment
               percentWeight = 0.22;
               break;
            }
            break;
         }
         break;
      }


      stepAdjustmentDemo = new StepAdjustmentDemoHelper(robotModel, script);

      // push parameters:

      double swingTime = stepAdjustmentDemo.getSwingTime();
      double delay = 0.5 * swingTime;
      double weight = stepAdjustmentDemo.getTotalMass() * 9.81;
      double magnitude = percentWeight * weight;
      double duration = 0.1 * swingTime;
      PrintTools.info("Weight = " + weight + ", Push Magnitude = " + magnitude + ", Impulse Magnitude = " + duration * magnitude);

      // push timing:
      StateTransitionCondition pushCondition = stepAdjustmentDemo.getSingleSupportStartCondition(RobotSide.RIGHT);
      stepAdjustmentDemo.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   }

   public void simulateAndBlock(double simulationTime) throws
         BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException
   {
      stepAdjustmentDemo.simulateAndBlock(simulationTime);
   }

   public void destroySimulation()
   {
      stepAdjustmentDemo.destroySimulation();
   }

   public static void main(String[] args)
   {
      AtlasStepAdjustmentDemo demo = new AtlasStepAdjustmentDemo(stepScriptType, testType, pushDirection);

      try
      {
         demo.simulateAndBlock(simulationTime);
      }
      catch (Exception e)
      {
      }
   }
}