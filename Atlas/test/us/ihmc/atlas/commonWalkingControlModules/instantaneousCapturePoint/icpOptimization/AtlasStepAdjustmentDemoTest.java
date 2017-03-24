package us.ihmc.atlas.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.atlas.StepAdjustmentVisualizers.AtlasStepAdjustmentDemo;
import us.ihmc.atlas.StepAdjustmentVisualizers.PushDirection;
import us.ihmc.atlas.StepAdjustmentVisualizers.StepScriptType;
import us.ihmc.atlas.StepAdjustmentVisualizers.TestType;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT, IntegrationCategory.VIDEO})
public class AtlasStepAdjustmentDemoTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double simulationTime = 15.0;
   private AtlasStepAdjustmentDemo demo;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepForwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.BIG_ADJUSTMENT, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);

      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepForwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.SPEED_UP_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepForwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.FEEDBACK_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepForwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.ADJUSTMENT_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepBackwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.BIG_ADJUSTMENT, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepBackwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.SPEED_UP_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepBackwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.FEEDBACK_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepBackwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.ADJUSTMENT_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepOutwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.BIG_ADJUSTMENT, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepOutwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.SPEED_UP_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepOutwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.FEEDBACK_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardFastStepOutwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_FAST, TestType.ADJUSTMENT_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepForwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.BIG_ADJUSTMENT, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepForwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.SPEED_UP_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepForwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.FEEDBACK_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepForwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.ADJUSTMENT_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepBackwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.BIG_ADJUSTMENT, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepBackwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.SPEED_UP_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepBackwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.FEEDBACK_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
      demo.destroySimulation();
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepBackwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.ADJUSTMENT_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepOutwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.BIG_ADJUSTMENT, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepOutwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.SPEED_UP_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepOutwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.FEEDBACK_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testForwardSlowStepOutwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.FORWARD_SLOW, TestType.ADJUSTMENT_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepForwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.BIG_ADJUSTMENT, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepForwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.SPEED_UP_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepForwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.FEEDBACK_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepForwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.ADJUSTMENT_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepBackwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.BIG_ADJUSTMENT, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepBackwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.SPEED_UP_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepBackwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.FEEDBACK_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepBackwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.ADJUSTMENT_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepOutwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.BIG_ADJUSTMENT, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepOutwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.SPEED_UP_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepOutwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.FEEDBACK_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationaryFastStepOutwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_FAST, TestType.ADJUSTMENT_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepForwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.BIG_ADJUSTMENT, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepForwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.SPEED_UP_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepForwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.FEEDBACK_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepForwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.ADJUSTMENT_ONLY, PushDirection.FORWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepBackwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.BIG_ADJUSTMENT, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepBackwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.SPEED_UP_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepBackwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.FEEDBACK_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepBackwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.ADJUSTMENT_ONLY, PushDirection.BACKWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepOutwardPushBigAdjustment() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.BIG_ADJUSTMENT, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepOutwardPushSpeedUpOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.SPEED_UP_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepOutwardPushFeedbackOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.FEEDBACK_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testStationarySlowStepOutwardPushAdjustmentOnly() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      demo = new AtlasStepAdjustmentDemo(StepScriptType.STATIONARY_SLOW, TestType.ADJUSTMENT_ONLY, PushDirection.OUTWARD);
      demo.simulateAndBlock(simulationTime);
   }
}
