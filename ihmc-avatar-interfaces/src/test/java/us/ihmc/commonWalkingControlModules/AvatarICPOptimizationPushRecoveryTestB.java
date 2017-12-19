package us.ihmc.commonWalkingControlModules;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.util.Random;

import static org.junit.Assert.assertTrue;

public abstract class AvatarICPOptimizationPushRecoveryTestB extends AvatarICPOptimizationPushRecoveryTestSetup
{
   public void testPushICPOptimizationNoPush() throws SimulationExceededMaximumTimeException
   {
      //setupTest(getScript());
      setupAndRunTest(createForwardWalkingFootstepMessage());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   public void testPushICPOptimizationOutwardPushInSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      setupAndRunTest(createForwardWalkingFootstepMessage());

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   public void testPushICPOptimizationOutwardPushInSlowSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      setupAndRunTest(createSlowForwardWalkingFootstepMessage());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:t
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   public void testPushICPOptimizationDiagonalOutwardPushInSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      setupAndRunTest(createForwardWalkingFootstepMessage());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(2.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendYawRotation(0.5);
      ReferenceFrame referenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("yawing", ReferenceFrame.getWorldFrame(), transform);

      //setupTest(getYawscript(), referenceFrame);
      setupAndRunTest(createYawingForwardWalkingFootstepMessage());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push parameters:
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;

      Vector3D firstForceDirection = new Vector3D(0.0, -1.0, 0.0);
      Vector3D secondForceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);
      boolean success;

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      pushRobotController.applyForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      assertTrue(success);
   }

   public void testPushICPOptimizationRandomPushInSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      setupAndRunTest(createForwardWalkingFootstepMessage());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      Random random = new Random(73712L);
      double xDirection = 1.0 - 2.0 * random.nextDouble();
      double yDirection = 1.0 - 2.0 * random.nextDouble();

      // push parameters:
      Vector3D forceDirection = new Vector3D(xDirection, yDirection, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   public void testPushICPOptimizationLongForwardPushInSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      setupAndRunTest(createForwardWalkingFootstepMessage());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.1 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.7 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }

   public void testPushICPOptimizationLongBackwardPushInSwing(double percentWeight) throws SimulationExceededMaximumTimeException
   {
      setupAndRunTest(createForwardWalkingFootstepMessage());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.1 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.8 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
   }
}
