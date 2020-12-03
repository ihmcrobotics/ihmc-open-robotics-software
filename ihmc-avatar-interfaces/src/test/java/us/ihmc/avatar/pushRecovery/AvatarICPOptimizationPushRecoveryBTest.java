package us.ihmc.avatar.pushRecovery;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

public abstract class AvatarICPOptimizationPushRecoveryBTest extends AvatarICPOptimizationPushRecoveryTestSetup
{
   @Test
   public void testPushICPOptimizationNoPush() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      validateTest(footsteps);
   }

   @Test
   public void testPushICPOptimizationOutwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @Test
   public void testPushICPOptimizationOutwardPushInSlowSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createSlowForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:t
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @Test
   public void testPushICPOptimizationDiagonalOutwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(2.0, -1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @Test
   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createYawingForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);

      // push parameters:
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;

      Vector3D firstForceDirection = new Vector3D(0.0, -1.0, 0.0);
      Vector3D secondForceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);
      pushRobotController.queueForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);
      pushRobotController.queueForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);
      pushRobotController.queueForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);
      pushRobotController.queueForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);
      pushRobotController.queueForceDelayed(secondPushCondition, delay, secondForceDirection, magnitude, duration);
      
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.0);
      assertTrue(success);

      validateTest(footsteps, false);
   }

   @Test
   public void testPushICPOptimizationRandomPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
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

      validateTest(footsteps);
   }

   @Test
   public void testPushICPOptimizationLongForwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.1 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.7 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }

   @Test
   public void testPushICPOptimizationLongBackwardPushInSwing() throws Exception
   {
      FootstepDataListMessage footsteps = createForwardWalkingFootstepMessage();
      setupAndRunTest(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.1 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.8 * swingTime;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      validateTest(footsteps);
   }
}
