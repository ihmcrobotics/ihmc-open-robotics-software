package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.SdfLoader.models.FullRobotModel;

public abstract class DRCInverseDynamicsCalculatorTest implements MultiRobotTestInterface
{
	@EstimatedDuration(duration = 0.5)
   @Test(timeout = 30000)
   public void testInverseDynamicsStartingWithRandomTorquesInSCS() throws UnreasonableAccelerationException
   {
      Random random = new Random(1776L);

      double maxGroundContactPointForce = 10.0;
      double maxFeetExternalForce = 10.0;
      double maxFeetExternalTorque = 10.0;
      double maxRootJointExternalForceAndTorque = 10.0;

      double maxJointVelocity = 1.0;
      double maxRootJointLinearAndAngularVelocity = 1.0;

      double maxJointTorque = 10.0;

      boolean visualize = false;
      boolean makeAssertions = true;

      double gravityZ = 9.81;
      DRCInverseDynamicsCalculatorTestHelper testHelper = createInverseDynamicsCalculatorTestHelper(visualize, gravityZ);
      testHelper.startSimulationOnAThread();

      int numberOfTicks = 1000;

      Robot robot = testHelper.getRobot();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();
      SimulationTestingParameters simulationTestingParameters = testHelper.getSimulationTestingParameters();
      FullRobotModel fullRobotModel = testHelper.getFullRobotModel();

      for (int i = 0; i < numberOfTicks; i++)
      {
         if (i > numberOfTicks / 2)
         {
            // For half of the ticks, don't exert external forces on the root body and make sure that the computed wrench on the body is zero
            maxRootJointExternalForceAndTorque = 0.0;
         }

         testHelper.setRobotStateRandomly(random, maxJointVelocity, maxRootJointLinearAndAngularVelocity);
         testHelper.setRobotExternalForcesRandomly(random, maxGroundContactPointForce, maxFeetExternalForce, maxFeetExternalTorque);

         testHelper.setRobotTorquesRandomly(random, maxJointTorque);
         testHelper.setRobotRootJointExternalForcesRandomly(random, maxRootJointExternalForceAndTorque);

         robot.doDynamicsButDoNotIntegrate();
         testHelper.setFullRobotModelStateAndAccelerationToMatchRobot();

         fullRobotModel.updateFrames();

         testHelper.setFullRobotModelWrenchesToMatchRobot();

         testHelper.computeTwistCalculatorAndInverseDynamicsCalculator();

         double epsilon = 1e-7;
         boolean torquesMatch = testHelper.checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);

         if (makeAssertions)
            assertTrue(torquesMatch);

         boolean computedRootJointWrenchIsZero = testHelper.checkComputedRootJointWrenchIsZero(1e-10);
         if (makeAssertions && (Math.abs(maxRootJointExternalForceAndTorque) < 1e-7))
            assertTrue(computedRootJointWrenchIsZero);


         if (scs != null)
            scs.tickAndUpdate();
      }

      cropSCSBuffer(scs, simulationTestingParameters);
   }


	@EstimatedDuration(duration = 0.2)
   @Test(timeout = 30000)
   public void testInverseDynamicsStartingWithRandomAccelerationsInInverseDynamics() throws UnreasonableAccelerationException
   {
      Random random = new Random(1984L);

      double maxPelvisLinearAcceleration = 1.0;
      double maxPelvisAngularAcceleration = 1.0;
      double maxJointAcceleration = 1.0;

      double maxFeetExternalForce = 10.0;
      double maxFeetExternalTorque = 10.0;

      double maxJointVelocity = 1.0;
      double maxRootJointLinearAndAngularVelocity = 1.0;

      boolean visualize = false;
      boolean makeAssertions = true;

      double gravityZ = 9.81;
      DRCInverseDynamicsCalculatorTestHelper testHelper = createInverseDynamicsCalculatorTestHelper(visualize, gravityZ);
      testHelper.startSimulationOnAThread();


      int numberOfTicks = 100;

      Robot robot = testHelper.getRobot();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();
      SimulationTestingParameters simulationTestingParameters = testHelper.getSimulationTestingParameters();
      FullRobotModel fullRobotModel = testHelper.getFullRobotModel();

      for (int i = 0; i < numberOfTicks; i++)
      {
         fullRobotModel.updateFrames();

         testHelper.setFullRobotModelStateRandomly(random, maxJointVelocity, maxRootJointLinearAndAngularVelocity);
         testHelper.setFullRobotModelExternalForcesRandomly(random, maxFeetExternalForce, maxFeetExternalTorque);
         testHelper.setFullRobotModelAccelerationRandomly(random, maxPelvisLinearAcceleration, maxPelvisAngularAcceleration, maxJointAcceleration);

         fullRobotModel.updateFrames();

         testHelper.computeTwistCalculatorAndInverseDynamicsCalculator();

         testHelper.setRobotStateToMatchFullRobotModel();
         testHelper.setRobotsExternalForcesToMatchFullRobotModel();
         testHelper.setRobotTorquesToMatchFullRobotModel();

         robot.doDynamicsButDoNotIntegrate();

         double epsilon = 1e-7;
         boolean accelerationsMatch = testHelper.checkAccelerationsMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);
         if (makeAssertions)
            assertTrue(accelerationsMatch);

         if (scs != null)
            scs.tickAndUpdate();
      }

      cropSCSBuffer(scs, simulationTestingParameters);
   }


   private void cropSCSBuffer(SimulationConstructionSet scs, SimulationTestingParameters simulationTestingParameters)
   {
      if (scs != null)
      {
         scs.gotoInPointNow();
         scs.tick(2);
         ThreadTools.sleep(100L);
         scs.setInPoint();
         ThreadTools.sleep(100L);
         scs.cropBuffer();

         if (simulationTestingParameters.getKeepSCSUp())
         {
            ThreadTools.sleepForever();
         }
      }
   }
      
   public DRCInverseDynamicsCalculatorTestHelper createInverseDynamicsCalculatorTestHelper(boolean visualize, double gravityZ)
   {
      DRCRobotModel drcRobotModel = getRobotModel();
      SDFFullHumanoidRobotModel fullRobotModel = drcRobotModel.createFullRobotModel();

      boolean createCollisionMeshes = false;
      drcRobotModel.setEnableJointDamping(false);
      SDFHumanoidRobot robot = drcRobotModel.createSdfRobot(createCollisionMeshes);
      robot.setGravity(gravityZ);

      return new DRCInverseDynamicsCalculatorTestHelper(fullRobotModel, robot, visualize, gravityZ);
   }
}
