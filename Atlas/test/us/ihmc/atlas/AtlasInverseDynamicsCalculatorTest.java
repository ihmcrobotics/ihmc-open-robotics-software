package us.ihmc.atlas;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCInverseDynamicsCalculatorTestHelper;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

public class AtlasInverseDynamicsCalculatorTest
{
   @EstimatedDuration(duration = 0.5)
   @Test(timeout = 30000)
   public void testAtlasInverseDynamicsStartingWithRandomTorquesInSCS() throws UnreasonableAccelerationException
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
      DRCInverseDynamicsCalculatorTestHelper testHelper = createAtlasInverseDynamicsCalculatorTestHelperUsingAtlasUnplugged(visualize, gravityZ);
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


   @EstimatedDuration(duration = 0.5)
   @Test(timeout = 30000)
   public void testAtlasInverseDynamicsStartingWithRandomAccelerationsInInverseDynamics() throws UnreasonableAccelerationException
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
      DRCInverseDynamicsCalculatorTestHelper testHelper = createAtlasInverseDynamicsCalculatorTestHelperUsingAtlasUnplugged(visualize, gravityZ);
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
   
   public static DRCInverseDynamicsCalculatorTestHelper createAtlasInverseDynamicsCalculatorTestHelperUsingAtlasUnplugged(boolean visualize, double gravityZ)
   {
      boolean headless = false;
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasTarget.SIM, headless);
      SDFFullRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();

      boolean createCollisionMeshes = false;
      atlasRobotModel.setEnableJointDamping(false);
      SDFRobot robot = atlasRobotModel.createSdfRobot(createCollisionMeshes);
      robot.setGravity(gravityZ);

      return new DRCInverseDynamicsCalculatorTestHelper(fullRobotModel, robot, visualize, gravityZ);
   }
}
