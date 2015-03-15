package us.ihmc.atlas;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

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
   public void testAtlasInverseDynamics() throws UnreasonableAccelerationException
   {
      Random random = new Random(1776L);

      double maxExternalForce = 10.0;
      double maxJointVelocity = 1.0;

      boolean visualize = false;
      AtlasInverseDynamicsCalculatorTestHelper testHelper = new AtlasInverseDynamicsCalculatorTestHelper(visualize);
      testHelper.startSimulationOnAThread();

      int numberOfTicks = 100;

      Robot robot = testHelper.getRobot();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();
      SimulationTestingParameters simulationTestingParameters = testHelper.getSimulationTestingParameters();
      FullRobotModel fullRobotModel = testHelper.getFullRobotModel();

      for (int i = 0; i < numberOfTicks; i++)
      {
         testHelper.setRobotStateRandomly(random, maxJointVelocity);
         testHelper.setRobotExternalForcesRandomly(random, maxExternalForce);
         testHelper.setRobotTorquesRandomly(random);

         robot.doDynamicsButDoNotIntegrate();
         testHelper.setFullRobotModelStateAndAccelerationToMatchRobot();

         fullRobotModel.updateFrames();

         testHelper.setFullRobotModelWrenchesToMatchRobot();

         testHelper.computeTwistCalculatorAndInverseDynamicsCalculator();

         double epsilon = 1e-7;
         boolean torquesMatch = testHelper.checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);

         assertTrue(torquesMatch);

         boolean computedRootJointWrenchIsZero = testHelper.checkComputedRootJointWrenchIsZero(1e-10);
         assertTrue(computedRootJointWrenchIsZero);


         if (scs != null)
            scs.tickAndUpdate();
      }

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
}
