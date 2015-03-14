package us.ihmc.atlas;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.simulationconstructionset.DataProcessingFunction;
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


   
   @Ignore("This is just for Jerry to help debug. Will be deleted or converted to a good test later.")
   @EstimatedDuration(duration = 0.5)
   @Test(timeout = 30000)
   public void testDebugDataFile() throws UnreasonableAccelerationException
   {
      boolean visualize = true;
      final AtlasInverseDynamicsCalculatorTestHelper testHelper = new AtlasInverseDynamicsCalculatorTestHelper(visualize);


      Robot robot = testHelper.getRobot();
      SimulationConstructionSet scs = testHelper.getSimulationConstructionSet();
      SimulationTestingParameters simulationTestingParameters = testHelper.getSimulationTestingParameters();
      final SDFFullRobotModel fullRobotModel = testHelper.getFullRobotModel();


      String dataFile = "D:/EclipseWorkspace/Atlas/DataFiles/150312_DebugAtlasPelvisInverseDynamicsProblem.data.gz";
      scs.readData(dataFile );

      scs.tick();

      testHelper.setGhostStateToMatchRobot();

      ThreadTools.sleep(1000L);

      DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
      {

         @Override
         public void processData()
         {
            testHelper.setFullRobotModelStateToMatchRobot();

            //          testHelper.setFullRobotModelAccelerationToMatchRobot();
            testHelper.setFullRobotModelAccelerationToMatchRecordedYoVariables();


            //            testHelper.setFullRobotModelStateAndAccelerationToMatchRobot();



            testHelper.setFullRobotModelWrenchesToMatchRobot();
            testHelper.computeTwistCalculatorAndInverseDynamicsCalculator();

            double epsilon = 1e-7;
            boolean torquesMatch = testHelper.checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(epsilon);

            boolean computedRootJointWrenchIsZero = testHelper.checkComputedRootJointWrenchIsZero(1e-10);


            ///

            //            testHelper.setGhostRobotAccelerationBasedOnDesiredAccelerationOfFullRobotModel();
            //            testHelper.ghostRobotRecursiveEulerIntegrate(0.006);
         }

         @Override
         public void initializeProcessing()
         {

         }
      };
      scs.applyDataProcessingFunction(dataProcessingFunction);

      System.out.println("Done applying data processing function!");
      testHelper.startSimulationOnAThread();


      ThreadTools.sleepForever();
   }



}
