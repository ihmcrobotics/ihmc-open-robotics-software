package us.ihmc.atlas;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.allocations.AllocationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.DRCControllerThread;

public class AtlasAllocationTest implements AllocationTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper testHelper;

   @ContinuousIntegrationTest(estimatedDuration = 20.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 100000)
   public void testForAllocations() throws SimulationExceededMaximumTimeException, IOException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      testHelper.createSimulation(getClass().getSimpleName());
      testHelper.simulateAndBlockAndCatchExceptions(0.25);

      List<Throwable> allocations = runAndCollectAllocations(() -> {
         try
         {
            testHelper.simulateAndBlock(0.25);
         }
         catch (Exception e)
         {
            Assert.fail(e.getMessage());
         }
      });

      if (!allocations.isEmpty())
      {
         allocations.forEach(allocation -> allocation.printStackTrace());
         Assert.fail("Found allocations in the controller.");
      }
   }

   @Override
   public List<Class<?>> getClassesOfInterest()
   {
      List<Class<?>> classesOfInterest = new ArrayList<>();
      classesOfInterest.add(DRCControllerThread.class);
      classesOfInterest.add(DRCEstimatorThread.class);
      return classesOfInterest;
   }

   @Override
   public List<Class<?>> getClassesToIgnore()
   {
      List<Class<?>> classesToIgnore = new ArrayList<>();
      classesToIgnore.add(MirroredYoVariableRegistry.class);
      return classesToIgnore;
   }

   @Before
   public void before()
   {
      AllocationTest.checkInstrumentation();

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());

      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }
   }
}
