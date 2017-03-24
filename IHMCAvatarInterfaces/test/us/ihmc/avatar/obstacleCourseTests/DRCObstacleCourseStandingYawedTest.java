package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

   public abstract class DRCObstacleCourseStandingYawedTest implements MultiRobotTestInterface
   {
      private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      
      private DRCSimulationTestHelper drcSimulationTestHelper;

      @Before
      public void showMemoryUsageBeforeTest()
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      }

      @After
      public void destroySimulationAndRecycleMemory()
      {
         if (simulationTestingParameters.getKeepSCSUp())
         {
            ThreadTools.sleepForever();
         }

         // Do this here in case a test fails. That way the memory will be recycled.
         if (drcSimulationTestHelper != null)
         {
            drcSimulationTestHelper.destroySimulation();
            drcSimulationTestHelper = null;
         }

         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      }


	@ContinuousIntegrationTest(estimatedDuration = 18.4)
	@Test(timeout = 92000)
      public void testStandingYawed() throws SimulationExceededMaximumTimeException
      {
         BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

         DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ROCKS;
         
         drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOntoRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

         SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
         ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

         setupCameraForWalkingOntoRocks(simulationConstructionSet);

         ThreadTools.sleep(1000);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
         
         drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
         drcSimulationTestHelper.checkNothingChanged();

         assertTrue(success);
         
         Point3D center = new Point3D(-2.179104505087052E-6, 2.050336483387291, 0.7874231497270643); 
         Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
         BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
         drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

         
         BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      }
      
      
      private void setupCameraForWalkingOntoRocks(SimulationConstructionSet scs)
      {
         Point3D cameraFix = new Point3D(0.1, 3.2, 0.5);
         Point3D cameraPosition = new Point3D(-2.8, 4.8, 1.5);

         drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
      }

   }
