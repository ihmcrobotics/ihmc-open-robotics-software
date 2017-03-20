package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseDoNothingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
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
      
      simulationTestingParameters = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @AfterClass
   public static void garbageCollectAndPauseForYourKitToSeeWhatIsStillAllocated()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("DRCObstacleCourseDoNothingTest after class.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 11.1)
   @Test(timeout = 55000)
   public void testDoNothing1() throws SimulationExceededMaximumTimeException
   {
      doATest();
   }
   
   private void doATest() throws SimulationExceededMaximumTimeException
   {
      doATestWithDRCStuff();      
   }
   
   private void doATestWithDRCStuff() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;

      
//      new DRCDemo01NavigationEnvironment(), new ScriptedFootstepDataListObjectCommunicator("Team"), name, scriptFileName, selectedLocation, checkNothingChanged, showGUI, showGUI,
//      createVideo, false, robotModel
      
      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new DefaultCommonAvatarEnvironment();
      String name = "DRCDoNothingTest";
      

      DRCRobotModel robotModel = getRobotModel();
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(commonAvatarEnvironmentInterface, name, selectedLocation, simulationTestingParameters, robotModel);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(100);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
   
   private void doATestWithJustAnSCS() throws SimulationExceededMaximumTimeException
   {
//      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      SimulationConstructionSetParameters simulationConstructionSetParameters = new SimulationConstructionSetParameters();
      simulationConstructionSetParameters.setCreateGUI(true);
      simulationConstructionSetParameters.setShowSplashScreen(false);
      simulationConstructionSetParameters.setShowWindows(true);

      
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("TEST"), simulationConstructionSetParameters);
      
      scs.startOnAThread();
      ThreadTools.sleep(4000);
      scs.closeAndDispose();

//      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private void setupCameraForWalkingOverSmallPlatform(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-3.0, -4.6, 0.8);
      Point3D cameraPosition = new Point3D(-11.5, -5.8, 2.5);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
