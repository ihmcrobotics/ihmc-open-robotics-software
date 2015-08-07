package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

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

	@EstimatedDuration(duration = 18.8)
   @Test(timeout = 94112)
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
      BambooTools.reportTestStartedMessage();

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.SMALL_PLATFORM;

      
//      new DRCDemo01NavigationEnvironment(), new ScriptedFootstepDataListObjectCommunicator("Team"), name, scriptFileName, selectedLocation, checkNothingChanged, showGUI, showGUI,
//      createMovie, false, robotModel
      
      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new DRCDemo01NavigationEnvironment();
      String name = "DRCDoNothingTest";
      String scriptFileName = "";
      

      DRCRobotModel robotModel = getRobotModel();
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(commonAvatarEnvironmentInterface, name, scriptFileName, selectedLocation, simulationTestingParameters, robotModel);
//      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCDoNothingTest", "", selectedLocation, checkNothingChanged, showGUI, createMovie,
//            getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingOverSmallPlatform(simulationConstructionSet);

      ThreadTools.sleep(100);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 2);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      BambooTools.reportTestFinishedMessage();
   }
   
   private void doATestWithJustAnSCS() throws SimulationExceededMaximumTimeException
   {
//      BambooTools.reportTestStartedMessage();

      SimulationConstructionSetParameters simulationConstructionSetParameters = new SimulationConstructionSetParameters();
      simulationConstructionSetParameters.setCreateGUI(true);
      simulationConstructionSetParameters.setShowSplashScreen(false);
      simulationConstructionSetParameters.setShowWindows(true);

      
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("TEST"), simulationConstructionSetParameters);
      
      scs.startOnAThread();
      ThreadTools.sleep(4000);
      scs.closeAndDispose();

//      BambooTools.reportTestFinishedMessage();
   }


   private void setupCameraForWalkingOverSmallPlatform(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(-3.0, -4.6, 0.8);
      Point3d cameraPosition = new Point3d(-11.5, -5.8, 2.5);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
