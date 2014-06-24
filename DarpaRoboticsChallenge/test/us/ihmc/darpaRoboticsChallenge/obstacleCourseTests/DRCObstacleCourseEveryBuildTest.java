package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public abstract class DRCObstacleCourseEveryBuildTest implements MultiRobotTestInterface
{
   private static final boolean KEEP_SCS_UP = false;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;
   
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
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

   @Test
   public void testSimpleFlatGroundScript() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DEFAULT;

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      String fileName = BambooTools.getFullFilenameUsingClassRelativeURL(DRCObstacleCourseFlatTest.class, scriptName);
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCSimpleFlatGroundScriptTest", fileName, selectedLocation,
              checkNothingChanged, showGUI, createMovie, getRobotModel());
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }


   @Test
   public void testWalkingUpToRampWithLongSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingUpToRampLongStepsTest", "", selectedLocation, checkNothingChanged,
            showGUI, createMovie, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);

      // FootstepDataList footstepDataList = createFootstepsForTwoLongFlatSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

 


   private void setupCameraForWalkingUpToRamp(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(scs, cameraFix, cameraPosition);
   }

  

   private FootstepDataList createFootstepsForWalkingOnFlatLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.5909646234016005, 0.10243127081250579, 0.08400000000000002},
            {3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856}
         },
         {
            {1.212701966120992, -0.09394691394679651, 0.084}, {1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856}
         },
         {
            {1.8317941784239657, 0.11014657591704705, 0.08619322927296164},
            {8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856}
         },
         {
            {2.4535283480857237, -0.08575120920059497, 0.08069788195751608},
            {-2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856}
         },
         {
            {3.073148474156348, 0.11833676240086898, 0.08590468550531082},
            {4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056}
         },
         {
            {3.0729346702590505, -0.0816428320664241, 0.0812390388356}, {-8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   
}
