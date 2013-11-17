package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCEnvironmentModel;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class DRCObstacleCourseFlatTest
{
   private static final boolean KEEP_SCS_UP = false;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();

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

   @Ignore("Invoked manually to test memory & thread leaks")
   @Test
   public void testForMemoryLeaks() throws Exception
   {
      for (int i = 0; i < 10; i++)
      {
         showMemoryUsageBeforeTest();
         testStandingForACoupleSeconds();
         destroySimulationAndRecycleMemory();
      }
   }

   @Test
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DEFAULT;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", selectedLocation, selectedEnvironment, checkNothingChanged, createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1); //2.0);
      
      ThreadTools.sleep(2000);
      
      
//      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }

   @Test
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.TOP_OF_SLOPES;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", selectedLocation, selectedEnvironment, checkNothingChanged, createMovie, true);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      
      Point3d cameraFix = new Point3d(3.25, 3.25, 1.02);
      Point3d cameraPosition = new Point3d(6.35, 0.18, 0.97);
      drcSimulationTestHelper.setupCameraForUnitTest(simulationConstructionSet, cameraFix, cameraPosition);
      
      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);
      
      
      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
//   @Test
//   public void testMemoryStuff()
//   {
//      for (int i=0; i<3; i++)
//      {
//         System.gc();
//         System.runFinalization();
//         ThreadTools.sleep(1000);
//         
//         System.out.println("Sleeping Forever");
//         ThreadTools.sleepForever();
//      }
//   }
   
   @Test
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DEFAULT;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingUpToRampShortStepsTest", selectedLocation, selectedEnvironment, checkNothingChanged, createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForWalkingUpToRampShortSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0);
      
      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   @Test
   public void testWalkingUpToRampWithLongSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DEFAULT;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingUpToRampLongStepsTest", selectedLocation, selectedEnvironment, checkNothingChanged, createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp(simulationConstructionSet);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataList footstepDataList = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);
//      FootstepDataList footstepDataList = createFootstepsForTwoLongFlatSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0);
      
      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   

   private void setupCameraForWalkingUpToRamp(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(1.56, -0.2, 0.89);
      Point3d cameraPosition = new Point3d(2.2, -7.8, 1.6);

      drcSimulationTestHelper.setupCameraForUnitTest(scs, cameraFix, cameraPosition);
   }

   

   private FootstepDataList createFootstepsForWalkingUpToRampShortSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.2148448504580547, -0.09930268518393547, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.4481532647842352, 0.10329823409587219, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.6834821762408051, -0.09551979778612019, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.9167977582017036, 0.10565710343022289, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.1521266696582735, -0.09316092845176947, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.385442251619172, 0.1080159727645736, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.620771163075742, -0.09080205911741877, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.8540867450366407, 0.11037484209892431, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.0894156564932107, -0.08844318978306806, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.322731238454109, 0.11273371143327501, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.558060149910679, -0.08608432044871735, 0.08398952447640476},
            {-5.047008501650524E-21, 4.53358964226292E-22, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.7913757318715775, 0.11509258076762573, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.0267046433281477, -0.08372545111436663, 0.08398952447640476},
            {-6.38257081820882E-21, -2.5377866560433405E-20, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.260020225289046, 0.11745145010197644, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.2610268900368817, -0.08254601644719128, 0.08398952447640476},
            {3.49577202412201E-21, 2.923107094657073E-20, 0.0025166698394258787, 0.9999968331814453}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForWalkingOnFlatLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{0.5909646234016005, 0.10243127081250579, 0.08400000000000002}, {3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856}},
            {{1.212701966120992, -0.09394691394679651, 0.084}, {1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856}},
            {{1.8317941784239657, 0.11014657591704705, 0.08619322927296164}, {8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856}},
            {{2.4535283480857237, -0.08575120920059497, 0.08069788195751608}, {-2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856}},
            {{3.073148474156348, 0.11833676240086898, 0.08590468550531082}, {4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056}},
            {{3.0729346702590505, -0.0816428320664241, 0.0812390388356}, {-8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
  
   
   private FootstepDataList createFootstepsForTwoLongFlatSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{0.5909646234016005, 0.10243127081250579, 0.08400000000000002}, {3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856}},
            {{1.212701966120992, -0.09394691394679651, 0.084}, {1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856}}
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

}
