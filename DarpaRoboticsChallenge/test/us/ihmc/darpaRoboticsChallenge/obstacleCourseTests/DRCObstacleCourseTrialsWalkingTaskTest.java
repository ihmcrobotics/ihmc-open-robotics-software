package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.packets.ComHeightPacket;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCEnvironmentModel;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class DRCObstacleCourseTrialsWalkingTaskTest
{
   private static final boolean KEEP_SCS_UP = true;

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

   @Test
   public void testStepOnCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest.xml", selectedLocation, selectedEnvironment, checkNothingChanged, createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);

      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   @Test
   public void testStepOnAndOffCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;
      DRCEnvironmentModel selectedEnvironment = DRCEnvironmentModel.OBSTACLE_COURSE;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOver_LeftFootTest.xml", selectedLocation, selectedEnvironment, checkNothingChanged, createMovie);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      BooleanYoVariable doToeTouchdownIfPossible = (BooleanYoVariable) simulationConstructionSet.getVariable("doToeTouchdownIfPossible");
      doToeTouchdownIfPossible.set(true);
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createMovie(simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   private void setupCameraForWalkingOverCinderBlocks(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(13.2664, 13.03, 0.75);
      Point3d cameraPosition = new Point3d(9.50, 15.59, 1.87);

      drcSimulationTestHelper.setupCameraForUnitTest(scs, cameraFix, cameraPosition);
   }
   
   private FootstepDataList createFootstepsForSteppingOntoCinderBlocks(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{7.052070849258692, 7.500648867588069, 0.08196082621896894}, {-2.1189346524232606E-10, -9.297234061933088E-11, 0.7039868338673644, 0.7102130227906299}},
            {{7.342180295989005, 7.6436612172922655, 0.38202669843371195}, {1.173755819446261E-10, -9.442195242575743E-11, 0.7039869537528869, 0.7102129039560818}},
            {{6.921517151053708, 8.0347403416141, 0.3807625731669934}, {-2.1189345728399698E-10, -9.297233072636428E-11, 0.7039867740072752, 0.7102130821259418}}};

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataList createFootstepsForSteppingOffOfCinderBlocks(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {{{7.306229830951352, 8.350152858971871, 0.08805334061043324}, {1.1737559761691812E-10, -9.442198814451009E-11, 0.7039868340327233, 0.7102130226267206}},
            {{7.131616832213055, 8.353969465249852, 0.08803687646370233}, {-2.1189346524232603E-10, -9.297234061933072E-11, 0.703986833867364, 0.7102130227906303}}};

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
}
