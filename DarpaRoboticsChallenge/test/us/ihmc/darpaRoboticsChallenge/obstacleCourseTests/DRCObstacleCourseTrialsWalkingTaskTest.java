package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public abstract class DRCObstacleCourseTrialsWalkingTaskTest implements MultiRobotTestInterface
{
	private Class thisClass = DRCObstacleCourseTrialsWalkingTaskTest.class;
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
   public void testStepOnCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOn_LeftFootTest.xml"; 
      String fileName = BambooTools.getFullFilenameUsingClassRelativeURL(thisClass, scriptName);
      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", fileName, selectedLocation, checkNothingChanged, showGUI, createMovie, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.5);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);
      
      BambooTools.reportTestFinishedMessage();
   }
   
   //@Test, we don't need step on/off two layer CinderBlocks anymore
   public void testStepOnAndOffCinderBlocks() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      String scriptName = "scripts/ExerciseAndJUnitScripts/TwoCinderBlocksStepOver_LeftFootTest.xml"; 
      String fileName = BambooTools.getFullFilenameUsingClassRelativeURL(thisClass, scriptName);
      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCObstacleCourseTrialsCinderBlocksTest", fileName, selectedLocation, checkNothingChanged, showGUI, createMovie, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverCinderBlocks(simulationConstructionSet);

      ThreadTools.sleep(0);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      BooleanYoVariable doToeTouchdownIfPossible = (BooleanYoVariable) simulationConstructionSet.getVariable("doToeTouchdownIfPossible");
      doToeTouchdownIfPossible.set(true);
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(13.0);

      drcSimulationTestHelper.createMovie(getSimpleRobotName(), simulationConstructionSet, 1);
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
}
