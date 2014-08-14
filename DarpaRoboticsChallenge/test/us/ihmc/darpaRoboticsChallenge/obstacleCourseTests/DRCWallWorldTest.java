package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataList;
import us.ihmc.commonWalkingControlModules.packets.HandstepPacket;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01StartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedHandstepGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public abstract class DRCWallWorldTest implements MultiRobotTestInterface
{
   private static final boolean KEEP_SCS_UP = true;

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
   public void testVariousHandstepsOnWalls() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCDemo01StartingLocation selectedLocation = DRCDemo01StartingLocation.DEFAULT;

      DRCWallWorldEnvironment environment = new DRCWallWorldEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCWalkingUpToRampShortStepsTest", "", selectedLocation, checkNothingChanged,
            showGUI, createMovie, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForHandstepsOnWalls();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

//      +++JEP 140814: Exploratory. Testing handsteps. Will be moved out of here once it's all working...
      ScriptedHandstepGenerator scriptedHandstepGenerator = drcSimulationTestHelper.createScriptedHandstepGenerator();
      
      ArrayList<Handstep> handsteps = createHandstepForTesting(scriptedHandstepGenerator);
      
      for (Handstep handstep : handsteps)
      {
         HandstepPacket handstepPacket = new HandstepPacket(handstep);
         drcSimulationTestHelper.sendHandstepPacketToListeners(handstepPacket);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      }

      
      FootstepDataList footstepDataList = createFootstepsForSteppingInPlace(scriptedFootstepGenerator);
      drcSimulationTestHelper.sendFootstepListToListeners(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0);

      handsteps = createHandstepForTesting(scriptedHandstepGenerator);
      
      for (Handstep handstep : handsteps)
      {
         HandstepPacket handstepPacket = new HandstepPacket(handstep);
         drcSimulationTestHelper.sendHandstepPacketToListeners(handstepPacket);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      }

      
      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupCameraForHandstepsOnWalls()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<Handstep> createHandstepForTesting(ScriptedHandstepGenerator scriptedHandstepGenerator)
   {
      ArrayList<Handstep> ret = new ArrayList<Handstep>();

      RobotSide robotSide = RobotSide.LEFT;
      Tuple3d position = new Point3d(0.6, 0.3, 1.0);
      Vector3d surfaceNormal = new Vector3d(-1.0, 0.0, 0.0);
      double rotationAngleAboutNormal = 0.0;

      Handstep handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal);
      ret.add(handstep);
      
      robotSide = RobotSide.RIGHT;
      position = new Point3d(0.6, -0.3, 1.0);
      surfaceNormal = new Vector3d(-1.0, 0.0, 0.0);
      rotationAngleAboutNormal = 0.0;

      handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal);
      ret.add(handstep);

      return ret;
   }
   
   private FootstepDataList createFootstepsForSteppingInPlace(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.1, -0.1, 0.084},
            {0.0, 0.0, 0.0, 1.0}
         },
         {
            {0.1, 0.1, 0.084},
            {0.0, 0.0, 0.0, 1.0}
         },
         {
            {0.0, -0.1, 0.084},
            {0.0, 0.0, 0.0, 1.0}
         },
         {
            {0.0, 0.1, 0.084},
            {0.0, 0.0, 0.0, 1.0}
         },
         {
            {0.05, -0.1, 0.084},
            {0.0, 0.0, 0.0, 1.0}
         },
         {
            {0.05, 0.1, 0.084},
            {0.0, 0.0, 0.0, 1.0}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

}
