package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.avatar.testTools.ScriptedHandstepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.WallWorldEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCWallWorldTest implements MultiRobotTestInterface
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

   /**
    * Needs to be reimplemented.
    */
   @ContinuousIntegrationTest(estimatedDuration = 192.2)
   @Test(timeout = 960000)
   public void testVariousHandstepsOnWalls() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      double wallMaxY = 3.5;
      WallWorldEnvironment environment = new WallWorldEnvironment(-1.0, wallMaxY);
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCWalkingUpToRampShortStepsTest", selectedLocation, simulationTestingParameters,
              getRobotModel());
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();
      setupCameraForHandstepsOnWalls();
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      ScriptedHandstepGenerator scriptedHandstepGenerator = drcSimulationTestHelper.createScriptedHandstepGenerator();
      double bodyY = 0.0;
      while (bodyY < wallMaxY - 0.65)
      {
         bodyY = bodyY + 0.15;
         double leftHandstepY = bodyY + 0.25;
         double rightHandstepY = bodyY - 0.25;
         ArrayList<Handstep> handsteps = createHandstepForTesting(leftHandstepY, rightHandstepY, scriptedHandstepGenerator);
         for (Handstep handstep : handsteps)
         {
            Point3d location = new Point3d();
            Quat4d orientation = new Quat4d();
            Vector3d surfaceNormal = new Vector3d();
            handstep.getPose(location, orientation);
            handstep.getSurfaceNormal(surfaceNormal);
            HandstepPacket handstepPacket = new HandstepPacket(handstep.getRobotSide(), location, orientation, surfaceNormal,
                                               handstep.getSwingTrajectoryTime());
            drcSimulationTestHelper.send(handstepPacket);
            success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
         }

         bodyY = bodyY + 0.15;
         FootstepDataListMessage footstepDataList = createFootstepsForTwoSideSteps(bodyY, scriptedFootstepGenerator);
         drcSimulationTestHelper.send(footstepDataList);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      }

//      HandPosePacket releaseLeftHandToHome = PacketControllerTools.createGoToHomeHandPosePacket(RobotSide.LEFT, 1.0);
//      drcSimulationTestHelper.send(releaseLeftHandToHome);
//      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
//      HandPosePacket releaseRightHandToHome = PacketControllerTools.createGoToHomeHandPosePacket(RobotSide.RIGHT, 1.0);
//      drcSimulationTestHelper.send(releaseRightHandToHome);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      for (int i = 0; i < 3; i++)
      {
         bodyY = bodyY + 0.3;
         FootstepDataListMessage footstepDataList = createFootstepsForTwoSideSteps(bodyY, scriptedFootstepGenerator);
         drcSimulationTestHelper.send(footstepDataList);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);
      Point3d center = new Point3d(0.022237149581994832, 3.6888378632721963, 0.7893353089719684);
      Vector3d plusMinusVector = new Vector3d(0.4, 0.3, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForHandstepsOnWalls()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<Handstep> createHandstepForTesting(double leftHandstepY, double rightHandstepY, ScriptedHandstepGenerator scriptedHandstepGenerator)
   {
      ArrayList<Handstep> ret = new ArrayList<Handstep>();
      RobotSide robotSide = RobotSide.LEFT;
      Tuple3d position = new Point3d(0.6, leftHandstepY, 1.0);
      Vector3d surfaceNormal = new Vector3d(-1.0, 0.0, 0.0);
      double rotationAngleAboutNormal = 0.0;
      double swingTrajectoryTime = 1.0;
      Handstep handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);
      robotSide = RobotSide.RIGHT;
      position = new Point3d(0.6, rightHandstepY, 1.0);
      surfaceNormal = new Vector3d(-1.0, 0.0, 0.0);
      rotationAngleAboutNormal = 0.0;
      handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);

      return ret;
   }

   private FootstepDataListMessage createFootstepsForTwoSideSteps(double bodyY, ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double stepWidth = 0.20;
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.0, bodyY + stepWidth / 2.0, 0.084}, {0.0, 0.0, 0.0, 1.0}
         },
         {
            {0.0, bodyY - stepWidth / 2.0, 0.084}, {0.0, 0.0, 0.0, 1.0}
         }
      };
      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
}
