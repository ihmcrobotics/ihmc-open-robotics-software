package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import java.util.ArrayList;

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
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
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
            Point3D location = new Point3D();
            Quaternion orientation = new Quaternion();
            Vector3D surfaceNormal = new Vector3D();
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
      Point3D center = new Point3D(0.022237149581994832, 3.6888378632721963, 0.7893353089719684);
      Vector3D plusMinusVector = new Vector3D(0.4, 0.3, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForHandstepsOnWalls()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<Handstep> createHandstepForTesting(double leftHandstepY, double rightHandstepY, ScriptedHandstepGenerator scriptedHandstepGenerator)
   {
      ArrayList<Handstep> ret = new ArrayList<Handstep>();
      RobotSide robotSide = RobotSide.LEFT;
      Tuple3DBasics position = new Point3D(0.6, leftHandstepY, 1.0);
      Vector3D surfaceNormal = new Vector3D(-1.0, 0.0, 0.0);
      double rotationAngleAboutNormal = 0.0;
      double swingTrajectoryTime = 1.0;
      Handstep handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);
      robotSide = RobotSide.RIGHT;
      position = new Point3D(0.6, rightHandstepY, 1.0);
      surfaceNormal = new Vector3D(-1.0, 0.0, 0.0);
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
