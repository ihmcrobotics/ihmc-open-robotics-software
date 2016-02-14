package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndFootTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test//(timeout = 300000)
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage();

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), "", selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3d desiredPosition = new Point3d();
         Quat4d desiredOrientation = new Quat4d();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getCapturePointPlannerParameters().getDoubleSupportInitialTransferDuration());
         assertTrue(success);
         
         // Now we can do the usual test.
         double trajectoryTime = 1.0;
         FramePose desiredRandomFootPose = new FramePose(foot.getBodyFixedFrame());
         desiredRandomFootPose.setOrientation(RandomTools.generateRandomQuaternion(random, 1.0));
         desiredRandomFootPose.setPosition(RandomTools.generateRandomPoint(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
         desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         desiredRandomFootPose.getPose(desiredPosition, desiredOrientation);
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);


         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.translate(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getCapturePointPlannerParameters().getDoubleSupportInitialTransferDuration());
         assertTrue(success);
      }
   }

   public static void assertSingleWaypointExecuted(RobotSide robotSide, Point3d desiredPosition, Quat4d desiredOrientation, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();

      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String positionTrajectoryName = footPrefix + "MultipleWaypointsPositionTrajectoryGenerator";
      String orientationTrajectoryName = footPrefix + "MultipleWaypointsOrientationTrajectoryGenerator";
      String numberOfWaypointsVarName = footPrefix + "NumberOfWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      double numberOfWaypoints = scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName).getValueAsDouble();
      assertEquals(2.0, numberOfWaypoints, 0.1);
      numberOfWaypoints = scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName).getValueAsDouble();
      assertEquals(2.0, numberOfWaypoints, 0.1);

      double trajOutput = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "X").getValueAsDouble();
      assertEquals(desiredPosition.getX(), trajOutput, EPSILON_FOR_DESIREDS);
      trajOutput = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Y").getValueAsDouble();
      assertEquals(desiredPosition.getY(), trajOutput, EPSILON_FOR_DESIREDS);
      trajOutput = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Z").getValueAsDouble();
      assertEquals(desiredPosition.getZ(), trajOutput, EPSILON_FOR_DESIREDS);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qx").getValueAsDouble();
      assertEquals(desiredOrientation.getX(), trajOutput, EPSILON_FOR_DESIREDS);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qy").getValueAsDouble();
      assertEquals(desiredOrientation.getY(), trajOutput, EPSILON_FOR_DESIREDS);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qz").getValueAsDouble();
      assertEquals(desiredOrientation.getZ(), trajOutput, EPSILON_FOR_DESIREDS);
      trajOutput = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qs").getValueAsDouble();
      assertEquals(desiredOrientation.getW(), trajOutput, EPSILON_FOR_DESIREDS);
   }

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
}
