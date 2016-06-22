package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndPelvisTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.2e-4;
   private static final double EPSILON_FOR_HEIGHT = 1.0e-2;

   private static final boolean DEBUG = false;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePose desiredRandomPelvisPose = new FramePose(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.setOrientation(RandomTools.generateRandomQuaternion(random, 1.0));
      desiredRandomPelvisPose.setPosition(RandomTools.generateRandomPoint(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() - 0.1);
      Point3d desiredPosition = new Point3d();
      Quat4d desiredOrientation = new Quat4d();

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      assertSingleWaypointExecuted(desiredPosition, desiredOrientation, scs);
   }

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      SDFFullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 5.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePose desiredRandomPelvisPose = new FramePose(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.setOrientation(RandomTools.generateRandomQuaternion(random, 1.0));
      desiredRandomPelvisPose.setPosition(RandomTools.generateRandomPoint(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() - 0.1);
      Point3d desiredPosition = new Point3d();
      Quat4d desiredOrientation = new Quat4d();

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.getPose(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
      assertTrue(success);

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.send(stopAllTrajectoryMessage);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertFalse(findControllerStopBooleanForOrientation(scs));
      assertFalse(findControllerStopBooleanForXY(scs));
      assertFalse(findControllerStopBooleanForHeight(scs));
      Quat4d controllerDesiredOrientationBeforeStop = findControllerDesiredWaypointOrientation(scs);
      Point2d controllerDesiredXYBeforeStop = findControllerDesiredWaypointXY(scs);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      assertTrue(success);

      assertTrue(findControllerStopBooleanForOrientation(scs));
      assertTrue(findControllerStopBooleanForXY(scs));
      assertTrue(findControllerStopBooleanForHeight(scs));
      Quat4d controllerDesiredOrientationAfterStop = findControllerDesiredWaypointOrientation(scs);
      Point2d controllerDesiredXYAfterStop = findControllerDesiredWaypointXY(scs);

      JUnitTools.assertQuaternionsEqual(controllerDesiredOrientationBeforeStop, controllerDesiredOrientationAfterStop, 1.0e-2);
      JUnitTools.assertPoint2dEquals("", controllerDesiredXYBeforeStop, controllerDesiredXYAfterStop, 1.0e-2);
   }

   public static Quat4d findControllerDesiredWaypointOrientation(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectoryName = pelvisPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      Quat4d desiredOrientation = new Quat4d();
      desiredOrientation.x = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qx").getValueAsDouble();
      desiredOrientation.y = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qy").getValueAsDouble();
      desiredOrientation.z = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qz").getValueAsDouble();
      desiredOrientation.w = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qs").getValueAsDouble();
      return desiredOrientation;
   }

   public static boolean findControllerStopBooleanForOrientation(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(PelvisOrientationManager.class.getSimpleName(), "isPelvisOrientationOffsetTrajectoryStopped")).getBooleanValue();
   }

   public static boolean findControllerStopBooleanForXY(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(PelvisICPBasedTranslationManager.class.getSimpleName(), "isPelvisTranslationalTrajectoryStopped")).getBooleanValue();
   }

   public static boolean findControllerStopBooleanForHeight(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(LookAheadCoMHeightTrajectoryGenerator.class.getSimpleName(), "isPelvisOffsetHeightTrajectoryStopped")).getBooleanValue();
   }

   public static int findControllerNumberOfWaypointsForOrientation(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String orientationTrajectoryName = pelvisPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsForXY(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String positionTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsForHeight(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisHeightOffset";
      String orientationTrajectoryName = pelvisPrefix + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static Point2d findControllerDesiredWaypointXY(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectoryName = pelvisPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      Point2d desiredXYPosition = new Point2d();
      desiredXYPosition.x = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "X").getValueAsDouble();
      desiredXYPosition.y = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Y").getValueAsDouble();
      return desiredXYPosition;
   }

   public static void assertSingleWaypointExecuted(Point3d desiredPosition, Quat4d desiredOrientation, SimulationConstructionSet scs)
   {
      assertEquals(2, findControllerNumberOfWaypointsForOrientation(scs));
      assertEquals(2, findControllerNumberOfWaypointsForXY(scs));

      Point2d desiredControllerXY = findControllerDesiredWaypointXY(scs);
      assertEquals(desiredPosition.getX(), desiredControllerXY.x, EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getY(), desiredControllerXY.y, EPSILON_FOR_DESIREDS);
      
      Quat4d desiredControllerOrientation = findControllerDesiredWaypointOrientation(scs);
      assertEquals(desiredOrientation.getX(), desiredControllerOrientation.x, EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getY(), desiredControllerOrientation.y, EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getZ(), desiredControllerOrientation.z, EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getW(), desiredControllerOrientation.w, EPSILON_FOR_DESIREDS);

      // Hard to figure out how to verify the desired there
//      trajOutput = scs.getVariable("pelvisHeightOffsetSubTrajectoryCubicPolynomialTrajectoryGenerator", "pelvisHeightOffsetSubTrajectoryCurrentValue").getValueAsDouble();
//      assertEquals(desiredPosition.getZ(), trajOutput, EPSILON_FOR_DESIREDS);
      // Ending up doing a rough check on the actual height
      double pelvisHeight = scs.getVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
      assertEquals(desiredPosition.getZ(), pelvisHeight, EPSILON_FOR_HEIGHT);
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
