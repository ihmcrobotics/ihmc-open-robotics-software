package us.ihmc.darpaRoboticsChallenge.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHandTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
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
         double trajectoryTime = 1.0;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         
         ScrewTestTools.setRandomPositionsWithinJointLimits(armClone, random);

         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose desiredRandomHandPose = new FramePose(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(ReferenceFrame.getWorldFrame());
         
         Point3d desiredPosition = new Point3d();
         Quat4d desiredOrientation = new Quat4d();
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(handTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);


         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         
         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);
      }
   }

   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testStopAllTrajectory() throws Exception
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
         double trajectoryTime = 5.0;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);

         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         
         FramePose desiredRandomHandPose = new FramePose(fullRobotModel.getHandControlFrame(robotSide));
         desiredRandomHandPose.changeFrame(ReferenceFrame.getWorldFrame());
         desiredRandomHandPose.translate(RandomTools.generateRandomVector(random, 0.2));
         
         Point3d desiredPosition = new Point3d();
         Quat4d desiredOrientation = new Quat4d();
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(handTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         
         HandControlState controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(robotSide, scs);
         assertEquals(HandControlState.TASK_SPACE_POSITION, controllerState);
         
         double timeStopSent = scs.getRobots()[0].getYoTime().getDoubleValue();
         int numberOfJoints = armJoints.length;
         double[] actualJointPositions = new double[numberOfJoints];
         double[] zeroVelocities = new double[numberOfJoints];
         for (int i = 0; i < numberOfJoints; i++)
         {
            actualJointPositions[i] = armJoints[i].getQ();
         }
         
         drcSimulationTestHelper.send(new StopAllTrajectoryMessage());

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
         assertTrue(success);


         controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(robotSide, scs);
         double switchTime = EndToEndArmTrajectoryMessageTest.findControllerSwitchTime(robotSide, scs);
         double[] controllerDesiredJointPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(robotSide, armJoints, numberOfJoints, scs);
         double[] controllerDesiredJointVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(robotSide, armJoints, numberOfJoints, scs);

         assertEquals(HandControlState.JOINT_SPACE, controllerState);
         assertEquals(timeStopSent, switchTime, getRobotModel().getControllerDT());
         assertArrayEquals(actualJointPositions, controllerDesiredJointPositions, 0.01);
         assertArrayEquals(zeroVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public static Quat4d findControllerDesiredOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String subTrajectoryName = handPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      Quat4d desiredOrientation = new Quat4d();
      desiredOrientation.x = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qx").getValueAsDouble();
      desiredOrientation.y = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qy").getValueAsDouble();
      desiredOrientation.z = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qz").getValueAsDouble();
      desiredOrientation.w = scs.getVariable(subTrajectoryName, currentOrientationVarNamePrefix + "Qs").getValueAsDouble();
      return desiredOrientation;
   }

   public static Point3d findControllerDesiredPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String subTrajectoryName = handPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      Point3d desiredPosition = new Point3d();
      desiredPosition.x = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "X").getValueAsDouble();
      desiredPosition.y = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Y").getValueAsDouble();
      desiredPosition.z = scs.getVariable(subTrajectoryName, currentPositionVarNamePrefix + "Z").getValueAsDouble();
      return desiredPosition;
   }

   public static int findNumberOfWaypointsForOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String numberOfWaypointsVarName = handPrefix + "NumberOfWaypoints";
      String orientationTrajectoryName = handPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      return ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static int findNumberOfWaypointsForPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String numberOfWaypointsVarName = handPrefix + "NumberOfWaypoints";
      String positionTrajectoryName = handPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      return ((IntegerYoVariable) scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static void assertSingleWaypointExecuted(RobotSide robotSide, Point3d desiredPosition, Quat4d desiredOrientation,
         SimulationConstructionSet scs)
   {
      assertEquals(2, findNumberOfWaypointsForPosition(robotSide, scs));
      assertEquals(2, findNumberOfWaypointsForOrientation(robotSide, scs));

      Point3d controllerDesiredPosition = findControllerDesiredPosition(robotSide, scs);
      assertEquals(desiredPosition.getX(), controllerDesiredPosition.x, EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getY(), controllerDesiredPosition.y, EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getZ(), controllerDesiredPosition.z, EPSILON_FOR_DESIREDS);

      Quat4d controllerDesiredOrientation = findControllerDesiredOrientation(robotSide, scs);
      assertEquals(desiredOrientation.getX(), controllerDesiredOrientation.x, EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getY(), controllerDesiredOrientation.y, EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getZ(), controllerDesiredOrientation.z, EPSILON_FOR_DESIREDS);
      assertEquals(desiredOrientation.getW(), controllerDesiredOrientation.w, EPSILON_FOR_DESIREDS);
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
