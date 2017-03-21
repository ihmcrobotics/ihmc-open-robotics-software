package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.*;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.*;

import java.util.Random;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.head.HeadOrientationManager;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHeadTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-5;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RigidBody head;
   private RigidBody chest;
   private OneDoFJoint[] neckJoints;
   private int numberOfJoints;

   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      Random random = new Random(564574L);
      double trajectoryTime = 1.0;

      ScrewTestTools.setRandomPositionsWithinJointLimits(neckJoints, random);
      RigidBody headClone = neckJoints[numberOfJoints - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(headClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      ReferenceFrame chestCoMFrame = chest.getBodyFixedFrame();
      HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(trajectoryTime, desiredOrientation, ReferenceFrame.getWorldFrame(), chestCoMFrame);
      drcSimulationTestHelper.send(headTrajectoryMessage);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getChestFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      assertSingleWaypointExecuted(desiredRandomChestOrientation.getQuaternion(), head.getName(), scs);
   }

   public void testLookingLeftAndRight() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      FrameOrientation lookStraightAhead = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame(), new Quaternion());
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookLeftQuat = new Quaternion();
      lookLeftQuat.appendYawRotation(Math.PI / 4.0);
      lookLeftQuat.appendPitchRotation(Math.PI / 16.0);
      lookLeftQuat.appendRollRotation(-Math.PI / 16.0);
      FrameOrientation lookLeft = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookRightQuat = new Quaternion();
      lookRightQuat.appendYawRotation(-Math.PI / 4.0);
      lookRightQuat.appendPitchRotation(-Math.PI / 16.0);
      lookRightQuat.appendRollRotation(Math.PI / 16.0);
      FrameOrientation lookRight = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());
      
      ReferenceFrame chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();

      HeadTrajectoryMessage lookStraightAheadMessage = new HeadTrajectoryMessage(trajectoryTime, lookStraightAhead.getQuaternion(), ReferenceFrame.getWorldFrame(), chestCoMFrame);
      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.1));

      HeadTrajectoryMessage lookLeftMessage = new HeadTrajectoryMessage(trajectoryTime, lookLeft.getQuaternion(), ReferenceFrame.getWorldFrame(), chestCoMFrame);
      drcSimulationTestHelper.send(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.1));

      HeadTrajectoryMessage lookRightMessage = new HeadTrajectoryMessage(trajectoryTime, lookRight.getQuaternion(), ReferenceFrame.getWorldFrame(), chestCoMFrame);
      drcSimulationTestHelper.send(lookRightMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.1));

      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.1));
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());
      ThreadTools.sleep(1000);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      head = fullRobotModel.getHead();
      chest = fullRobotModel.getChest();
      neckJoints = ScrewTools.createOneDoFJointPath(chest, head);
      numberOfJoints = neckJoints.length;

      drcSimulationTestHelper.getSimulationConstructionSet().hideAllYoGraphics();
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(5.0, 0.0, 2.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(0.0, 0.0, 0.4);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }

   @SuppressWarnings("unchecked")
   public static HeadControlMode findControllerState(SimulationConstructionSet scs)
   {
      String headOrientatManagerName = HeadOrientationManager.class.getSimpleName();
      String headControlStateName = "headControlState";
      return ((EnumYoVariable<HeadControlMode>) scs.getVariable(headOrientatManagerName, headControlStateName)).getEnumValue();
   }

   public static double findControllerSwitchTime(SimulationConstructionSet scs)
   {
      String headOrientatManagerName = HeadOrientationManager.class.getSimpleName();
      String headControlStateName = "headControlState";
      return scs.getVariable(headOrientatManagerName, headControlStateName + "SwitchTime").getValueAsDouble();
   }

   public static Quaternion findControllerDesiredOrientation(String bodyName, SimulationConstructionSet scs)
   {
      return findQuat4d("FeedbackControllerToolbox", bodyName + "DesiredOrientation", scs);
   }

   public static int findNumberOfWaypoints(String bodyName, SimulationConstructionSet scs)
   {
      String numberOfWaypointsVarName = bodyName + "NumberOfWaypoints";
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      return ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static void assertSingleWaypointExecuted(Quaternion desiredOrientation, String bodyName, SimulationConstructionSet scs)
   {
      assertEquals(2, findNumberOfWaypoints(bodyName, scs));
      Quaternion controllerDesiredOrientation = findControllerDesiredOrientation(bodyName, scs);
      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientation, controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
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
