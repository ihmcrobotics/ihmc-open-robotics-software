package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HeadTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class EndToEndHeadTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-4;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private RigidBodyBasics head;
   private RigidBodyBasics chest;
   private OneDoFJointBasics[] neckJoints;
   private int numberOfJoints;

   @Test
   public void testSingleWaypoint()
   {
      setupTest();

      Random random = new Random(564574L);
      double trajectoryTime = 1.0;

      MultiBodySystemRandomTools.nextStateWithinJointLimits(random, JointStateType.CONFIGURATION, neckJoints);
      RigidBodyBasics headClone = neckJoints[numberOfJoints - 1].getSuccessor();
      FrameQuaternion desiredRandomChestOrientation = new FrameQuaternion(headClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion desiredOrientation = new Quaternion(desiredRandomChestOrientation);
      ReferenceFrame chestCoMFrame = chest.getBodyFixedFrame();
      HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime, desiredOrientation, chestCoMFrame);
      headTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      simulationTestHelper.publishToController(headTrajectoryMessage);

      boolean success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      CommonHumanoidReferenceFrames humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getChestFrame());

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      assertSingleWaypointExecuted(desiredRandomChestOrientation, head.getName(), simulationTestHelper);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   public void testLookingLeftAndRight()
   {
      setupTest();

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      CommonHumanoidReferenceFrames humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      FrameQuaternion lookStraightAhead = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), new Quaternion());
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookLeftQuat = new Quaternion();
      lookLeftQuat.appendYawRotation(Math.PI / 4.0);
      lookLeftQuat.appendPitchRotation(Math.PI / 16.0);
      lookLeftQuat.appendRollRotation(-Math.PI / 16.0);
      FrameQuaternion lookLeft = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookRightQuat = new Quaternion();
      lookRightQuat.appendYawRotation(-Math.PI / 4.0);
      lookRightQuat.appendPitchRotation(-Math.PI / 16.0);
      lookRightQuat.appendRollRotation(Math.PI / 16.0);
      FrameQuaternion lookRight = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());

      ReferenceFrame chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();

      HeadTrajectoryMessage lookStraightAheadMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime,
                                                                                                        lookStraightAhead,
                                                                                                        ReferenceFrame.getWorldFrame(),
                                                                                                        chestCoMFrame);
      simulationTestHelper.publishToController(lookStraightAheadMessage);
      assertTrue(simulationTestHelper.simulateNow(trajectoryTime + 0.1));

      HeadTrajectoryMessage lookLeftMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime,
                                                                                               lookLeft,
                                                                                               ReferenceFrame.getWorldFrame(),
                                                                                               chestCoMFrame);
      simulationTestHelper.publishToController(lookLeftMessage);
      assertTrue(simulationTestHelper.simulateNow(trajectoryTime + 0.1));

      HeadTrajectoryMessage lookRightMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryTime,
                                                                                                lookRight,
                                                                                                ReferenceFrame.getWorldFrame(),
                                                                                                chestCoMFrame);
      simulationTestHelper.publishToController(lookRightMessage);
      assertTrue(simulationTestHelper.simulateNow(trajectoryTime + 0.1));

      simulationTestHelper.publishToController(lookStraightAheadMessage);
      assertTrue(simulationTestHelper.simulateNow(trajectoryTime + 0.1));

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), environment, simulationTestingParameters);
      simulationTestHelper.start();
      ThreadTools.sleep(1000);
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      head = fullRobotModel.getHead();
      chest = fullRobotModel.getChest();
      neckJoints = MultiBodySystemTools.createOneDoFJointPath(chest, head);
      numberOfJoints = neckJoints.length;

      simulationTestHelper.setCamera(new Point3D(0.0, 0.0, 0.4), new Point3D(5.0, 0.0, 2.0));

      assertTrue(simulationTestHelper.simulateNow(1.0));
   }

   public static Quaternion findControllerDesiredOrientation(String bodyName, YoVariableHolder yoVariableHolder)
   {
      return EndToEndTestTools.findQuaternion("FeedbackControllerToolbox", bodyName + "DesiredOrientation", yoVariableHolder);
   }

   public static int findNumberOfWaypoints(String bodyName, YoVariableHolder yoVariableHolder)
   {
      String numberOfWaypointsVarName = bodyName + "NumberOfWaypoints";
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      return ((YoInteger) yoVariableHolder.findVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static void assertSingleWaypointExecuted(QuaternionReadOnly desiredOrientation, String bodyName, YoVariableHolder yoVariableHolder)
   {
      assertEquals(2, findNumberOfWaypoints(bodyName, yoVariableHolder));
      Quaternion controllerDesiredOrientation = findControllerDesiredOrientation(bodyName, yoVariableHolder);
      EuclidCoreTestTools.assertEquals(desiredOrientation, controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      head = null;
      chest = null;
      neckJoints = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
