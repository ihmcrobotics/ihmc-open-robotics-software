package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.OneDoFTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class EndToEndArmTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   protected double getTimePerWaypoint()
   {
      return 0.5;
   }

   protected boolean isRobotJointDampingEnabled()
   {
      return false;
   }

   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      createSimulationTestHelper();
      simulationTestHelper.start();

      List<JointspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, statusMessages::add);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 0.5;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
         double[] desiredJointPositions = generateRandomJointPositions(random, armJoints);
         double[] desiredJointVelocities = new double[numberOfJoints];
         long sequenceID = random.nextLong();

         generateRandomJointPositions(random, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
         armTrajectoryMessage.setSequenceId(sequenceID);

         if (DEBUG)
         {
            for (int i = 0; i < numberOfJoints; i++)
            {
               OneDoFJointBasics armJoint = armJoints[i];
               System.out.println(armJoint.getName() + ": q = " + armJoint.getQ());
            }
         }

         simulationTestHelper.publishToController(armTrajectoryMessage);

         success = simulationTestHelper.simulateAndWait(1.0 + trajectoryTime);
         assertTrue(success);

         EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(armJointNames,
                                                                       desiredJointPositions,
                                                                       desiredJointVelocities,
                                                                       epsilon,
                                                                       simulationTestHelper.getControllerRegistry());

         assertEquals(2, statusMessages.size());
         JointspaceTrajectoryStatusMessage startedStatus = statusMessages.remove(0);
         JointspaceTrajectoryStatusMessage completedStatus = statusMessages.remove(0);
         EndToEndTestTools.assertJointspaceTrajectoryStatus(sequenceID,
                                                            TrajectoryExecutionStatus.STARTED,
                                                            0.0,
                                                            armJointNames,
                                                            startedStatus,
                                                            getRobotModel().getControllerDT());
         EndToEndTestTools.assertJointspaceTrajectoryStatus(sequenceID,
                                                            TrajectoryExecutionStatus.COMPLETED,
                                                            trajectoryTime,
                                                            desiredJointPositions,
                                                            armJointNames,
                                                            completedStatus,
                                                            1.0e-12,
                                                            getRobotModel().getControllerDT());
      }

      simulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void createSimulationTestHelper()
   {
      SCS2AvatarTestingSimulationFactory testSimulationFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                       simulationTestingParameters);
      testSimulationFactory.setEnableSimulatedRobotDamping(isRobotJointDampingEnabled());
      simulationTestHelper = testSimulationFactory.createAvatarTestingSimulation();
   }

   @Test
   public void testForceExecutionWithSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      createSimulationTestHelper();
      simulationTestHelper.start();

      List<JointspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, statusMessages::add);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      // Sending a bunch of footsteps
      simulationTestHelper.publishToController(EndToEndTestTools.generateStepsInPlace(simulationTestHelper.getControllerFullRobotModel(), 10));
      success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 0.5;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
         double[] desiredJointPositions = generateRandomJointPositions(random, armJoints);
         double[] desiredJointVelocities = new double[numberOfJoints];
         long sequenceID = random.nextLong();

         generateRandomJointPositions(random, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
         armTrajectoryMessage.setForceExecution(true);
         armTrajectoryMessage.setSequenceId(sequenceID);

         if (DEBUG)
         {
            for (int i = 0; i < numberOfJoints; i++)
            {
               OneDoFJointBasics armJoint = armJoints[i];
               System.out.println(armJoint.getName() + ": q = " + armJoint.getQ());
            }
         }

         simulationTestHelper.publishToController(armTrajectoryMessage);

         success = simulationTestHelper.simulateAndWait(1.0 + trajectoryTime);
         assertTrue(success);

         EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(armJointNames,
                                                                       desiredJointPositions,
                                                                       desiredJointVelocities,
                                                                       epsilon,
                                                                       simulationTestHelper.getControllerRegistry());

         assertEquals(2, statusMessages.size());
         JointspaceTrajectoryStatusMessage startedStatus = statusMessages.remove(0);
         JointspaceTrajectoryStatusMessage completedStatus = statusMessages.remove(0);
         EndToEndTestTools.assertJointspaceTrajectoryStatus(sequenceID,
                                                            TrajectoryExecutionStatus.STARTED,
                                                            0.0,
                                                            armJointNames,
                                                            startedStatus,
                                                            getRobotModel().getControllerDT());
         EndToEndTestTools.assertJointspaceTrajectoryStatus(sequenceID,
                                                            TrajectoryExecutionStatus.COMPLETED,
                                                            trajectoryTime,
                                                            desiredJointPositions,
                                                            armJointNames,
                                                            completedStatus,
                                                            1.0e-12,
                                                            getRobotModel().getControllerDT());
      }
      simulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      createSimulationTestHelper();
      simulationTestHelper.start();

      List<JointspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = getTimePerWaypoint();
      int numberOfTrajectoryPoints = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

      SideDependentList<OneDoFJointBasics[]> armsJoints = new SideDependentList<>();
      SideDependentList<ArmTrajectoryMessage> armTrajectoryMessages = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = generateRandomArmTrajectoryMessage(random, numberOfTrajectoryPoints, trajectoryTime, robotSide, armJoints);
         armTrajectoryMessage.setSequenceId(random.nextLong());
         armTrajectoryMessages.put(robotSide, armTrajectoryMessage);
         simulationTestHelper.publishToController(armTrajectoryMessage);
      }

      success = simulationTestHelper.simulateAndWait(2 * controllerDT); // Not sure why, but the controller needs 2*dt to initialize.
      assertTrue(success);

      assertEquals(2, statusMessages.size());

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
         ArmTrajectoryMessage armTrajectoryMessage = armTrajectoryMessages.get(robotSide);

         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(numberOfTrajectoryPoints + 1,
                                                                           fullRobotModel.getHand(robotSide).getName(),
                                                                           armJointNames,
                                                                           simulationTestHelper.getControllerRegistry());

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];
            OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex);

            int numberOfPointToCheck = Math.min(jointTrajectoryMessage.getTrajectoryPoints().size(), RigidBodyJointspaceControlState.maxPointsInGenerator - 1);
            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfPointToCheck; trajectoryPointIndex++)
            {
               TrajectoryPoint1DMessage expectedTrajectoryPoint = jointTrajectoryMessage.getTrajectoryPoints().get(trajectoryPointIndex);
               OneDoFTrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(armJoint,
                                                                                     trajectoryPointIndex + 1,
                                                                                     simulationTestHelper.getControllerRegistry());
               assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), epsilon);
               assertEquals(expectedTrajectoryPoint.getPosition(), controllerTrajectoryPoint.getPosition(), epsilon);
               assertEquals(expectedTrajectoryPoint.getVelocity(), controllerTrajectoryPoint.getVelocity(), epsilon);
            }
         }

         JointspaceTrajectoryStatusMessage startedStatus = statusMessages.stream()
                                                                         .filter(m -> m.getTrajectoryExecutionStatus() == TrajectoryExecutionStatus.STARTED.toByte())
                                                                         .filter(m -> m.getJointNames().getString(0).equals(armJoints[0].getName())).findFirst()
                                                                         .get();

         EndToEndTestTools.assertJointspaceTrajectoryStatus(armTrajectoryMessage.getSequenceId(),
                                                            TrajectoryExecutionStatus.STARTED,
                                                            0.0,
                                                            armJointNames,
                                                            startedStatus,
                                                            controllerDT);
      }

      statusMessages.clear();

      success = simulationTestHelper.simulateAndWait(2.0 + trajectoryTime);
      assertTrue(success);

      assertEquals(2, statusMessages.size());

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);

         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
            desiredJointPositions[jointIndex] = armTrajectoryMessages.get(robotSide).getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex)
                                                                     .getTrajectoryPoints().getLast().getPosition();

         EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(armJointNames,
                                                                       desiredJointPositions,
                                                                       desiredJointVelocities,
                                                                       epsilon,
                                                                       simulationTestHelper.getControllerRegistry());

         JointspaceTrajectoryStatusMessage completedStatus = statusMessages.stream()
                                                                           .filter(m -> m.getTrajectoryExecutionStatus() == TrajectoryExecutionStatus.COMPLETED.toByte())
                                                                           .filter(m -> m.getJointNames().getString(0).equals(armJoints[0].getName()))
                                                                           .findFirst().get();
         EndToEndTestTools.assertJointspaceTrajectoryStatus(armTrajectoryMessages.get(robotSide).getSequenceId(),
                                                            TrajectoryExecutionStatus.COMPLETED,
                                                            trajectoryTime + getTimePerWaypoint(),
                                                            desiredJointPositions,
                                                            armJointNames,
                                                            completedStatus,
                                                            1.0e-12,
                                                            controllerDT);
      }

      simulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      Random random = new Random(34536);
      simulationTestingParameters.setRunMultiThreaded(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      createSimulationTestHelper();
      simulationTestHelper.start();

      List<JointspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
      String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(MultiBodySystemTools.createOneDoFJointPath(chest, hand));

      {
         int numberOfPoints = RigidBodyJointspaceControlState.maxPoints;
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            message.getJointspaceTrajectory().getJointTrajectoryMessages().add();
         double time = 0.05 + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            {
               message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx).getTrajectoryPoints().add()
                      .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(time, 0.0, 0.0));
            }
            time = time + 0.05;
         }
         simulationTestHelper.publishToController(message);

         success = simulationTestHelper.simulateAndWait(8.0 * controllerDT);
         assertTrue(success);

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(1, bodyName, armJointNames, simulationTestHelper.getControllerRegistry());
         assertEquals(0, statusMessages.size(), "Did not expect a status, but got: " + statusMessages.toString());
      }

      {
         int numberOfPoints = RigidBodyJointspaceControlState.maxPoints - 1;
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            message.getJointspaceTrajectory().getJointTrajectoryMessages().add();
         double time = 0.05 + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
            {
               message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx).getTrajectoryPoints().add()
                      .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(time, 0.0, 0.0));
            }
            time = time + 0.05;
         }
         message.setSequenceId(random.nextLong());
         simulationTestHelper.publishToController(message);

         success = simulationTestHelper.simulateAndWait(8.0 * controllerDT);
         assertTrue(success);

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(RigidBodyJointspaceControlState.maxPoints,
                                                                           bodyName,
                                                                           armJointNames,
                                                                           simulationTestHelper.getControllerRegistry());
         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertJointspaceTrajectoryStatus(message.getSequenceId(),
                                                            TrajectoryExecutionStatus.STARTED,
                                                            0.0,
                                                            armJointNames,
                                                            statusMessages.remove(0),
                                                            controllerDT);
      }
   }

   @Test
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      createSimulationTestHelper();
      simulationTestHelper.start();
      List<JointspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);
      LogTools.info("simulated for 0.5");

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = getTimePerWaypoint();
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;

      OneDoFJointBasics[] armJoints;
      List<ArmTrajectoryMessage> armTrajectoryMessages;

      long id = 1264L;
      RobotSide robotSide = RobotSide.LEFT;

      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
      String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);

      armTrajectoryMessages = new ArrayList<>();

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         ArmTrajectoryMessage trajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
         trajectoryMessage.setSequenceId(random.nextLong());
         trajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
         if (messageIndex > 0)
         {
            trajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            trajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setPreviousMessageId(id - 1);
         }
         id++;

         OneDoFTrajectoryPointCalculator trajectoryPoint1DCalculator = new OneDoFTrajectoryPointCalculator();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJointBasics joint = armJoints[jointIndex];
            OneDoFJointTrajectoryMessage jointTrajectoryMessage = trajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();
            trajectoryPoint1DCalculator.clear();

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
               trajectoryPoint1DCalculator.appendTrajectoryPoint(timePerWaypoint * trajectoryPointIndex, desiredJointPosition);
            }

            trajectoryPoint1DCalculator.compute(timePerWaypoint * (numberOfTrajectoryPoints - 1));
            OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
            trajectoryData.addTimeOffset(getTimePerWaypoint());

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
               jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
            }
         }

         armTrajectoryMessages.add(trajectoryMessage);
         simulationTestHelper.publishToController(trajectoryMessage);

         success = simulationTestHelper.simulateAndWait(controllerDT);

         LogTools.info("simulated for controllerDT");
         assertTrue(success);
      }

      success = simulationTestHelper.simulateAndWait(controllerDT);
      LogTools.info("simulated for controllerDT");
      assertTrue(success);

      int totalNumberOfPoints = numberOfTrajectoryPoints * numberOfMessages + 1;
      boolean firstSegment = true;

      while (true)
      {
         int expectedNumberOfPointsInGenerator = Math.min(totalNumberOfPoints, RigidBodyJointspaceControlState.maxPointsInGenerator);
         if (firstSegment)
            expectedNumberOfPointsInGenerator = Math.min(RigidBodyJointspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);
         int expectedPointsInQueue = totalNumberOfPoints - expectedNumberOfPointsInGenerator;

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(totalNumberOfPoints,
                                                                           bodyName,
                                                                           armJointNames,
                                                                           simulationTestHelper.getControllerRegistry());

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];
            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               assertEquals(expectedPointsInQueue, findNumberOfQueuedPoints(bodyName, armJoint, simulationTestHelper.getControllerRegistry()));
         }

         success = simulationTestHelper.simulateAndWait((expectedNumberOfPointsInGenerator - 1) * timePerWaypoint);
         assertTrue(success);

         totalNumberOfPoints = totalNumberOfPoints - (expectedNumberOfPointsInGenerator - 1);
         firstSegment = false;

         if (expectedPointsInQueue == 0)
            break;
      }

      success = simulationTestHelper.simulateAndWait(1.0);

      double[] desiredJointPositions = new double[armJoints.length];
      double[] desiredJointVelocities = new double[armJoints.length];

      for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         desiredJointPositions[jointIndex] = armTrajectoryMessages.get(numberOfMessages - 1).getJointspaceTrajectory().getJointTrajectoryMessages()
                                                                  .get(jointIndex).getTrajectoryPoints().getLast().getPosition();

      EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(armJointNames,
                                                                    desiredJointPositions,
                                                                    desiredJointVelocities,
                                                                    epsilon,
                                                                    simulationTestHelper.getControllerRegistry());
      simulationTestHelper.createVideo(getSimpleRobotName(), 2);

      assertEquals(2 * armTrajectoryMessages.size(), statusMessages.size());

      double startTime = 0.0;

      for (int inputIndex = 0; inputIndex < armTrajectoryMessages.size(); inputIndex++)
      {
         ArmTrajectoryMessage armTrajectoryMessage = armTrajectoryMessages.get(inputIndex);
         double[] finalDesiredPositions = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().stream()
                                                              .mapToDouble(jointTrajectory -> jointTrajectory.getTrajectoryPoints().getLast().getPosition())
                                                              .toArray();
         Object<TrajectoryPoint1DMessage> firstJointTrajectoryPoints = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().get(0)
                                                                                           .getTrajectoryPoints();
         double endTime = startTime + firstJointTrajectoryPoints.getLast().getTime();
         if (inputIndex > 0)
            startTime += firstJointTrajectoryPoints.getFirst().getTime();
         JointspaceTrajectoryStatusMessage startedStatusMessage = statusMessages.remove(0);
         JointspaceTrajectoryStatusMessage completedStatusMessage = statusMessages.remove(0);
         EndToEndTestTools.assertJointspaceTrajectoryStatus(armTrajectoryMessage.getSequenceId(),
                                                            TrajectoryExecutionStatus.STARTED,
                                                            startTime,
                                                            armJointNames,
                                                            startedStatusMessage,
                                                            controllerDT);
         EndToEndTestTools.assertJointspaceTrajectoryStatus(armTrajectoryMessage.getSequenceId(),
                                                            TrajectoryExecutionStatus.COMPLETED,
                                                            endTime,
                                                            finalDesiredPositions,
                                                            armJointNames,
                                                            completedStatusMessage,
                                                            1.0e-3,
                                                            controllerDT);
         startTime = endTime;
      }
   }

   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      createSimulationTestHelper();
      simulationTestHelper.start();

      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = getTimePerWaypoint();
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;

      SideDependentList<OneDoFJointBasics[]> armsJoints = new SideDependentList<>();
      SideDependentList<List<ArmTrajectoryMessage>> armTrajectoryMessages = new SideDependentList<>();

      long id = 1264L;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);

         List<ArmTrajectoryMessage> messageList = new ArrayList<>();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
            armTrajectoryMessage.setSequenceId(random.nextLong());
            armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
            if (messageIndex > 0)
            {
               long previousMessageId = id - 1;
               if (messageIndex == numberOfMessages - 1)
                  previousMessageId = id + 100; // Bad ID

               armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
               armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setPreviousMessageId(previousMessageId);
            }
            id++;

            OneDoFTrajectoryPointCalculator trajectoryPoint1DCalculator = new OneDoFTrajectoryPointCalculator();

            for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
            {
               OneDoFJointBasics joint = armJoints[jointIndex];
               OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();

               trajectoryPoint1DCalculator.clear();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
                  trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
               }

               trajectoryPoint1DCalculator.compute(timePerWaypoint * (numberOfTrajectoryPoints - 1));
               OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
               trajectoryData.addTimeOffset(getTimePerWaypoint());

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
               }
            }
            messageList.add(armTrajectoryMessage);
         }
         armTrajectoryMessages.put(robotSide, messageList);
      }

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         for (RobotSide robotSide : RobotSide.values)
            simulationTestHelper.publishToController(armTrajectoryMessages.get(robotSide).get(messageIndex));

         success = simulationTestHelper.simulateAndWait(controllerDT);

         assertTrue(success);
      }

      success = simulationTestHelper.simulateAndWait(controllerDT);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];

            String jointName = armJoints[jointIndex].getName();
            String namespace = jointName + "PDController";
            assertEquals(1,
                         EndToEndTestTools.findTotalNumberOfWaypointsInJointspaceManager(fullRobotModel.getHand(robotSide).getName(),
                                                                                         armJoint.getName(),
                                                                                         simulationTestHelper.getControllerRegistry()));
            desiredJointPositions[jointIndex] = EndToEndTestTools.findYoDouble(namespace, "q_" + jointName, simulationTestHelper.getControllerRegistry())
                                                                 .getValue();
         }

         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(1,
                                                                           fullRobotModel.getHand(robotSide).getName(),
                                                                           armJointNames,
                                                                           simulationTestHelper.getControllerRegistry());
         EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(armJointNames,
                                                                       desiredJointPositions,
                                                                       desiredJointVelocities,
                                                                       0.01,
                                                                       simulationTestHelper.getControllerRegistry());
      }
   }

   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      createSimulationTestHelper();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = getTimePerWaypoint();
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;

      SideDependentList<OneDoFJointBasics[]> armsJoints = new SideDependentList<>();

      long id = 1264L;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
            armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
            if (messageIndex > 0)
            {
               armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
               armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setPreviousMessageId(id - 1);
            }
            id++;

            OneDoFTrajectoryPointCalculator trajectoryPoint1DCalculator = new OneDoFTrajectoryPointCalculator();

            for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
            {
               OneDoFJointBasics joint = armJoints[jointIndex];
               OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();

               trajectoryPoint1DCalculator.clear();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
                  trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
               }

               trajectoryPoint1DCalculator.compute(timePerWaypoint * (numberOfTrajectoryPoints - 1));
               OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
               trajectoryData.addTimeOffset(getTimePerWaypoint());

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
               }
            }
            simulationTestHelper.publishToController(armTrajectoryMessage);

            success = simulationTestHelper.simulateAndWait(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = simulationTestHelper.simulateAndWait(0.1);
      assertTrue(success);

      SideDependentList<ArmTrajectoryMessage> overridingMessages = new SideDependentList<>();

      double overrideTrajectoryTime = 2.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide,
                                                                                                     overrideTrajectoryTime,
                                                                                                     generateRandomJointPositions(random,
                                                                                                                                  armsJoints.get(robotSide)));
         simulationTestHelper.publishToController(armTrajectoryMessage);
         overridingMessages.put(robotSide, armTrajectoryMessage);
      }

      success = simulationTestHelper.simulateAndWait(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);
         ArmTrajectoryMessage overridingMessage = overridingMessages.get(robotSide);

         EndToEndTestTools.assertTotalNumberOfWaypointsInJointspaceManager(2,
                                                                           fullRobotModel.getHand(robotSide).getName(),
                                                                           armJointNames,
                                                                           simulationTestHelper.getControllerRegistry());

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];

            TrajectoryPoint1DMessage expectedTrajectoryPoint = overridingMessage.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex)
                                                                                .getTrajectoryPoints().get(0);
            OneDoFTrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(armJoint, 1, simulationTestHelper.getControllerRegistry());
            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), epsilon);
            assertEquals(expectedTrajectoryPoint.getPosition(), controllerTrajectoryPoint.getPosition(), epsilon);
            assertEquals(expectedTrajectoryPoint.getVelocity(), controllerTrajectoryPoint.getVelocity(), epsilon);
            assertEquals(0, findNumberOfQueuedPoints(fullRobotModel.getHand(robotSide).getName(), armJoint, simulationTestHelper.getControllerRegistry()));
         }
      }

      success = simulationTestHelper.simulateAndWait(overrideTrajectoryTime);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         String[] armJointNames = Stream.of(armJoints).map(JointReadOnly::getName).toArray(String[]::new);

         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
            desiredJointPositions[jointIndex] = overridingMessages.get(robotSide).getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex)
                                                                  .getTrajectoryPoints().getLast().getPosition();

         EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(armJointNames,
                                                                       desiredJointPositions,
                                                                       desiredJointVelocities,
                                                                       epsilon,
                                                                       simulationTestHelper.getControllerRegistry());
      }
   }

   @Test
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      createSimulationTestHelper();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 1.0;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = armJoints.length;
         double[] desiredJointPositions = new double[numberOfJoints];

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
         simulationTestHelper.publishToController(armTrajectoryMessage);

         success = simulationTestHelper.simulateAndWait(trajectoryTime / 2.0);
         assertTrue(success);

         // publish the message and simulate for one control dt
         simulationTestHelper.publishToController(new StopAllTrajectoryMessage());
         int simulationTicks = (int) (getRobotModel().getControllerDT() / getRobotModel().getSimulateDT());
         simulationTestHelper.simulateAndWait(simulationTicks);

         double[] expectedControllerDesiredJointVelocities = new double[numberOfJoints];
         double[] expectedControllerDesiredJointPositions = findControllerDesiredPositions(armJoints, simulationTestHelper.getControllerRegistry());

         success = simulationTestHelper.simulateAndWait(0.1);
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndTestTools.findRigidBodyControlManagerState(hand.getName(),
                                                                                                   simulationTestHelper.getControllerRegistry());
         double[] actualControllerDesiredJointPositions = findControllerDesiredPositions(armJoints, simulationTestHelper.getControllerRegistry());
         double[] controllerDesiredJointVelocities = findControllerDesiredVelocities(armJoints, simulationTestHelper.getControllerRegistry());

         assertEquals(RigidBodyControlMode.JOINTSPACE, controllerState);
         assertArrayEquals(expectedControllerDesiredJointPositions, actualControllerDesiredJointPositions, 1.0e-10);
         assertArrayEquals(expectedControllerDesiredJointVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(54651);

      YoRegistry testRegistry = new YoRegistry("testStreaming");

      createSimulationTestHelper();
      simulationTestHelper.start();
      simulationTestHelper.getSimulationSession().getRootRegistry().addChild(testRegistry);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = simulationTestHelper.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);
      SideDependentList<YoDouble[]> initialArmJointAngles = new SideDependentList<>();
      SideDependentList<YoDouble[]> finalArmJointAngles = new SideDependentList<>();
      SideDependentList<YoDouble[]> desiredArmJointAngles = new SideDependentList<>();
      SideDependentList<YoDouble[]> desiredArmJointVelocities = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         initialArmJointAngles.put(robotSide, new YoDouble[armJoints.length]);
         finalArmJointAngles.put(robotSide, new YoDouble[armJoints.length]);
         desiredArmJointAngles.put(robotSide, new YoDouble[armJoints.length]);
         desiredArmJointVelocities.put(robotSide, new YoDouble[armJoints.length]);

         for (int i = 0; i < armJoints.length; i++)
         {
            OneDoFJointBasics armJoint = armJoints[i];
            YoDouble qInitial = new YoDouble("test_q_initial_" + armJoint.getName(), testRegistry);
            YoDouble qFinal = new YoDouble("test_q_final_" + armJoint.getName(), testRegistry);
            YoDouble qDesired = new YoDouble("test_q_des_" + armJoint.getName(), testRegistry);
            YoDouble qDDesired = new YoDouble("test_qd_des_" + armJoint.getName(), testRegistry);
            qInitial.set(armJoint.getQ());
            qFinal.set(RandomNumbers.nextDouble(random, armJoint.getJointLimitLower(), armJoint.getJointLimitUpper()));
            initialArmJointAngles.get(robotSide)[i] = qInitial;
            finalArmJointAngles.get(robotSide)[i] = qFinal;
            desiredArmJointAngles.get(robotSide)[i] = qDesired;
            desiredArmJointVelocities.get(robotSide)[i] = qDDesired;
         }
      }

      simulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         private boolean everyOtherTick = false;

         @Override
         public void doControl()
         {
            everyOtherTick = !everyOtherTick;

            if (!everyOtherTick)
               return;

            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            for (RobotSide robotSide : RobotSide.values)
            {
               YoDouble[] yoQInitials = initialArmJointAngles.get(robotSide);
               YoDouble[] yoQFinals = finalArmJointAngles.get(robotSide);
               YoDouble[] yoQDesireds = desiredArmJointAngles.get(robotSide);
               YoDouble[] yoQDDesireds = desiredArmJointVelocities.get(robotSide);
               double[] qDesireds = new double[yoQInitials.length];
               double[] qDDesireds = new double[yoQInitials.length];

               for (int i = 0; i < yoQInitials.length; i++)
               {
                  double qDes = EuclidCoreTools.interpolate(yoQInitials[i].getValue(), yoQFinals[i].getValue(), alpha);
                  double qDDes;
                  if (alpha <= 0.0 || alpha >= 1.0)
                     qDDes = 0.0;
                  else
                     qDDes = (yoQFinals[i].getValue() - yoQInitials[i].getValue()) / trajectoryTime.getValue();
                  yoQDesireds[i].set(qDes);
                  yoQDDesireds[i].set(qDDes);
                  qDesireds[i] = qDes;
                  qDDesireds[i] = qDDes;
               }

               ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, 0.0, qDesireds, qDDesireds, null);
               message.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
               message.getJointspaceTrajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
               simulationTestHelper.publishToController(message);
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return null;
         }

         @Override
         public String getDescription()
         {
            return RobotController.super.getDescription();
         }

         @Override
         public String getName()
         {
            return RobotController.super.getName();
         }
      });

      success = simulationTestHelper.simulateAndWait(0.5 * trajectoryTime.getValue());
      assertTrue(success);

      double desiredEpsilon = 6.0e-3;
      double trackingEpsilon = 0.1;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         double[] controllerDesiredPositions = findControllerDesiredPositions(armJoints, simulationTestHelper.getControllerRegistry());
         double[] controllerDesiredVelocities = findControllerDesiredVelocities(armJoints, simulationTestHelper.getControllerRegistry());

         for (int i = 0; i < armJoints.length; i++)
         {
            double qDDes = desiredArmJointVelocities.get(robotSide)[i].getValue();
            double qDes = desiredArmJointAngles.get(robotSide)[i].getValue() - getRobotModel().getControllerDT() * qDDes; // Hack to approx the previous desired. The last computed desired has not been processed yet.

            assertEquals(qDes,
                         controllerDesiredPositions[i],
                         desiredEpsilon,
                         "Desired position mismatch for joint " + armJoints[i].getName() + " diff: " + Math.abs(qDes - controllerDesiredPositions[i]));
            assertEquals(qDDes,
                         controllerDesiredVelocities[i],
                         desiredEpsilon,
                         "Desired velocity mismatch for joint " + armJoints[i].getName() + " diff: " + Math.abs(qDDes - controllerDesiredVelocities[i]));
            assertEquals(controllerDesiredPositions[i],
                         armJoints[i].getQ(),
                         trackingEpsilon,
                         "Poor position tracking for joint " + armJoints[i].getName() + " err: "
                               + Math.abs(controllerDesiredPositions[i] - armJoints[i].getQ()));
            assertEquals(controllerDesiredVelocities[i],
                         armJoints[i].getQd(),
                         2.0 * trackingEpsilon,
                         "Poor velocity tracking for joint " + armJoints[i].getName() + " err: "
                               + Math.abs(controllerDesiredVelocities[i] - armJoints[i].getQd()));
         }
      }

      success = simulationTestHelper.simulateAndWait(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);

      desiredEpsilon = 1.0e-7;
      trackingEpsilon = 0.1;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         double[] controllerDesiredPositions = findControllerDesiredPositions(armJoints, simulationTestHelper.getControllerRegistry());
         double[] controllerDesiredVelocities = findControllerDesiredVelocities(armJoints, simulationTestHelper.getControllerRegistry());

         for (int i = 0; i < armJoints.length; i++)
         {
            double qDes = desiredArmJointAngles.get(robotSide)[i].getValue();
            double qDDes = desiredArmJointVelocities.get(robotSide)[i].getValue();

            assertEquals(qDes,
                         controllerDesiredPositions[i],
                         desiredEpsilon,
                         "Desired position mismatch for joint " + armJoints[i].getName() + " diff: " + Math.abs(qDes - controllerDesiredPositions[i]));
            assertEquals(qDDes,
                         controllerDesiredVelocities[i],
                         desiredEpsilon,
                         "Desired velocity mismatch for joint " + armJoints[i].getName() + " diff: " + Math.abs(qDDes - controllerDesiredVelocities[i]));
            assertEquals(controllerDesiredPositions[i],
                         armJoints[i].getQ(),
                         trackingEpsilon,
                         "Poor position tracking for joint " + armJoints[i].getName() + " err: "
                               + Math.abs(controllerDesiredPositions[i] - armJoints[i].getQ()));
            assertEquals(controllerDesiredVelocities[i],
                         armJoints[i].getQd(),
                         3.0 * trackingEpsilon,
                         "Poor velocity tracking for joint " + armJoints[i].getName() + " err: "
                               + Math.abs(controllerDesiredVelocities[i] - armJoints[i].getQd()));
         }
      }
   }

   public static double[] findControllerDesiredPositions(OneDoFJointBasics[] armJoints, YoVariableHolder scs)
   {
      double[] controllerDesiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         String jointName = armJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentPosition";
         YoDouble q_d = (YoDouble) scs.findVariable(subTrajectoryName, variableName);
         controllerDesiredJointPositions[i] = q_d.getDoubleValue();
      }
      return controllerDesiredJointPositions;
   }

   public static double[] findControllerDesiredVelocities(OneDoFJointBasics[] armJoints, YoVariableHolder scs)
   {
      double[] controllerDesiredJointVelocities = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         String jointName = armJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentVelocity";
         YoDouble qd_d = (YoDouble) scs.findVariable(subTrajectoryName, variableName);
         controllerDesiredJointVelocities[i] = qd_d.getDoubleValue();
      }
      return controllerDesiredJointVelocities;
   }

   public static OneDoFTrajectoryPoint findTrajectoryPoint(OneDoFJointBasics armJoint, int trajectoryPointIndex, YoVariableHolder scs)
   {
      String jointName = armJoint.getName();
      String trajectoryName = jointName + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
      String timeName = jointName + "TimeAtWaypoint" + trajectoryPointIndex;
      String positionName = jointName + "PositionAtWaypoint" + trajectoryPointIndex;
      String velocityName = jointName + "VelocityAtWaypoint" + trajectoryPointIndex;

      double time = ((YoDouble) scs.findVariable(trajectoryName, timeName)).getDoubleValue();
      double position = ((YoDouble) scs.findVariable(trajectoryName, positionName)).getDoubleValue();
      double velocity = ((YoDouble) scs.findVariable(trajectoryName, velocityName)).getDoubleValue();

      OneDoFTrajectoryPoint trajectoryPoint = new OneDoFTrajectoryPoint();
      trajectoryPoint.set(time, position, velocity);
      return trajectoryPoint;
   }

   public static OneDoFTrajectoryPoint findLastTrajectoryPoint(String bodyName, OneDoFJointBasics armJoint, YoVariableHolder scs)
   {
      int lastTrajectoryPointIndex = EndToEndTestTools.findTotalNumberOfWaypointsInJointspaceManager(bodyName, armJoint.getName(), scs) - 1;
      return findTrajectoryPoint(armJoint, lastTrajectoryPointIndex, scs);
   }

   public static int findNumberOfQueuedPoints(String bodyName, OneDoFJointBasics armJoint, YoVariableHolder scs)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + armJoint.getName() + "_numberOfPointsInQueue";
      return ((YoInteger) scs.findVariable(namespace, variable)).getIntegerValue();
   }

   private ArmTrajectoryMessage generateRandomArmTrajectoryMessage(Random random,
                                                                   int numberOfTrajectoryPoints,
                                                                   double trajectoryTime,
                                                                   RobotSide robotSide,
                                                                   OneDoFJointBasics[] armJoints)
   {
      int numberOfJoints = armJoints.length;

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
      OneDoFTrajectoryPointCalculator trajectoryPoint1DCalculator = new OneDoFTrajectoryPointCalculator();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = armJoints[jointIndex];
         OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();

         trajectoryPoint1DCalculator.clear();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
            trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
         }

         trajectoryPoint1DCalculator.compute(trajectoryTime);
         OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
         trajectoryData.addTimeOffset(getTimePerWaypoint());

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
            jointTrajectoryMessage.getTrajectoryPoints().add()
                                  .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint.getTime(),
                                                                                           trajectoryPoint.getPosition(),
                                                                                           trajectoryPoint.getVelocity()));
         }
      }
      return armTrajectoryMessage;
   }

   private static double[] generateRandomJointPositions(Random random, OneDoFJointBasics[] armJoints)
   {
      double[] desiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         OneDoFJointBasics joint = armJoints[i];
         desiredJointPositions[i] = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
      }
      return desiredJointPositions;
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
