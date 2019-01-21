package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointControlHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.generators.OneDoFTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class EndToEndArmTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 27.8)
   @Test(timeout = 140000)
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 0.5;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);
         double[] desiredJointPositions = generateRandomJointPositions(random, armJoints);
         double[] desiredJointVelocities = new double[numberOfJoints];

         generateRandomJointPositions(random, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);

         if (DEBUG)
         {
            for (int i = 0; i < numberOfJoints; i++)
            {
               OneDoFJointBasics armJoint = armJoints[i];
               System.out.println(armJoint.getName() + ": q = " + armJoint.getQ());
            }
         }

         drcSimulationTestHelper.publishToController(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      }
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 36.0)
   @Test(timeout = 180000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
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

         armTrajectoryMessages.put(robotSide, armTrajectoryMessage);
         drcSimulationTestHelper.publishToController(armTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2 * getRobotModel().getControllerDT()); // Not sure why, but the controller needs 2*dt to initialize.
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         ArmTrajectoryMessage armTrajectoryMessage = armTrajectoryMessages.get(robotSide);

         assertNumberOfWaypoints(fullRobotModel.getHand(robotSide).getName(), armJoints, numberOfTrajectoryPoints + 1, scs);

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyJointspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
            {
               TrajectoryPoint1DMessage expectedTrajectoryPoint = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex).getTrajectoryPoints().get(trajectoryPointIndex);
               OneDoFTrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(armJoint, trajectoryPointIndex + 1, scs);
               assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), epsilon);
               assertEquals(expectedTrajectoryPoint.getPosition(), controllerTrajectoryPoint.getPosition(), epsilon);
               assertEquals(expectedTrajectoryPoint.getVelocity(), controllerTrajectoryPoint.getVelocity(), epsilon);
            }
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 + trajectoryTime);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);

         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
            desiredJointPositions[jointIndex] = armTrajectoryMessages.get(robotSide).getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex).getTrajectoryPoints().getLast().getPosition();

         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 16.5)
   @Test(timeout = 83000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
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
               message.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIdx).getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(time, 0.0, 0.0));
            }
            time = time + 0.05;
         }
         drcSimulationTestHelper.publishToController(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(bodyName, armJoints, 1, drcSimulationTestHelper.getSimulationConstructionSet());
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
                                                                                                  .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(time,
                                                                                                                                                           0.0,
                                                                                                                                                           0.0));
            }
            time = time + 0.05;
         }
         drcSimulationTestHelper.publishToController(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(bodyName, armJoints, RigidBodyJointspaceControlState.maxPoints, drcSimulationTestHelper.getSimulationConstructionSet());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.9)
   @Test(timeout = 300000)
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;

      OneDoFJointBasics[] armJoints;
      List<ArmTrajectoryMessage> armTrajectoryMessages;

      long id = 1264L;
      RobotSide robotSide = RobotSide.LEFT;

      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
      int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);

      armTrajectoryMessages = new ArrayList<>();

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         ArmTrajectoryMessage trajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
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

            double timeAtWaypoint = timePerWaypoint;
            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
               trajectoryPoint1DCalculator.appendTrajectoryPoint(timeAtWaypoint, desiredJointPosition);
               timeAtWaypoint += timePerWaypoint;
            }

            trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
            OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
               jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
            }
         }

         armTrajectoryMessages.add(trajectoryMessage);
         drcSimulationTestHelper.publishToController(trajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      int totalNumberOfPoints = numberOfTrajectoryPoints * numberOfMessages + 1;
      boolean firstSegment = true;

      while (true)
      {
         int expectedNumberOfPointsInGenerator = Math.min(totalNumberOfPoints, RigidBodyJointspaceControlState.maxPointsInGenerator);
         if (firstSegment)
            expectedNumberOfPointsInGenerator = Math.min(RigidBodyJointspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);
         int expectedPointsInQueue = totalNumberOfPoints - expectedNumberOfPointsInGenerator;

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(bodyName, armJoints, totalNumberOfPoints, scs);

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];
            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               assertEquals(expectedPointsInQueue, findNumberOfQueuedPoints(bodyName, armJoint, scs));
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions((expectedNumberOfPointsInGenerator - 1) * timePerWaypoint);
         assertTrue(success);

         totalNumberOfPoints = totalNumberOfPoints - (expectedNumberOfPointsInGenerator - 1);
         firstSegment = false;

         if (expectedPointsInQueue == 0)
            break;
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double[] desiredJointPositions = new double[armJoints.length];
      double[] desiredJointVelocities = new double[armJoints.length];

      for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         desiredJointPositions[jointIndex] = armTrajectoryMessages.get(numberOfMessages - 1).getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex).getTrajectoryPoints().getLast().getPosition();

      assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 16.9)
   @Test(timeout = 85000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;
      double trajectoryTime = (numberOfTrajectoryPoints + 1) * timePerWaypoint;

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

               trajectoryPoint1DCalculator.computeTrajectoryPointTimes(timePerWaypoint, trajectoryTime);
               trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
               OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

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
            drcSimulationTestHelper.publishToController(armTrajectoryMessages.get(robotSide).get(messageIndex));

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());

         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];
            assertEquals(1, findNumberOfTrajectoryPoints(fullRobotModel.getHand(robotSide).getName(), armJoint, scs));
            desiredJointPositions[jointIndex] = armJoints[jointIndex].getQ();
         }

         assertNumberOfWaypoints(fullRobotModel.getHand(robotSide).getName(), armJoints, 1, scs);
         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, 0.01, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 19.0)
   @Test(timeout = 95000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;
      double trajectoryTime = (numberOfTrajectoryPoints + 1) * timePerWaypoint;

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

               trajectoryPoint1DCalculator.computeTrajectoryPointTimes(timePerWaypoint, trajectoryTime);
               trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
               OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  jointTrajectoryMessage.getTrajectoryPoints().add().set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint));
               }
            }
            drcSimulationTestHelper.publishToController(armTrajectoryMessage);

            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);

      SideDependentList<ArmTrajectoryMessage> overridingMessages = new SideDependentList<>();

      double overrideTrajectoryTime = 0.5;

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, overrideTrajectoryTime, generateRandomJointPositions(random, armsJoints.get(robotSide)));
         drcSimulationTestHelper.publishToController(armTrajectoryMessage);
         overridingMessages.put(robotSide, armTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);
         ArmTrajectoryMessage overridingMessage = overridingMessages.get(robotSide);

         assertNumberOfWaypoints(fullRobotModel.getHand(robotSide).getName(), armJoints, 2, scs);

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJointBasics armJoint = armJoints[jointIndex];

            TrajectoryPoint1DMessage expectedTrajectoryPoint = overridingMessage.getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex).getTrajectoryPoints().get(0);
            OneDoFTrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(armJoint, 1, scs);
            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), epsilon);
            assertEquals(expectedTrajectoryPoint.getPosition(), controllerTrajectoryPoint.getPosition(), epsilon);
            assertEquals(expectedTrajectoryPoint.getVelocity(), controllerTrajectoryPoint.getVelocity(), epsilon);
            assertEquals(0, findNumberOfQueuedPoints(fullRobotModel.getHand(robotSide).getName(), armJoint, scs));
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(overrideTrajectoryTime);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJointBasics[] armJoints = armsJoints.get(robotSide);

         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
            desiredJointPositions[jointIndex] = overridingMessages.get(robotSide).getJointspaceTrajectory().getJointTrajectoryMessages().get(jointIndex).getTrajectoryPoints().getLast().getPosition();

         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 34.7)
   @Test(timeout = 170000)
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 1.0;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = armJoints.length;
         double[] desiredJointPositions = new double[numberOfJoints];

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
         drcSimulationTestHelper.publishToController(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         // publish the message and simulate for one control dt
         drcSimulationTestHelper.publishToController(new StopAllTrajectoryMessage());
         int simulationTicks = (int) (getRobotModel().getControllerDT() / scs.getDT());
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTicks);

         double[] expectedControllerDesiredJointVelocities = new double[numberOfJoints];
         double[] expectedControllerDesiredJointPositions = findControllerDesiredPositions(armJoints, scs);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);

         RigidBodyControlMode controllerState = findControllerState(hand.getName(), scs);
         double[] actualControllerDesiredJointPositions = findControllerDesiredPositions(armJoints, scs);
         double[] controllerDesiredJointVelocities = findControllerDesiredVelocities(armJoints, scs);

         assertEquals(RigidBodyControlMode.JOINTSPACE, controllerState);
         assertArrayEquals(expectedControllerDesiredJointPositions, actualControllerDesiredJointPositions, 1.0e-10);
         assertArrayEquals(expectedControllerDesiredJointVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public static void assertNumberOfWaypoints(String bodyName, OneDoFJointBasics[] armJoints, int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs)
   {
      for (int i = 0; i < armJoints.length; i++)
      {
         int controllerNumberOfTrajectoryPoints = findNumberOfTrajectoryPoints(bodyName, armJoints[i], scs);
         assertTrue(
               "Unexpected number of trajectory points: expected = " + expectedNumberOfTrajectoryPoints + ", controller = " + controllerNumberOfTrajectoryPoints,
               controllerNumberOfTrajectoryPoints == expectedNumberOfTrajectoryPoints);
      }
   }

   public static void assertSingleWaypointExecuted(OneDoFJointBasics[] armJoints, double[] desiredJointPositions, double[] desiredJointVelocities, double epsilon,
         SimulationConstructionSet scs)
   {
      for (int jointIdx = 0; jointIdx < armJoints.length; jointIdx++)
      {
         assertJointDesired(scs, armJoints[jointIdx], desiredJointPositions[jointIdx], desiredJointVelocities[jointIdx], epsilon);
      }
   }

   private static void assertJointDesired(SimulationConstructionSet scs, OneDoFJointBasics joint, double desiredPosition, double desiredVelocity, double epsilon)
   {
      YoDouble scsDesiredPosition = findJointDesiredPosition(scs, joint);
      assertEquals(desiredPosition, scsDesiredPosition.getDoubleValue(), epsilon);
      YoDouble scsDesiredVelocity = findJointDesiredVelocity(scs, joint);
      assertEquals(desiredVelocity, scsDesiredVelocity.getDoubleValue(), epsilon);
   }

   private static YoDouble findJointDesiredPosition(SimulationConstructionSet scs, OneDoFJointBasics joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "q_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace);
   }

   private static YoDouble findJointDesiredVelocity(SimulationConstructionSet scs, OneDoFJointBasics joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "qd_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace);
   }

   private static YoDouble getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoDouble.class);
   }

   private static <T extends YoVariable<T>> T getYoVariable(SimulationConstructionSet scs, String name, String namespace, Class<T> clazz)
   {
      YoVariable<?> uncheckedVariable = scs.getVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
   }

   @SuppressWarnings("unchecked")
   public static RigidBodyControlMode findControllerState(String bodyName, SimulationConstructionSet scs)
   {
      String managerName = bodyName + "Manager";
      return ((YoEnum<RigidBodyControlMode>) scs.getVariable(managerName, managerName + "CurrentState")).getEnumValue();
   }

   public static double[] findControllerDesiredPositions(OneDoFJointBasics[] armJoints, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         String jointName = armJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentValue";
         YoDouble q_d = (YoDouble) scs.getVariable(subTrajectoryName, variableName);
         controllerDesiredJointPositions[i] = q_d.getDoubleValue();
      }
      return controllerDesiredJointPositions;
   }

   public static double[] findControllerDesiredVelocities(OneDoFJointBasics[] armJoints, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointVelocities = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         String jointName = armJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentVelocity";
         YoDouble qd_d = (YoDouble) scs.getVariable(subTrajectoryName, variableName);
         controllerDesiredJointVelocities[i] = qd_d.getDoubleValue();
      }
      return controllerDesiredJointVelocities;
   }

   public static int findNumberOfTrajectoryPoints(String bodyName, OneDoFJointBasics armJoint, SimulationConstructionSet scs)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + armJoint.getName() + "_numberOfPoints";
      return ((YoInteger) scs.getVariable(namespace, variable)).getIntegerValue();
   }

   public static OneDoFTrajectoryPoint findTrajectoryPoint(OneDoFJointBasics armJoint, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String jointName = armJoint.getName();
      String trajectoryName = jointName + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
      String timeName = jointName + "TimeAtWaypoint" + trajectoryPointIndex;
      String positionName = jointName + "PositionAtWaypoint" + trajectoryPointIndex;
      String velocityName = jointName + "VelocityAtWaypoint" + trajectoryPointIndex;

      double time = ((YoDouble) scs.getVariable(trajectoryName, timeName)).getDoubleValue();
      double position = ((YoDouble) scs.getVariable(trajectoryName, positionName)).getDoubleValue();
      double velocity = ((YoDouble) scs.getVariable(trajectoryName, velocityName)).getDoubleValue();

      OneDoFTrajectoryPoint trajectoryPoint = new OneDoFTrajectoryPoint();
      trajectoryPoint.set(time, position, velocity);
      return trajectoryPoint;
   }

   public static OneDoFTrajectoryPoint findLastTrajectoryPoint(String bodyName, OneDoFJointBasics armJoint, SimulationConstructionSet scs)
   {
      int lastTrajectoryPointIndex = findNumberOfTrajectoryPoints(bodyName, armJoint, scs) - 1;
      return findTrajectoryPoint(armJoint, lastTrajectoryPointIndex, scs);
   }

   public static int findNumberOfQueuedPoints(String bodyName, OneDoFJointBasics armJoint, SimulationConstructionSet scs)
   {
      String namespace = bodyName + RigidBodyJointControlHelper.shortName;
      String variable = bodyName + "Jointspace_" + armJoint.getName() + "_numberOfPointsInQueue";
      return ((YoInteger) scs.getVariable(namespace, variable)).getIntegerValue();
   }

   private ArmTrajectoryMessage generateRandomArmTrajectoryMessage(Random random, int numberOfTrajectoryPoints, double trajectoryTime, RobotSide robotSide,
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

         trajectoryPoint1DCalculator.computeTrajectoryPointTimes(0.5, trajectoryTime + 0.5);
         trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
         OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

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

   private double[] generateRandomJointPositions(Random random, OneDoFJointBasics[] armJoints)
   {
      double[] desiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         OneDoFJointBasics joint = armJoints[i];
         desiredJointPositions[i] = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
      }
      return desiredJointPositions;
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
