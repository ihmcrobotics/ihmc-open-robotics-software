package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyJointspaceControlState;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPoint1DCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndArmTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 60000)
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 0.5;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);
         double[] desiredJointPositions = generateRandomJointPositions(random, armJoints);
         double[] desiredJointVelocities = new double[numberOfJoints];

         generateRandomJointPositions(random, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);

         if (DEBUG)
         {
            for (int i = 0; i < numberOfJoints; i++)
            {
               OneDoFJoint armJoint = armJoints[i];
               System.out.println(armJoint.getName() + ": q = " + armJoint.getQ());
            }
         }

         drcSimulationTestHelper.send(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 70000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfTrajectoryPoints = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

      SideDependentList<OneDoFJoint[]> armsJoints = new SideDependentList<>();
      SideDependentList<ArmTrajectoryMessage> armTrajectoryMessages = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = generateRandomArmTrajectoryMessage(random, numberOfTrajectoryPoints, trajectoryTime, robotSide, armJoints);

         armTrajectoryMessages.put(robotSide, armTrajectoryMessage);
         drcSimulationTestHelper.send(armTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2 * getRobotModel().getControllerDT()); // Not sure why, but the controller needs 2*dt to initialize.
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint[] armJoints = armsJoints.get(robotSide);
         ArmTrajectoryMessage armTrajectoryMessage = armTrajectoryMessages.get(robotSide);

         assertNumberOfWaypoints(fullRobotModel.getHand(robotSide).getName(), armJoints, numberOfTrajectoryPoints + 1, scs);

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJoint armJoint = armJoints[jointIndex];

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyJointspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
            {
               TrajectoryPoint1DMessage expectedTrajectoryPoint = armTrajectoryMessage.getJointTrajectoryPoint(jointIndex, trajectoryPointIndex);
               SimpleTrajectoryPoint1D controllerTrajectoryPoint = findTrajectoryPoint(armJoint, trajectoryPointIndex + 1, scs);
               assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), epsilon);
               assertEquals(expectedTrajectoryPoint.getPosition(), controllerTrajectoryPoint.getPosition(), epsilon);
               assertEquals(expectedTrajectoryPoint.getVelocity(), controllerTrajectoryPoint.getVelocity(), epsilon);
            }
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint[] armJoints = armsJoints.get(robotSide);

         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
            desiredJointPositions[jointIndex] = armTrajectoryMessages.get(robotSide).getJointTrajectoryPointList(jointIndex).getLastTrajectoryPoint().getPosition();

         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);
      OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      int numberOfJoints = ScrewTools.computeDegreesOfFreedom(ScrewTools.createOneDoFJointPath(chest, hand));

      {
         int numberOfPoints = RigidBodyJointspaceControlState.maxPoints;
         ArmTrajectoryMessage message = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfPoints);
         double time = 0.05;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
               message.setTrajectoryPoint(jointIdx, pointIdx, time, 0.0, 0.0);
            time = time + 0.05;
         }
         drcSimulationTestHelper.send(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(bodyName, armJoints, 1, drcSimulationTestHelper.getSimulationConstructionSet());
      }

      {
         int numberOfPoints = RigidBodyJointspaceControlState.maxPoints - 1;
         ArmTrajectoryMessage message = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfPoints);
         double time = 0.05;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
               message.setTrajectoryPoint(jointIdx, pointIdx, time, 0.0, 0.0);
            time = time + 0.05;
         }
         drcSimulationTestHelper.send(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         String bodyName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(bodyName, armJoints, RigidBodyJointspaceControlState.maxPoints, drcSimulationTestHelper.getSimulationConstructionSet());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 65.0)
   @Test(timeout = 130000)
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;

      OneDoFJoint[] armJoints;
      List<ArmTrajectoryMessage> armTrajectoryMessages;

      long id = 1264L;
      RobotSide robotSide = RobotSide.LEFT;

      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);
      armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);

      armTrajectoryMessages = new ArrayList<>();

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         ArmTrajectoryMessage trajectoryMessage = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfTrajectoryPoints);
         trajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            trajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;

         TrajectoryPoint1DCalculator trajectoryPoint1DCalculator = new TrajectoryPoint1DCalculator();

         for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
         {
            OneDoFJoint joint = armJoints[jointIndex];
            trajectoryPoint1DCalculator.clear();

            double timeAtWaypoint = timePerWaypoint;
            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
               trajectoryPoint1DCalculator.appendTrajectoryPoint(timeAtWaypoint, desiredJointPosition);
               timeAtWaypoint += timePerWaypoint;
            }

            trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
            SimpleTrajectoryPoint1DList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
               trajectoryMessage.setTrajectoryPoint(jointIndex, trajectoryPointIndex, trajectoryPoint.getTime(), trajectoryPoint.getPosition(),
                     trajectoryPoint.getVelocity());
            }
         }

         armTrajectoryMessages.add(trajectoryMessage);
         drcSimulationTestHelper.send(trajectoryMessage);

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
            OneDoFJoint armJoint = armJoints[jointIndex];
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
         desiredJointPositions[jointIndex] = armTrajectoryMessages.get(numberOfMessages - 1).getJointTrajectoryPointList(jointIndex).getLastTrajectoryPoint().getPosition();

      assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;
      double trajectoryTime = (numberOfTrajectoryPoints + 1) * timePerWaypoint;

      SideDependentList<OneDoFJoint[]> armsJoints = new SideDependentList<>();
      SideDependentList<List<ArmTrajectoryMessage>> armTrajectoryMessages = new SideDependentList<>();

      long id = 1264L;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);
         int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);

         List<ArmTrajectoryMessage> messageList = new ArrayList<>();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfTrajectoryPoints);
            armTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
            {
               long previousMessageId = id - 1;
               if (messageIndex == numberOfMessages - 1)
                  previousMessageId = id + 100; // Bad ID

               armTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, previousMessageId);
            }
            id++;

            TrajectoryPoint1DCalculator trajectoryPoint1DCalculator = new TrajectoryPoint1DCalculator();

            for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
            {
               OneDoFJoint joint = armJoints[jointIndex];

               trajectoryPoint1DCalculator.clear();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
                  trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
               }

               trajectoryPoint1DCalculator.computeTrajectoryPointTimes(timePerWaypoint, trajectoryTime);
               trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
               SimpleTrajectoryPoint1DList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  armTrajectoryMessage.setTrajectoryPoint(jointIndex, trajectoryPointIndex, trajectoryPoint.getTime(), trajectoryPoint.getPosition(),
                        trajectoryPoint.getVelocity());
               }
            }
            messageList.add(armTrajectoryMessage);
         }
         armTrajectoryMessages.put(robotSide, messageList);
      }

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         for (RobotSide robotSide : RobotSide.values)
            drcSimulationTestHelper.send(armTrajectoryMessages.get(robotSide).get(messageIndex));

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());

         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint[] armJoints = armsJoints.get(robotSide);
         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJoint armJoint = armJoints[jointIndex];
            assertEquals(1, findNumberOfTrajectoryPoints(fullRobotModel.getHand(robotSide).getName(), armJoint, scs));
            desiredJointPositions[jointIndex] = armJoints[jointIndex].getQ();
         }

         assertNumberOfWaypoints(fullRobotModel.getHand(robotSide).getName(), armJoints, 1, scs);
         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, 0.01, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;
      double trajectoryTime = (numberOfTrajectoryPoints + 1) * timePerWaypoint;

      SideDependentList<OneDoFJoint[]> armsJoints = new SideDependentList<>();

      long id = 1264L;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);
         int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfTrajectoryPoints);
            armTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               armTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;

            TrajectoryPoint1DCalculator trajectoryPoint1DCalculator = new TrajectoryPoint1DCalculator();

            for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
            {
               OneDoFJoint joint = armJoints[jointIndex];

               trajectoryPoint1DCalculator.clear();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
                  trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
               }

               trajectoryPoint1DCalculator.computeTrajectoryPointTimes(timePerWaypoint, trajectoryTime);
               trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
               SimpleTrajectoryPoint1DList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  armTrajectoryMessage.setTrajectoryPoint(jointIndex, trajectoryPointIndex, trajectoryPoint.getTime(), trajectoryPoint.getPosition(),
                        trajectoryPoint.getVelocity());
               }
            }
            drcSimulationTestHelper.send(armTrajectoryMessage);

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
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, overrideTrajectoryTime, generateRandomJointPositions(random, armsJoints.get(robotSide)));
         drcSimulationTestHelper.send(armTrajectoryMessage);
         overridingMessages.put(robotSide, armTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint[] armJoints = armsJoints.get(robotSide);
         ArmTrajectoryMessage overridingMessage = overridingMessages.get(robotSide);

         assertNumberOfWaypoints(fullRobotModel.getHand(robotSide).getName(), armJoints, 2, scs);

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
         {
            OneDoFJoint armJoint = armJoints[jointIndex];

            TrajectoryPoint1DMessage expectedTrajectoryPoint = overridingMessage.getJointTrajectoryPoint(jointIndex, 0);
            SimpleTrajectoryPoint1D controllerTrajectoryPoint = findTrajectoryPoint(armJoint, 1, scs);
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
         OneDoFJoint[] armJoints = armsJoints.get(robotSide);

         double[] desiredJointPositions = new double[armJoints.length];
         double[] desiredJointVelocities = new double[armJoints.length];

         for (int jointIndex = 0; jointIndex < armJoints.length; jointIndex++)
            desiredJointPositions[jointIndex] = overridingMessages.get(robotSide).getJointTrajectoryPointList(jointIndex).getLastTrajectoryPoint().getPosition();

         assertSingleWaypointExecuted(armJoints, desiredJointPositions, desiredJointVelocities, epsilon, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 60000)
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 5.0;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
         int numberOfJoints = armJoints.length;
         double[] desiredJointPositions = new double[numberOfJoints];

         generateRandomJointPositions(random, armJoints);

         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);

         drcSimulationTestHelper.send(armTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
         double[] actualJointPositions = new double[numberOfJoints];
         double[] zeroVelocities = new double[numberOfJoints];
         for (int i = 0; i < numberOfJoints; i++)
         {
            actualJointPositions[i] = armJoints[i].getQ();
         }

         drcSimulationTestHelper.send(new StopAllTrajectoryMessage());

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
         assertTrue(success);

         RigidBodyControlMode controllerState = findControllerState(hand.getName(), scs);
         double[] controllerDesiredJointPositions = findControllerDesiredPositions(armJoints, scs);
         double[] controllerDesiredJointVelocities = findControllerDesiredVelocities(armJoints, scs);

         assertEquals(RigidBodyControlMode.JOINTSPACE, controllerState);
         assertArrayEquals(actualJointPositions, controllerDesiredJointPositions, 0.01);
         assertArrayEquals(zeroVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public static void assertNumberOfWaypoints(String bodyName, OneDoFJoint[] armJoints, int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs)
   {
      for (int i = 0; i < armJoints.length; i++)
      {
         int controllerNumberOfTrajectoryPoints = findNumberOfTrajectoryPoints(bodyName, armJoints[i], scs);
         assertTrue(
               "Unexpected number of trajectory points: expected = " + expectedNumberOfTrajectoryPoints + ", controller = " + controllerNumberOfTrajectoryPoints,
               controllerNumberOfTrajectoryPoints == expectedNumberOfTrajectoryPoints);
      }
   }

   public static void assertSingleWaypointExecuted(OneDoFJoint[] armJoints, double[] desiredJointPositions, double[] desiredJointVelocities, double epsilon,
         SimulationConstructionSet scs)
   {
      for (int jointIdx = 0; jointIdx < armJoints.length; jointIdx++)
      {
         assertJointDesired(scs, armJoints[jointIdx], desiredJointPositions[jointIdx], desiredJointVelocities[jointIdx], epsilon);
      }
   }

   private static void assertJointDesired(SimulationConstructionSet scs, OneDoFJoint joint, double desiredPosition, double desiredVelocity, double epsilon)
   {
      DoubleYoVariable scsDesiredPosition = findJointDesiredPosition(scs, joint);
      assertEquals(desiredPosition, scsDesiredPosition.getDoubleValue(), epsilon);
      DoubleYoVariable scsDesiredVelocity = findJointDesiredVelocity(scs, joint);
      assertEquals(desiredVelocity, scsDesiredVelocity.getDoubleValue(), epsilon);
   }

   private static DoubleYoVariable findJointDesiredPosition(SimulationConstructionSet scs, OneDoFJoint joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "q_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace);
   }

   private static DoubleYoVariable findJointDesiredVelocity(SimulationConstructionSet scs, OneDoFJoint joint)
   {
      String jointName = joint.getName();
      String namespace = jointName + "PDController";
      String variable = "qd_d_" + jointName;
      return getDoubleYoVariable(scs, variable, namespace);
   }

   private static DoubleYoVariable getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, DoubleYoVariable.class);
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
      return ((EnumYoVariable<RigidBodyControlMode>) scs.getVariable(managerName, managerName + "State")).getEnumValue();
   }

   public static double[] findControllerDesiredPositions(OneDoFJoint[] armJoints, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         String jointName = armJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentValue";
         DoubleYoVariable q_d = (DoubleYoVariable) scs.getVariable(subTrajectoryName, variableName);
         controllerDesiredJointPositions[i] = q_d.getDoubleValue();
      }
      return controllerDesiredJointPositions;
   }

   public static double[] findControllerDesiredVelocities(OneDoFJoint[] armJoints, SimulationConstructionSet scs)
   {
      double[] controllerDesiredJointVelocities = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         String jointName = armJoints[i].getName();
         String subTrajectory = "SubTrajectory";
         String subTrajectoryName = jointName + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();
         String variableName = jointName + subTrajectory + "CurrentVelocity";
         DoubleYoVariable qd_d = (DoubleYoVariable) scs.getVariable(subTrajectoryName, variableName);
         controllerDesiredJointVelocities[i] = qd_d.getDoubleValue();
      }
      return controllerDesiredJointVelocities;
   }

   public static int findNumberOfTrajectoryPoints(String bodyName, OneDoFJoint armJoint, SimulationConstructionSet scs)
   {
      String namespace = bodyName + "JointspaceControlModule";
      String variable = bodyName + "Jointspace_" + armJoint.getName() + "_numberOfPoints";
      return ((IntegerYoVariable) scs.getVariable(namespace, variable)).getIntegerValue();
   }

   public static SimpleTrajectoryPoint1D findTrajectoryPoint(OneDoFJoint armJoint, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String jointName = armJoint.getName();
      String trajectoryName = jointName + MultipleWaypointsTrajectoryGenerator.class.getSimpleName();
      String timeName = jointName + "TimeAtWaypoint" + trajectoryPointIndex;
      String positionName = jointName + "PositionAtWaypoint" + trajectoryPointIndex;
      String velocityName = jointName + "VelocityAtWaypoint" + trajectoryPointIndex;

      double time = ((DoubleYoVariable) scs.getVariable(trajectoryName, timeName)).getDoubleValue();
      double position = ((DoubleYoVariable) scs.getVariable(trajectoryName, positionName)).getDoubleValue();
      double velocity = ((DoubleYoVariable) scs.getVariable(trajectoryName, velocityName)).getDoubleValue();

      SimpleTrajectoryPoint1D trajectoryPoint = new SimpleTrajectoryPoint1D();
      trajectoryPoint.set(time, position, velocity);
      return trajectoryPoint;
   }

   public static SimpleTrajectoryPoint1D findLastTrajectoryPoint(String bodyName, OneDoFJoint armJoint, SimulationConstructionSet scs)
   {
      int lastTrajectoryPointIndex = findNumberOfTrajectoryPoints(bodyName, armJoint, scs) - 1;
      return findTrajectoryPoint(armJoint, lastTrajectoryPointIndex, scs);
   }

   public static int findNumberOfQueuedPoints(String bodyName, OneDoFJoint armJoint, SimulationConstructionSet scs)
   {
      String namespace = bodyName + "JointspaceControlModule";
      String variable = bodyName + "Jointspace_" + armJoint.getName() + "_numberOfPointsInQueue";
      return ((IntegerYoVariable) scs.getVariable(namespace, variable)).getIntegerValue();
   }

   private ArmTrajectoryMessage generateRandomArmTrajectoryMessage(Random random, int numberOfTrajectoryPoints, double trajectoryTime, RobotSide robotSide,
         OneDoFJoint[] armJoints)
   {
      int numberOfJoints = armJoints.length;

      ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage(robotSide, numberOfJoints, numberOfTrajectoryPoints);
      TrajectoryPoint1DCalculator trajectoryPoint1DCalculator = new TrajectoryPoint1DCalculator();

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJoint joint = armJoints[jointIndex];

         trajectoryPoint1DCalculator.clear();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
            trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
         }

         trajectoryPoint1DCalculator.computeTrajectoryPointTimes(0.5, trajectoryTime + 0.5);
         trajectoryPoint1DCalculator.computeTrajectoryPointVelocities(true);
         SimpleTrajectoryPoint1DList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            SimpleTrajectoryPoint1D trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
            armTrajectoryMessage.setTrajectoryPoint(jointIndex, trajectoryPointIndex, trajectoryPoint.getTime(), trajectoryPoint.getPosition(),
                  trajectoryPoint.getVelocity());
         }
      }
      return armTrajectoryMessage;
   }

   private double[] generateRandomJointPositions(Random random, OneDoFJoint[] armJoints)
   {
      double[] desiredJointPositions = new double[armJoints.length];
      for (int i = 0; i < armJoints.length; i++)
      {
         OneDoFJoint joint = armJoints[i];
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
