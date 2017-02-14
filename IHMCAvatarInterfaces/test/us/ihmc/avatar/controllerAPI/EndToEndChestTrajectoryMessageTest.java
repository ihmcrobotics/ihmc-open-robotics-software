package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;
import static us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.chest.TaskspaceChestControlState;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSO3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndChestTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 22.8)
   @Test (timeout = 110000)
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);

      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);

      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d desiredOrientation = new Quat4d();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertControlErrorIsLow(scs, chest, 1.0e-2);
      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs);
   }

   @ContinuousIntegrationTest(estimatedDuration = 19.0)
   @Test(timeout = 95000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 15;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody chest = fullRobotModel.getChest();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);

      FrameOrientation[] desiredChestOrientations = new FrameOrientation[numberOfTrajectoryPoints];
      FrameVector[] desiredChestAngularVelocities = new FrameVector[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(20.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
         double pitch = amp * Math.sin(t * w);
         double pitchDot = w * amp * Math.cos(t * w);
         desiredChestOrientations[trajectoryPointIndex] = new FrameOrientation();
         desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
         desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector();
         desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);

         Quat4d desiredOrientation = new Quat4d();
         Vector3d desiredAngularVelocity = new Vector3d();

         desiredChestOrientations[trajectoryPointIndex].getQuaternion(desiredOrientation);
         desiredChestAngularVelocities[trajectoryPointIndex].get(desiredAngularVelocity);
         chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity);
      }

      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(numberOfTrajectoryPoints + 1, scs);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         double time = chestTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex).getTime();
         SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs);
         assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         desiredChestOrientations[trajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         desiredChestAngularVelocities[trajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 25.4)
   @Test(timeout = 130000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.05;
      int numberOfTrajectoryPoints = 65;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody chest = fullRobotModel.getChest();

      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);

      FrameOrientation[] desiredChestOrientations = new FrameOrientation[numberOfTrajectoryPoints];
      FrameVector[] desiredChestAngularVelocities = new FrameVector[numberOfTrajectoryPoints];

      double t = 0.0;
      double w = 2.0 * Math.PI / trajectoryTime;
      double amp = Math.toRadians(30.0);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         t = (trajectoryPointIndex + 1) * timePerWaypoint;
         double roll = amp * Math.sin(t * w);
         double rollDot = w * amp * Math.cos(t * w);
         desiredChestOrientations[trajectoryPointIndex] = new FrameOrientation();
         desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, 0.0, roll);
         desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector();
         desiredChestAngularVelocities[trajectoryPointIndex].set(rollDot, 0.0, 0.0);

         Quat4d desiredOrientation = new Quat4d();
         Vector3d desiredAngularVelocity = new Vector3d();

         desiredChestOrientations[trajectoryPointIndex].getQuaternion(desiredOrientation);
         desiredChestAngularVelocities[trajectoryPointIndex].get(desiredAngularVelocity);
         chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity);
      }

      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timePerWaypoint + getRobotModel().getControllerDT());
      assertTrue(success);

      int expectedTrajectoryPointIndex = 0;
      boolean isDone = false;
      double previousTimeInState = timePerWaypoint;

      while(!isDone)
      {
         assertNumberOfWaypoints(Math.min(defaultMaximumNumberOfWaypoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs);

         double timeInState = 0.0;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < defaultMaximumNumberOfWaypoints - 1; trajectoryPointIndex++)
         {
            double time = chestTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex).getTime();
            SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs);
            assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            desiredChestOrientations[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
            desiredChestAngularVelocities[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

            timeInState = Math.max(time, timeInState);

            expectedTrajectoryPointIndex++;

            if (expectedTrajectoryPointIndex == numberOfTrajectoryPoints)
            {
               isDone = true;
               break;
            }
         }

         double simulationTime = timeInState - previousTimeInState;
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
         assertTrue(success);
         previousTimeInState = timeInState;
      }

      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs);
      desiredChestOrientations[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredChestAngularVelocities[numberOfTrajectoryPoints - 1].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 23.6)
   @Test(timeout = 120000)
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.02;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody chest = fullRobotModel.getChest();


      List<FrameOrientation[]> desiredChestOrientationsList = new ArrayList<>();
      List<FrameVector[]> desiredChestAngularVelocitiesList = new ArrayList<>();
      List<ChestTrajectoryMessage> messageList = new ArrayList<>();

      double t = 0.0;
      double w = 2.0 * Math.PI / (trajectoryTime * numberOfMessages);
      double amp = Math.toRadians(20.0);
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FrameOrientation[] desiredChestOrientations = new FrameOrientation[numberOfTrajectoryPoints];
         FrameVector[] desiredChestAngularVelocities = new FrameVector[numberOfTrajectoryPoints];
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
         chestTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            chestTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint;
            double pitch = amp * Math.sin((t + tOffset) * w);
            double pitchDot = w * amp * Math.cos((t + tOffset) * w);
            desiredChestOrientations[trajectoryPointIndex] = new FrameOrientation();
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector();
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);

            Quat4d desiredOrientation = new Quat4d();
            Vector3d desiredAngularVelocity = new Vector3d();

            desiredChestOrientations[trajectoryPointIndex].getQuaternion(desiredOrientation);
            desiredChestAngularVelocities[trajectoryPointIndex].get(desiredAngularVelocity);
            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity);
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
         assertTrue(success);
         HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
         humanoidReferenceFrames.updateFrames();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         }

         desiredChestOrientationsList.add(desiredChestOrientations);
         desiredChestAngularVelocitiesList.add(desiredChestAngularVelocities);
         messageList.add(chestTrajectoryMessage);
      }


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      double timeOffset = 0.0;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         double simulationTime = 0.0;

         ChestTrajectoryMessage chestTrajectoryMessage = messageList.get(messageIndex);
         assertNumberOfWaypoints(chestTrajectoryMessage.getNumberOfTrajectoryPoints() + 1, scs);

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double time = chestTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex).getTime();
            FrameOrientation desiredOrientation = desiredChestOrientationsList.get(messageIndex)[trajectoryPointIndex];
            FrameVector desiredAngularVelocity = desiredChestAngularVelocitiesList.get(messageIndex)[trajectoryPointIndex];

            SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs);

            assertEquals(time + timeOffset, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            desiredOrientation.epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
            desiredAngularVelocity.epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

            simulationTime = Math.max(time, simulationTime);
         }
         timeOffset += simulationTime;
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
         assertTrue(success);
      }

      FrameOrientation desiredOrientation = desiredChestOrientationsList.get(numberOfMessages - 1)[numberOfTrajectoryPoints - 1];
      FrameVector desiredAngularVelocity = desiredChestAngularVelocitiesList.get(numberOfMessages - 1)[numberOfTrajectoryPoints - 1];
      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs);
      desiredOrientation.epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      desiredAngularVelocity.epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.3)
   @Test(timeout = 77000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.02;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

      List<FrameOrientation[]> desiredChestOrientationsList = new ArrayList<>();
      List<FrameVector[]> desiredChestAngularVelocitiesList = new ArrayList<>();
      List<ChestTrajectoryMessage> messageList = new ArrayList<>();

      double t = 0.0;
      double w = 2.0 * Math.PI / (trajectoryTime * numberOfMessages);
      double amp = Math.toRadians(20.0);
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FrameOrientation[] desiredChestOrientations = new FrameOrientation[numberOfTrajectoryPoints];
         FrameVector[] desiredChestAngularVelocities = new FrameVector[numberOfTrajectoryPoints];
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
         chestTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
         {
            long previousMessageId = id - 1;
            if (messageIndex == numberOfMessages - 1)
               previousMessageId = id + 100; // Bad ID

            chestTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, previousMessageId);
         }
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint;
            double pitch = amp * Math.sin((t + tOffset) * w);
            double pitchDot = w * amp * Math.cos((t + tOffset) * w);
            desiredChestOrientations[trajectoryPointIndex] = new FrameOrientation();
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector();
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);

            Quat4d desiredOrientation = new Quat4d();
            Vector3d desiredAngularVelocity = new Vector3d();

            desiredChestOrientations[trajectoryPointIndex].getQuaternion(desiredOrientation);
            desiredChestAngularVelocities[trajectoryPointIndex].get(desiredAngularVelocity);
            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity);
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
         assertTrue(success);
         HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
         humanoidReferenceFrames.updateFrames();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         }

         desiredChestOrientationsList.add(desiredChestOrientations);
         desiredChestAngularVelocitiesList.add(desiredChestAngularVelocities);
         messageList.add(chestTrajectoryMessage);
      }


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05 + getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(1, scs);
   }

   @ContinuousIntegrationTest(estimatedDuration = 16.8)
   @Test(timeout = 84000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double timePerWaypoint = 0.02;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody chest = fullRobotModel.getChest();


      List<FrameOrientation[]> desiredChestOrientationsList = new ArrayList<>();
      List<FrameVector[]> desiredChestAngularVelocitiesList = new ArrayList<>();
      List<ChestTrajectoryMessage> messageList = new ArrayList<>();

      double t = 0.0;
      double w = 2.0 * Math.PI / (trajectoryTime * numberOfMessages);
      double amp = Math.toRadians(20.0);
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FrameOrientation[] desiredChestOrientations = new FrameOrientation[numberOfTrajectoryPoints];
         FrameVector[] desiredChestAngularVelocities = new FrameVector[numberOfTrajectoryPoints];
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(numberOfTrajectoryPoints);
         chestTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            chestTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            double tOffset = messageIndex * numberOfTrajectoryPoints * timePerWaypoint;
            t = (trajectoryPointIndex + 1) * timePerWaypoint;
            double pitch = amp * Math.sin((t + tOffset) * w);
            double pitchDot = w * amp * Math.cos((t + tOffset) * w);
            desiredChestOrientations[trajectoryPointIndex] = new FrameOrientation();
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector();
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);

            Quat4d desiredOrientation = new Quat4d();
            Vector3d desiredAngularVelocity = new Vector3d();

            desiredChestOrientations[trajectoryPointIndex].getQuaternion(desiredOrientation);
            desiredChestAngularVelocities[trajectoryPointIndex].get(desiredAngularVelocity);
            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity);
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
         assertTrue(success);
         HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
         humanoidReferenceFrames.updateFrames();

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            desiredChestOrientations[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());
         }

         desiredChestOrientationsList.add(desiredChestOrientations);
         desiredChestAngularVelocitiesList.add(desiredChestAngularVelocities);
         messageList.add(chestTrajectoryMessage);
      }


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      RigidBody pelvis = fullRobotModel.getPelvis();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);

      Random random = new Random(21651L);
      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);

      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d desiredOrientation = new Quat4d();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      trajectoryTime = 0.5;
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation);
      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(2, scs);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      assertSingleWaypointExecuted(desiredRandomChestOrientation, scs);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 21.8)
   @Test (timeout = 110000)
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 5.0;
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody chest = fullRobotModel.getChest();

      OneDoFJoint[] spineClone = ScrewTools.cloneOneDoFJointPath(pelvis, chest);

      ScrewTestTools.setRandomPositionsWithinJointLimits(spineClone, random);

      RigidBody chestClone = spineClone[spineClone.length - 1].getSuccessor();
      FrameOrientation desiredRandomChestOrientation = new FrameOrientation(chestClone.getBodyFixedFrame());
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d desiredOrientation = new Quat4d();
      desiredRandomChestOrientation.getQuaternion(desiredOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage(trajectoryTime, desiredOrientation);

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      drcSimulationTestHelper.send(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      Quat4d desiredOrientationBeforeStop = findControllerDesiredOrientation(scs);

      assertFalse(findControllerStopBoolean(scs));

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.send(stopAllTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      assertTrue(success);

      Quat4d desiredOrientationAfterStop = findControllerDesiredOrientation(scs);

      assertTrue(findControllerStopBoolean(scs));
      JUnitTools.assertQuaternionsEqual(desiredOrientationBeforeStop, desiredOrientationAfterStop, 5.0e-2);
   }

   public static Quat4d findControllerDesiredOrientation(SimulationConstructionSet scs)
   {
      String chestPrefix = "chest";
      String subTrajectoryName = chestPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";
      return findQuat4d(subTrajectoryName, currentOrientationVarNamePrefix, scs);
   }

   public static Vector3d findControllerDesiredAngularVelocity(SimulationConstructionSet scs)
   {
      String chestPrefix = "chest";
      String subTrajectoryName = chestPrefix + "SubTrajectory";
      String currentAngularVelocityVarNamePrefix = subTrajectoryName + "CurrentAngularVelocity";

      return findVector3d(subTrajectoryName, currentAngularVelocityVarNamePrefix, scs);
   }

   public static boolean findControllerStopBoolean(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable(TaskspaceChestControlState.class.getSimpleName(), "isChestOrientationTrajectoryStopped")).getBooleanValue();
   }

   public static int findControllerNumberOfWaypoints(SimulationConstructionSet scs)
   {
      String chestPrefix = "chest";
      String orientationTrajectoryName = chestPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = chestPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static Vector3d findControlErrorRotationVector(SimulationConstructionSet scs, RigidBody chest)
   {
      String chestPrefix = chest.getName();
      String nameSpace = chestPrefix + AxisAngleOrientationController.class.getSimpleName();
      String varName = chestPrefix + "RotationErrorInBody";
      return findVector3d(nameSpace, varName, scs);
   }

   public static SimpleSO3TrajectoryPoint findTrajectoryPoint(int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String chestPrefix = "chest";
      String orientationTrajectoryName = chestPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = chestPrefix + "Time";
      String orientationName = chestPrefix + "Orientation";
      String angularVelocityName = chestPrefix + "AngularVelocity";

      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setTime(scs.getVariable(orientationTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSO3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSO3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSO3TrajectoryPoint;
   }

   public static SimpleSO3TrajectoryPoint findLastTrajectoryPoint(SimulationConstructionSet scs)
   {
      int numberOfWaypoints = findControllerNumberOfWaypoints(scs);
      return findTrajectoryPoint(numberOfWaypoints - 1, scs);
   }

   public static SimpleSO3TrajectoryPoint findCurrentDesiredTrajectoryPoint(SimulationConstructionSet scs)
   {
      SimpleSO3TrajectoryPoint simpleSO3TrajectoryPoint = new SimpleSO3TrajectoryPoint();
      simpleSO3TrajectoryPoint.setOrientation(findControllerDesiredOrientation(scs));
      simpleSO3TrajectoryPoint.setAngularVelocity(findControllerDesiredAngularVelocity(scs));
      return simpleSO3TrajectoryPoint;
   }

   public static void assertControlErrorIsLow(SimulationConstructionSet scs, RigidBody chest, double errorTolerance)
   {
      Vector3d error = findControlErrorRotationVector(scs, chest);
      boolean isErrorLow = error.length() <= errorTolerance;
      assertTrue("Error: " + error, isErrorLow);
   }

   public static void assertSingleWaypointExecuted(FrameOrientation desiredChestOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(2, scs);

      Quat4d controllerDesiredOrientation = findControllerDesiredOrientation(scs);
      JUnitTools.assertQuaternionsEqual(desiredChestOrientation.getQuaternion(), controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
   }

   public static void assertNumberOfWaypoints(int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs)
   {
      assertEquals(expectedNumberOfTrajectoryPoints, findControllerNumberOfWaypoints(scs));
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
