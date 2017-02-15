package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;

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
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
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
import us.ihmc.robotics.geometry.RotationTools;
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
   private static final double EPSILON_FOR_DESIREDS = 1.0e-5;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 22.8)
   @Test (timeout = 110000)
   public void testLookingLeftAndRight() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      double trajectoryTime = 1.0;
      FrameOrientation lookStraightAhead = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame(), new Quat4d(0.0, 0.0, 0.0, 1.0));
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d lookLeftQuat = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(Math.PI / 8.0, Math.PI / 16.0, -Math.PI / 16.0, lookLeftQuat);
      FrameOrientation lookLeft = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quat4d lookRightQuat = new Quat4d();
      RotationTools.convertYawPitchRollToQuaternion(-Math.PI / 8.0, -Math.PI / 16.0, Math.PI / 16.0, lookRightQuat);
      FrameOrientation lookRight = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage lookStraightAheadMessage = new ChestTrajectoryMessage(trajectoryTime, lookStraightAhead.getQuaternion());
      lookStraightAheadMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookLeftMessage = new ChestTrajectoryMessage(trajectoryTime, lookLeft.getQuaternion());
      lookLeftMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));

      ChestTrajectoryMessage lookRightMessage = new ChestTrajectoryMessage(trajectoryTime, lookRight.getQuaternion());
      lookRightMessage.setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookRightMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * trajectoryTime + 1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 22.8)
   @Test (timeout = 110000)
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

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

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

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
         if (trajectoryPointIndex == numberOfTrajectoryPoints - 1)
            desiredChestAngularVelocities[trajectoryPointIndex].setToZero();

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
   public void testMessageWithALotOfTrajectoryPoints() throws Exception
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
      double previousTimeInState = timePerWaypoint;

      assertNumberOfWaypoints(Math.min(RigidBodyTaskspaceControlState.maxPoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs);

      double timeInState = 0.0;

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator - 1; trajectoryPointIndex++)
      {
         double time = chestTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex).getTime();
         SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(trajectoryPointIndex + 1, scs);
         assertEquals(time, controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         desiredChestOrientations[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
         desiredChestAngularVelocities[expectedTrajectoryPointIndex].epsilonEquals(controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);

         timeInState = Math.max(time, timeInState);

         expectedTrajectoryPointIndex++;

         if (expectedTrajectoryPointIndex == numberOfTrajectoryPoints)
            break;
      }

      double simulationTime = timeInState - previousTimeInState;
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      assertTrue(success);
      previousTimeInState = timeInState;

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

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      double timePerWaypoint = 0.05;
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
            desiredChestOrientations[trajectoryPointIndex] = new FrameOrientation(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestOrientations[trajectoryPointIndex].setYawPitchRoll(0.0, pitch, 0.0);
            desiredChestOrientations[trajectoryPointIndex].changeFrame(ReferenceFrame.getWorldFrame());
            desiredChestAngularVelocities[trajectoryPointIndex] = new FrameVector(humanoidReferenceFrames.getPelvisZUpFrame());
            desiredChestAngularVelocities[trajectoryPointIndex].set(0.0, pitchDot, 0.0);
            desiredChestAngularVelocities[trajectoryPointIndex].changeFrame(ReferenceFrame.getWorldFrame());

            if (messageIndex == numberOfMessages - 1 && trajectoryPointIndex == numberOfTrajectoryPoints - 1)
               desiredChestAngularVelocities[trajectoryPointIndex].setToZero();
            if (messageIndex == 0 && trajectoryPointIndex == 0)
               desiredChestAngularVelocities[trajectoryPointIndex].setToZero();

            Quat4d desiredOrientation = new Quat4d();
            Vector3d desiredAngularVelocity = new Vector3d();

            desiredChestOrientations[trajectoryPointIndex].getQuaternion(desiredOrientation);
            desiredChestAngularVelocities[trajectoryPointIndex].get(desiredAngularVelocity);
            chestTrajectoryMessage.setTrajectoryPoint(trajectoryPointIndex, t, desiredOrientation, desiredAngularVelocity);
         }
         drcSimulationTestHelper.send(chestTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
         assertTrue(success);
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

      int totalPoints = numberOfMessages * numberOfTrajectoryPoints + 1;
      assertNumberOfWaypoints(totalPoints, scs);

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         ChestTrajectoryMessage chestTrajectoryMessage = messageList.get(messageIndex);
         double simulationTime = chestTrajectoryMessage.getLastTrajectoryPoint().getTime();
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      FrameOrientation desiredOrientation = desiredChestOrientationsList.get(numberOfMessages - 1)[numberOfTrajectoryPoints - 1];
      FrameVector desiredAngularVelocity = desiredChestAngularVelocitiesList.get(numberOfMessages - 1)[numberOfTrajectoryPoints - 1];
      SimpleSO3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(scs);
      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      desiredAngularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      JUnitTools.assertQuaternionsEqual(desiredOrientation.getQuaternion(), controllerTrajectoryPoint.getOrientationCopy(), 1.0e-3);
      JUnitTools.assertVector3dEquals("", desiredAngularVelocity.getVector(), controllerTrajectoryPoint.getAngularVelocityCopy(), 1.0e-3);

      int maxPointsInGenerator = RigidBodyTaskspaceControlState.maxPointsInGenerator;
      int pointsInLastTrajectory = totalPoints - Math.min((numberOfTrajectoryPoints + 1), maxPointsInGenerator); // fist set in generator
      while (pointsInLastTrajectory > (maxPointsInGenerator - 1))
         pointsInLastTrajectory -= (maxPointsInGenerator - 1); // keep filling the generator
      pointsInLastTrajectory++;

      assertNumberOfWaypoints(pointsInLastTrajectory, scs);
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

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(humanoidReferenceFrames.getPelvisZUpFrame());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()));
      assertNumberOfWaypoints(2, scs);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.0));
      humanoidReferenceFrames.updateFrames();
      desiredRandomChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

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

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      Quat4d desiredOrientationAfterStop = findControllerDesiredOrientation(scs);

      assertTrue(findControllerStopBoolean(scs));
      JUnitTools.assertQuaternionsEqual(desiredOrientationBeforeStop, desiredOrientationAfterStop, 1.0e-3);
      assertControlErrorIsLow(scs, chest, 1.0e-2);
   }

   public static Quat4d findControllerDesiredOrientation(SimulationConstructionSet scs)
   {
      return findQuat4d("FeedbackControllerToolbox", "utorsoDesiredOrientation", scs);
   }

   public static Vector3d findControllerDesiredAngularVelocity(SimulationConstructionSet scs)
   {
      return findVector3d("FeedbackControllerToolbox", "utorsoDesiredAngularVelocity", scs);
   }

   public static boolean findControllerStopBoolean(SimulationConstructionSet scs)
   {
      return ((BooleanYoVariable) scs.getVariable("utorsoTaskspaceControlModule", "utorsoTaskspaceTrajectoryStopped")).getBooleanValue();
   }

   public static int findControllerNumberOfWaypoints(SimulationConstructionSet scs)
   {
      int numberOfWaypoints = ((IntegerYoVariable) scs.getVariable("utorsoTaskspaceControlModule", "utorsoTaskspaceNumberOfPoints")).getIntegerValue();
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
      String chestPrefix = "utorso";
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
      FrameOrientation controllerDesiredFrameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), controllerDesiredOrientation);
      controllerDesiredFrameOrientation.changeFrame(desiredChestOrientation.getReferenceFrame());

      JUnitTools.assertQuaternionsEqual(desiredChestOrientation.getQuaternion(), controllerDesiredFrameOrientation.getQuaternion(), EPSILON_FOR_DESIREDS);
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
