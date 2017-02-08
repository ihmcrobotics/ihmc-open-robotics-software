package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHandTrajectoryMessageTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 23.2)
   @Test(timeout = 120000)
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
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, trajectoryTime, desiredPosition,
               desiredOrientation);

         drcSimulationTestHelper.send(handTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 22.5)
   @Test(timeout = 110000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 1.5;
      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 25;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

      SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint circleCenter = new FramePoint(chestFrame);
         circleCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.35);
         double radius = 0.15;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, circleCenter, taskspaceToJointspaceCalculator, 500);

         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, numberOfTrajectoryPoints);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / (numberOfTrajectoryPoints - 1.0) * 2.0 * Math.PI;
            tempPoint.setIncludingFrame(chestFrame, 0.0, radius * Math.cos(angle), radius * Math.sin(angle));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3d desiredPosition = new Point3d();
            Vector3d desiredLinearVelocity = new Vector3d();
            Quat4d desiredOrientation = new Quat4d();
            Vector3d desiredAngularVelocity = new Vector3d();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            handTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
         }

         handTrajectoryMessages.put(robotSide, handTrajectoryMessage);

         drcSimulationTestHelper.send(handTrajectoryMessage);

      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         assertNumberOfWaypoints(robotSide, numberOfTrajectoryPoints + 1, scs);

         HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide);

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex);
            SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
            expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(robotSide, trajectoryPointIndex + 1, scs);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + firstTrajectoryPointTime);
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide);

         SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 27.3)
   @Test(timeout = 140000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = 7.0;

      SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint sphereCenter = new FramePoint(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         double radius = 0.15;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);

         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, numberOfTrajectoryPoints);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3d[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints);

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3d desiredPosition = new Point3d();
            Vector3d desiredLinearVelocity = new Vector3d();
            Quat4d desiredOrientation = new Quat4d();
            Vector3d desiredAngularVelocity = new Vector3d();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            handTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
         }

         handTrajectoryMessages.put(robotSide, handTrajectoryMessage);

         drcSimulationTestHelper.send(handTrajectoryMessage);

      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(firstTrajectoryPointTime + 2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      int expectedTrajectoryPointIndex = 0;
      boolean isDone = false;
      double previousTimeInState = firstTrajectoryPointTime;

      while(!isDone)
      {
         for (RobotSide robotSide : RobotSide.values)
            assertNumberOfWaypoints(robotSide, Math.min(defaultMaximumNumberOfWaypoints, numberOfTrajectoryPoints - expectedTrajectoryPointIndex + 1), scs);

         double timeInState = 0.0;

         for (int trajectoryPointIndex = 0; trajectoryPointIndex < defaultMaximumNumberOfWaypoints - 1; trajectoryPointIndex++)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide);

               SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getTrajectoryPoint(expectedTrajectoryPointIndex);
               SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
               expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
               SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(robotSide, trajectoryPointIndex + 1, scs);
               assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

               timeInState = Math.max(fromMessage.time, timeInState);
            }

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

      for (RobotSide robotSide : RobotSide.values)
      {
         HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide);

         SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 31.3)
   @Test(timeout = 160000)
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = 7.0;

      SideDependentList<ArrayList<HandTrajectoryMessage>> handTrajectoryMessages = new SideDependentList<>(new ArrayList<HandTrajectoryMessage>(), new ArrayList<HandTrajectoryMessage>());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint sphereCenter = new FramePoint(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         double radius = 0.15;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3d[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, numberOfTrajectoryPoints);
            handTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               handTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3d desiredPosition = new Point3d();
               Vector3d desiredLinearVelocity = new Vector3d();
               Quat4d desiredOrientation = new Quat4d();
               Vector3d desiredAngularVelocity = new Vector3d();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
               calculatorIndex++;
            }

            handTrajectoryMessages.get(robotSide).add(handTrajectoryMessage);
            drcSimulationTestHelper.send(handTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      double timeOffset = 0.0;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         double simulationTime = 0.0;

         for (RobotSide robotSide : RobotSide.values)
         {
            HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide).get(messageIndex);

            assertNumberOfWaypoints(robotSide, handTrajectoryMessage.getNumberOfTrajectoryPoints() + 1, scs);

            for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
            {
               SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getTrajectoryPoint(trajectoryPointIndex);
               SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
               expectedTrajectoryPoint.set(fromMessage.time + timeOffset, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
               SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(robotSide, trajectoryPointIndex + 1, scs);
               assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));

               simulationTime = Math.max(fromMessage.time, simulationTime);
            }
         }
         timeOffset += simulationTime;
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
         assertTrue(success);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide).get(numberOfMessages - 1);

         SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 14.0)
   @Test(timeout = 70000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = 7.0;

      SideDependentList<ArrayList<HandTrajectoryMessage>> handTrajectoryMessages = new SideDependentList<>(new ArrayList<HandTrajectoryMessage>(), new ArrayList<HandTrajectoryMessage>());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint sphereCenter = new FramePoint(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         double radius = 0.15;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3d[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, numberOfTrajectoryPoints);
            handTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
            {
               long previousMessageId = id - 1;
               if (messageIndex == numberOfMessages - 1)
                  previousMessageId = id + 100; // Bad ID

               handTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, previousMessageId);
            }
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3d desiredPosition = new Point3d();
               Vector3d desiredLinearVelocity = new Vector3d();
               Quat4d desiredOrientation = new Quat4d();
               Vector3d desiredAngularVelocity = new Vector3d();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
               calculatorIndex++;
            }

            handTrajectoryMessages.get(robotSide).add(handTrajectoryMessage);
            drcSimulationTestHelper.send(handTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05 + getRobotModel().getControllerDT());
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         HandControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(robotSide, scs);
         assertTrue(controllerState == HandControlMode.JOINTSPACE);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 14.5)
   @Test(timeout = 72000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double trajectoryTime = 7.0;

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint sphereCenter = new FramePoint(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         double radius = 0.15;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3d[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, numberOfTrajectoryPoints);
            handTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               handTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3d desiredPosition = new Point3d();
               Vector3d desiredLinearVelocity = new Vector3d();
               Quat4d desiredOrientation = new Quat4d();
               Vector3d desiredAngularVelocity = new Vector3d();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
               calculatorIndex++;
            }

            drcSimulationTestHelper.send(handTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);

      double overrideTrajectoryTime = 1.0;
      SideDependentList<HandTrajectoryMessage> overridingMessages = new SideDependentList<>();
      Random random = new Random(792L);

      for (RobotSide robotSide : RobotSide.values)
      {
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
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, overrideTrajectoryTime, desiredPosition,
               desiredOrientation);

         drcSimulationTestHelper.send(handTrajectoryMessage);
         overridingMessages.put(robotSide, handTrajectoryMessage);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         assertNumberOfWaypoints(robotSide, 2, scs);
         
         SE3TrajectoryPointMessage fromMessage = overridingMessages.get(robotSide).getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findLastTrajectoryPoint(robotSide, scs);
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(overrideTrajectoryTime);
      assertTrue(success);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         HandTrajectoryMessage handTrajectoryMessage = overridingMessages.get(robotSide);

         SE3TrajectoryPointMessage fromMessage = handTrajectoryMessage.getLastTrajectoryPoint();
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.time, fromMessage.position, fromMessage.orientation, fromMessage.linearVelocity, fromMessage.angularVelocity);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(robotSide, scs);
         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime()); // Don't want to check the time here.
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, EPSILON_FOR_DESIREDS));
      }
   }

   public static FrameOrientation computeBestOrientationForDesiredPosition(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
         FramePoint desiredPosition, TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator, int numberOfIterations)
   {
      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      Twist desiredTwist = new Twist(handFrame, chestFrame, handControlFrame);
      FramePose desiredPose = new FramePose(desiredPosition.getReferenceFrame());
      desiredPose.setPosition(desiredPosition);
      desiredPose.changeFrame(chestFrame);
      for (int i = 0; i < numberOfIterations; i++)
         taskspaceToJointspaceCalculator.compute(desiredPose, desiredTwist);
      taskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredPose, ReferenceFrame.getWorldFrame());

      FrameOrientation tempOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
      desiredPose.getOrientationIncludingFrame(tempOrientation);
      return tempOrientation;
   }

   public static TaskspaceToJointspaceCalculator createTaskspaceToJointspaceCalculator(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = new TaskspaceToJointspaceCalculator("blop", chest, hand, 0.005, new YoVariableRegistry("Dummy"));
      taskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
      taskspaceToJointspaceCalculator.setupWithDefaultParameters();
      DenseMatrix64F selectionMatrix = CommonOps.identity(6);
      for (int i = 0; i < 3; i++)
         MatrixTools.removeRow(selectionMatrix, 0);
      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);
      return taskspaceToJointspaceCalculator;
   }

   @ContinuousIntegrationTest(estimatedDuration = 21.2)
   @Test(timeout = 110000)
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
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, BaseForControl.WORLD, trajectoryTime, desiredPosition,
               desiredOrientation);

         drcSimulationTestHelper.send(handTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         HandControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(robotSide, scs);
         assertEquals(HandControlMode.TASKSPACE, controllerState);

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
         double[] controllerDesiredJointPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(armJoints, scs);
         double[] controllerDesiredJointVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(armJoints, scs);

         assertEquals(HandControlMode.JOINTSPACE, controllerState);
         assertEquals(timeStopSent, switchTime, getRobotModel().getControllerDT());
         assertArrayEquals(actualJointPositions, controllerDesiredJointPositions, 0.01);
         assertArrayEquals(zeroVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public static Point3d findControllerDesiredPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String subTrajectoryName = handPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      return findPoint3d(subTrajectoryName, currentPositionVarNamePrefix, scs);
   }

   public static Quat4d findControllerDesiredOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String subTrajectoryName = handPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      return findQuat4d(subTrajectoryName, currentOrientationVarNamePrefix, scs);
   }

   public static Vector3d findControllerDesiredLinearVelocity(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String subTrajectoryName = handPrefix + "SubTrajectory";
      String currentLinearVelocityVarNamePrefix = subTrajectoryName + "CurrentVelocity";

      return findVector3d(subTrajectoryName, currentLinearVelocityVarNamePrefix, scs);
   }

   public static Vector3d findControllerDesiredAngularVelocity(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String subTrajectoryName = handPrefix + "SubTrajectory";
      String currentAngularVelocityVarNamePrefix = subTrajectoryName + "CurrentAngularVelocity";

      return findVector3d(subTrajectoryName, currentAngularVelocityVarNamePrefix, scs);
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

   public static SimpleSE3TrajectoryPoint findTrajectoryPoint(RobotSide robotSide, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String handPrefix = sidePrefix + "Hand";
      String positionTrajectoryName = handPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String orientationTrajectoryName = handPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = handPrefix + "Time";
      String positionName = handPrefix + "Position";
      String orientationName = handPrefix + "Orientation";
      String linearVelocityName = handPrefix + "LinearVelocity";
      String angularVelocityName = handPrefix + "AngularVelocity";

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setTime(scs.getVariable(positionTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.setPosition(findPoint3d(positionTrajectoryName, positionName, suffix, scs));
      simpleSE3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findVector3d(positionTrajectoryName, linearVelocityName, suffix, scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static SimpleSE3TrajectoryPoint findLastTrajectoryPoint(RobotSide robotSide, SimulationConstructionSet scs)
   {
      int numberOfWaypoints = findNumberOfWaypointsForPosition(robotSide, scs);
      return findTrajectoryPoint(robotSide, numberOfWaypoints - 1, scs);
   }

   public static SimpleSE3TrajectoryPoint findCurrentDesiredTrajectoryPoint(RobotSide robotSide, SimulationConstructionSet scs)
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setPosition(findControllerDesiredPosition(robotSide, scs));
      simpleSE3TrajectoryPoint.setOrientation(findControllerDesiredOrientation(robotSide, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findControllerDesiredLinearVelocity(robotSide, scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findControllerDesiredAngularVelocity(robotSide, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static void assertSingleWaypointExecuted(RobotSide robotSide, Point3d desiredPosition, Quat4d desiredOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(robotSide, 2, scs);

      Point3d controllerDesiredPosition = findControllerDesiredPosition(robotSide, scs);
      JUnitTools.assertTuple3dEquals(desiredPosition, controllerDesiredPosition, EPSILON_FOR_DESIREDS);

      Quat4d controllerDesiredOrientation = findControllerDesiredOrientation(robotSide, scs);
      JUnitTools.assertQuaternionsEqual(desiredOrientation, controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
   }

   public static void assertNumberOfWaypoints(RobotSide robotSide, int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs)
   {
      assertEquals(expectedNumberOfTrajectoryPoints, findNumberOfWaypointsForPosition(robotSide, scs));
      assertEquals(expectedNumberOfTrajectoryPoints, findNumberOfWaypointsForOrientation(robotSide, scs));
   }

   public static Quat4d findQuat4d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findQuat4d(nameSpace, varname, "", scs);
   }

   public static Quat4d findQuat4d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      Quat4d quat4d = new Quat4d();
      quat4d.setX(scs.getVariable(nameSpace, YoFrameVariableNameTools.createQxName(prefix, suffix)).getValueAsDouble());
      quat4d.setY(scs.getVariable(nameSpace, YoFrameVariableNameTools.createQyName(prefix, suffix)).getValueAsDouble());
      quat4d.setZ(scs.getVariable(nameSpace, YoFrameVariableNameTools.createQzName(prefix, suffix)).getValueAsDouble());
      quat4d.setW(scs.getVariable(nameSpace, YoFrameVariableNameTools.createQsName(prefix, suffix)).getValueAsDouble());
      return quat4d;
   }

   public static Point3d findPoint3d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findPoint3d(nameSpace, varname, "", scs);
   }

   public static Point3d findPoint3d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Point3d(findTuple3d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Vector3d findVector3d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findVector3d(nameSpace, varname, "", scs);
   }

   public static Vector3d findVector3d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Vector3d(findTuple3d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Tuple3d findTuple3d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      Tuple3d tuple3d = new Point3d();
      tuple3d.setX(scs.getVariable(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix)).getValueAsDouble());
      tuple3d.setY(scs.getVariable(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix)).getValueAsDouble());
      tuple3d.setZ(scs.getVariable(nameSpace, YoFrameVariableNameTools.createZName(prefix, suffix)).getValueAsDouble());
      return tuple3d;
   }

   public static Point2d findPoint2d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findPoint2d(nameSpace, varname, "", scs);
   }

   public static Point2d findPoint2d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Point2d(findTuple2d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Vector2d findVector2d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findVector2d(nameSpace, varname, "", scs);
   }

   public static Vector2d findVector2d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Vector2d(findTuple2d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Tuple2d findTuple2d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      Tuple2d tuple2d = new Point2d();
      tuple2d.setX(scs.getVariable(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix)).getValueAsDouble());
      tuple2d.setY(scs.getVariable(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix)).getValueAsDouble());
      return tuple2d;
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
