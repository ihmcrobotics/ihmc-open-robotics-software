package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.*;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.EuclideanPositionController;
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
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndHandTrajectoryMessageTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      simulationTestingParameters.setUsePefectSensors(true);
   }

   private static final double EPSILON_FOR_DESIREDS = 1.0e-3;

   protected DRCSimulationTestHelper drcSimulationTestHelper;

   /**
    * Method used to scale down trajectories for different robots.
    * @return shinLength + thighLength of the robot
    */
   public abstract double getLegLength();

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 50000)
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
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 1.0;
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);

         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);

         ScrewTestTools.setRandomPositionsWithinJointLimits(armClone, random);

         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose desiredRandomHandPose = new FramePose(handClone.getBodyFixedFrame());
         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);
         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame, chestFrame);

         drcSimulationTestHelper.send(handTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(humanoidReferenceFrames.getChestFrame());

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);

         String handName = fullRobotModel.getHand(robotSide).getName();
         assertSingleWaypointExecuted(handName, desiredPosition, desiredOrientation, scs);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 50000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

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
      SideDependentList<ArrayDeque<SE3TrajectoryPointMessage>> handTrajectoryPoints = new SideDependentList<>(new ArrayDeque<>(), new ArrayDeque<>());
      SideDependentList<FrameSE3TrajectoryPoint> lastTrajectoryPoints = new SideDependentList<>();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RigidBody chest = fullRobotModel.getChest();
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint circleCenter = new FramePoint(chestFrame);
         circleCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.35);
         circleCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, circleCenter, taskspaceToJointspaceCalculator, 500);

         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
         handTrajectoryMessage.setDataReferenceFrameId(worldFrame);
         handTrajectoryMessage.setTrajectoryReferenceFrameId(chestFrame);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / (numberOfTrajectoryPoints - 1.0) * 2.0 * Math.PI;
            tempPoint.setIncludingFrame(chestFrame, 0.0, radius * Math.cos(angle), radius * Math.sin(angle));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(worldFrame);
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            handTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);

            SE3TrajectoryPointMessage point = new SE3TrajectoryPointMessage(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            handTrajectoryPoints.get(robotSide).addLast(point);
         }

         handTrajectoryMessages.put(robotSide, handTrajectoryMessage);

         drcSimulationTestHelper.send(handTrajectoryMessage);

      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);
      fullRobotModel.updateFrames();
      int expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);

      for (RobotSide robotSide : RobotSide.values)
      {
         
         SE3TrajectoryPointMessage lastPoint = handTrajectoryPoints.get(robotSide).peekLast();
         FrameSE3TrajectoryPoint lastFramePoint = new FrameSE3TrajectoryPoint(worldFrame);
         lastFramePoint.set(lastPoint.time, lastPoint.position, lastPoint.orientation, lastPoint.linearVelocity, lastPoint.angularVelocity);
         lastFramePoint.changeFrame(chestFrame);
         lastTrajectoryPoints.put(robotSide, lastFramePoint);
         
         String handName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(handName, numberOfTrajectoryPoints + 1, scs);

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage point = handTrajectoryPoints.get(robotSide).removeFirst();
            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(worldFrame);
            framePoint.set(point.time, point.position, point.orientation, point.linearVelocity, point.angularVelocity);
            framePoint.changeFrame(chestFrame);

            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(handName, trajectoryPointIndex, scs);
            SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
            framePoint.get(expectedTrajectoryPoint);

            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + firstTrajectoryPointTime);
      assertTrue(success);
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = fullRobotModel.getHand(robotSide).getName();
         FrameSE3TrajectoryPoint framePoint = lastTrajectoryPoints.get(robotSide);
         framePoint.changeFrame(worldFrame);

         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(handName, scs);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         framePoint.get(expectedTrajectoryPoint);

         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime());
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RobotSide robotSide = RobotSide.LEFT;
      String handName = fullRobotModel.getHand(robotSide).getName();
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      {
         int numberOfPoints = RigidBodyTaskspaceControlState.maxPoints;
         HandTrajectoryMessage message = new HandTrajectoryMessage(robotSide, numberOfPoints);
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.setTrajectoryReferenceFrameId(chestFrame);
         message.setDataReferenceFrameId(worldFrame);
         double time = 0.05;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            message.setTrajectoryPoint(pointIdx, time, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D(), worldFrame);
            time = time + 0.05;
         }
         drcSimulationTestHelper.send(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         assertTrue(controllerState == RigidBodyControlMode.JOINTSPACE);
         assertNumberOfWaypoints(handName, 0, scs);
      }

      {
         int numberOfPoints = RigidBodyTaskspaceControlState.maxPoints - 1;
         HandTrajectoryMessage message = new HandTrajectoryMessage(robotSide, numberOfPoints);
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.setTrajectoryReferenceFrameId(chestFrame);
         message.setDataReferenceFrameId(worldFrame);
         double time = 0.05;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            message.setTrajectoryPoint(pointIdx, time, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D(), worldFrame);
            time = time + 0.05;
         }
         drcSimulationTestHelper.send(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         assertTrue(controllerState == RigidBodyControlMode.TASKSPACE);
         assertNumberOfWaypoints(handName, RigidBodyTaskspaceControlState.maxPoints, drcSimulationTestHelper.getSimulationConstructionSet());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 40.0)
   @Test(timeout = 80000)
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

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

      ArrayList<HandTrajectoryMessage> handTrajectoryMessages = new ArrayList<>();
      ArrayDeque<FrameSE3TrajectoryPoint> handTrajectoryPoints = new ArrayDeque<>();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      RobotSide robotSide = RobotSide.LEFT;
      String handName = fullRobotModel.getHand(robotSide).getName();
      fullRobotModel.updateFrames();
      ReferenceFrame chestBodyFixedFrame = fullRobotModel.getChest().getBodyFixedFrame();

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;

      FramePoint sphereCenter = new FramePoint(chestBodyFixedFrame);
      sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
      sphereCenter.scale(scale);
      double radius = 0.15 * scale;
      FramePoint tempPoint = new FramePoint();
      TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
      FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
      {
         if (robotSide == RobotSide.RIGHT)
            pointsOnSphere[i].negate();
         tempPoint.setIncludingFrame(chestBodyFixedFrame, pointsOnSphere[i]);
         tempPoint.add(sphereCenter);
         tempPoint.changeFrame(worldFrame);
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
      }

      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      int calculatorIndex = 0;
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
         handTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            handTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;
         double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            handTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);

            SE3TrajectoryPointMessage point = new SE3TrajectoryPointMessage(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(worldFrame);
            framePoint.set(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            framePoint.changeFrame(chestBodyFixedFrame);
            handTrajectoryPoints.addLast(framePoint);

            calculatorIndex++;
         }

         handTrajectoryMessages.add(handTrajectoryMessage);
         drcSimulationTestHelper.send(handTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      fullRobotModel.updateFrames();
      assertTrue(success);
      
      double timeOffset = 0.0;
      int totalNumberOfPoints = numberOfMessages * numberOfTrajectoryPoints + 1;
      boolean firstSegment = true;
      FrameSE3TrajectoryPoint lastPoint = new FrameSE3TrajectoryPoint();

      while (true)
      {
         int expectedNumberOfPointsInGenerator = Math.min(totalNumberOfPoints, RigidBodyTaskspaceControlState.maxPointsInGenerator);
         if (firstSegment)
            expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);
         int expectedPointsInQueue = totalNumberOfPoints - expectedNumberOfPointsInGenerator;
         assertNumberOfWaypoints(handName, totalNumberOfPoints, scs);

         double lastPointTime = 0.0;
         fullRobotModel.updateFrames();

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            FrameSE3TrajectoryPoint framePoint = handTrajectoryPoints.removeFirst();

            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(handName, trajectoryPointIndex, scs);
            SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
            framePoint.get(expectedTrajectoryPoint);
            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));

            lastPointTime = Math.max(framePoint.getTime(), lastPointTime);
            lastPoint.setIncludingFrame(framePoint);
            System.out.println(trajectoryPointIndex);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(lastPointTime - timeOffset);
         assertTrue(success);

         timeOffset = lastPointTime;
         totalNumberOfPoints = totalNumberOfPoints - (expectedNumberOfPointsInGenerator - 1);
         firstSegment = false;

         if (expectedPointsInQueue == 0)
            break;
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      // check internal tracking is decent:
      String nameSpaceRotation = handName + AxisAngleOrientationController.class.getSimpleName();
      String varnameRotation = handName + "RotationErrorInBody";
      Vector3D rotationError = findVector3d(nameSpaceRotation, varnameRotation, scs);

      String nameSpacePosition = handName + EuclideanPositionController.class.getSimpleName();
      String varnamePosition = handName + "PositionError";
      Vector3D positionError = findVector3d(nameSpacePosition, varnamePosition, scs);

      assertTrue(rotationError.length() < Math.toRadians(10.0));
      assertTrue(positionError.length() < 0.05);

      // check internal desired matches last trajectory point:
      String nameSpacePositionDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePositionDesired = handName + Type.DESIRED.getName() + Space.POSITION.getName();
      Vector3D desiredPosition = findVector3d(nameSpacePositionDesired, varnamePositionDesired, scs);

      String nameSpaceOrientationDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnameOrientationDesired = handName + Type.DESIRED.getName() + Space.ORIENTATION.getName();
      Quaternion desiredOrientation = findQuat4d(nameSpaceOrientationDesired, varnameOrientationDesired, scs);

      lastPoint.changeFrame(worldFrame);
      EuclidCoreTestTools.assertTuple3DEquals(lastPoint.getPositionCopy().getPoint(), desiredPosition, 0.001);
      EuclidCoreTestTools.assertQuaternionEquals(lastPoint.getOrientationCopy().getQuaternion(), desiredOrientation, 0.001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
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

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;

      SideDependentList<ArrayList<HandTrajectoryMessage>> handTrajectoryMessages = new SideDependentList<>(new ArrayList<HandTrajectoryMessage>(), new ArrayList<HandTrajectoryMessage>());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint sphereCenter = new FramePoint(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         sphereCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(worldFrame);
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            handTrajectoryMessage.setUniqueId(id);
            handTrajectoryMessage.setDataReferenceFrameId(worldFrame);
            handTrajectoryMessage.setTrajectoryReferenceFrameId(chestFrame);
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
               Point3D desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               Quaternion desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);
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
         String handName = fullRobotModel.getHand(robotSide).getName();
         RigidBodyControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         assertTrue(controllerState == RigidBodyControlMode.JOINTSPACE);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 40000)
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

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint sphereCenter = new FramePoint(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         sphereCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint tempPoint = new FramePoint();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameOrientation tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
         euclideanTrajectoryPointCalculator.enableWeightMethod(2.0, 1.0);

         Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(worldFrame);
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            handTrajectoryMessage.setUniqueId(id);
            handTrajectoryMessage.setDataReferenceFrameId(worldFrame);
            handTrajectoryMessage.setTrajectoryReferenceFrameId(chestFrame);
            
            if (messageIndex > 0)
               handTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3D desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               Quaternion desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, worldFrame);
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
      SideDependentList<FramePose> overridingPoses = new SideDependentList<>();
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePose desiredHandPose = new FramePose(chestFrame);
         Point3D position = new Point3D(0.25, robotSide.negateIfRightSide(0.35), -0.4);
         position.scale(scale);
         desiredHandPose.setPosition(position);
         desiredHandPose.changeFrame(worldFrame);

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredHandPose.getPose(desiredPosition, desiredOrientation);
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, overrideTrajectoryTime, desiredPosition, desiredOrientation, worldFrame, chestFrame);

         drcSimulationTestHelper.send(handTrajectoryMessage);
         overridingPoses.put(robotSide, desiredHandPose);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         overridingPoses.get(robotSide).changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
         String handName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(handName, 2, scs);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(overrideTrajectoryTime + 1.0);
      assertTrue(success);
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose desiredPose = overridingPoses.get(robotSide);
         desiredPose.changeFrame(worldFrame);

         String handName = fullRobotModel.getHand(robotSide).getName();
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(handName, scs);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.setPosition(desiredPose.getPosition());
         expectedTrajectoryPoint.setOrientation(desiredPose.getOrientation());

         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
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

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 60000)
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
         String handName = fullRobotModel.getHand(robotSide).getName();

         OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);

         FramePose desiredRandomHandPose = new FramePose(fullRobotModel.getHandControlFrame(robotSide));
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredRandomHandPose.translate(RandomGeometry.nextVector3D(random, 0.2));

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.getPose(desiredPosition, desiredOrientation);
         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame, chestFrame);

         drcSimulationTestHelper.send(handTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         RigidBodyControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         assertEquals(RigidBodyControlMode.TASKSPACE, controllerState);

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

         controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         double[] controllerDesiredJointPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(armJoints, scs);
         double[] controllerDesiredJointVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(armJoints, scs);

         assertEquals(RigidBodyControlMode.JOINTSPACE, controllerState);
         assertArrayEquals(actualJointPositions, controllerDesiredJointPositions, 0.01);
         assertArrayEquals(zeroVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public static Point3D findControllerDesiredPosition(String bodyName, SimulationConstructionSet scs)
   {
      return findPoint3d(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.POSITION.getName(), scs);
   }

   public static Quaternion findControllerDesiredOrientation(String bodyName, SimulationConstructionSet scs)
   {
      return findQuat4d(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.ORIENTATION.getName(), scs);
   }

   public static Vector3D findControllerDesiredLinearVelocity(String bodyName, SimulationConstructionSet scs)
   {
      return findVector3d(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.LINEAR_VELOCITY.getName(), scs);
   }

   public static Vector3D findControllerDesiredAngularVelocity(String bodyName, SimulationConstructionSet scs)
   {
      return findVector3d(FeedbackControllerToolbox.class.getSimpleName(), bodyName + Type.DESIRED.getName() + Space.ANGULAR_VELOCITY.getName(), scs);
   }

   public static int findNumberOfWaypoints(String bodyName, SimulationConstructionSet scs)
   {
      return ((IntegerYoVariable) scs.getVariable(bodyName + "TaskspaceControlModule", bodyName + "TaskspaceNumberOfPoints")).getIntegerValue();
   }

   public static SimpleSE3TrajectoryPoint findTrajectoryPoint(String bodyName, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String positionTrajectoryName = bodyName + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = bodyName + "Time";
      String positionName = bodyName + "Position";
      String orientationName = bodyName + "Orientation";
      String linearVelocityName = bodyName + "LinearVelocity";
      String angularVelocityName = bodyName + "AngularVelocity";

      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setTime(scs.getVariable(positionTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.setPosition(findPoint3d(positionTrajectoryName, positionName, suffix, scs));
      simpleSE3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findVector3d(positionTrajectoryName, linearVelocityName, suffix, scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static SimpleSE3TrajectoryPoint findLastTrajectoryPoint(String bodyName, SimulationConstructionSet scs)
   {
      int numberOfWaypoints = findNumberOfWaypoints(bodyName, scs);
      return findTrajectoryPoint(bodyName, numberOfWaypoints - 1, scs);
   }

   public static SimpleSE3TrajectoryPoint findCurrentDesiredTrajectoryPoint(String bodyName, SimulationConstructionSet scs)
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      simpleSE3TrajectoryPoint.setPosition(findControllerDesiredPosition(bodyName, scs));
      simpleSE3TrajectoryPoint.setOrientation(findControllerDesiredOrientation(bodyName, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findControllerDesiredLinearVelocity(bodyName, scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(findControllerDesiredAngularVelocity(bodyName, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static void assertSingleWaypointExecuted(String bodyName, Point3D desiredPosition, Quaternion desiredOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(bodyName, 2, scs);

      Point3D controllerDesiredPosition = findControllerDesiredPosition(bodyName, scs);
      EuclidCoreTestTools.assertTuple3DEquals(desiredPosition, controllerDesiredPosition, EPSILON_FOR_DESIREDS);

      Quaternion controllerDesiredOrientation = findControllerDesiredOrientation(bodyName, scs);
      EuclidCoreTestTools.assertQuaternionEqualsSmart(desiredOrientation, controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
   }

   public static void assertNumberOfWaypoints(String bodyName, int expectedNumberOfTrajectoryPoints, SimulationConstructionSet scs)
   {
      assertEquals(expectedNumberOfTrajectoryPoints, findNumberOfWaypoints(bodyName, scs));
   }

   public static Quaternion findQuat4d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findQuat4d(nameSpace, varname, "", scs);
   }

   public static Quaternion findQuat4d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      double x = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQxName(prefix, suffix)).getValueAsDouble();
      double y = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQyName(prefix, suffix)).getValueAsDouble();
      double z = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQzName(prefix, suffix)).getValueAsDouble();
      double s = scs.getVariable(nameSpace, YoFrameVariableNameTools.createQsName(prefix, suffix)).getValueAsDouble();

      return new Quaternion(x, y, z, s);
   }

   public static Point3D findPoint3d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findPoint3d(nameSpace, varname, "", scs);
   }

   public static Point3D findPoint3d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Point3D(findTuple3d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Vector3D findVector3d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findVector3d(nameSpace, varname, "", scs);
   }

   public static Vector3D findVector3d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Vector3D(findTuple3d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Tuple3DBasics findTuple3d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      Tuple3DBasics tuple3d = new Point3D();
      tuple3d.setX(scs.getVariable(nameSpace, YoFrameVariableNameTools.createXName(prefix, suffix)).getValueAsDouble());
      tuple3d.setY(scs.getVariable(nameSpace, YoFrameVariableNameTools.createYName(prefix, suffix)).getValueAsDouble());
      tuple3d.setZ(scs.getVariable(nameSpace, YoFrameVariableNameTools.createZName(prefix, suffix)).getValueAsDouble());
      return tuple3d;
   }

   public static Point2D findPoint2d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findPoint2d(nameSpace, varname, "", scs);
   }

   public static Point2D findPoint2d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Point2D(findTuple2d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Vector2D findVector2d(String nameSpace, String varname, SimulationConstructionSet scs)
   {
      return findVector2d(nameSpace, varname, "", scs);
   }

   public static Vector2D findVector2d(String nameSpace, String varnamePrefix, String varnameSuffix, SimulationConstructionSet scs)
   {
      return new Vector2D(findTuple2d(nameSpace, varnamePrefix, varnameSuffix, scs));
   }

   public static Tuple2DBasics findTuple2d(String nameSpace, String prefix, String suffix, SimulationConstructionSet scs)
   {
      Tuple2DBasics tuple2d = new Point2D();
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
