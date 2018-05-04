package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
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
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class EndToEndHandTrajectoryMessageTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
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

   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

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

         OneDoFJoint[] armOriginal = ScrewTools.createOneDoFJointPath(chest, hand);
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         for (int jointIndex = 0; jointIndex < armOriginal.length; jointIndex++)
         {
            OneDoFJoint original = armOriginal[jointIndex];
            OneDoFJoint clone = armClone[jointIndex];

            double limitLower = clone.getJointLimitLower();
            double limitUpper = clone.getJointLimitUpper();

            double randomQ = RandomNumbers.nextDouble(random, original.getQ() - 0.2, original.getQ() + 0.2);
            randomQ = MathTools.clamp(randomQ, limitLower, limitUpper);
            clone.setQ(randomQ);
         }

         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, chestFrame);
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

         drcSimulationTestHelper.send(handTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(humanoidReferenceFrames.getChestFrame());

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);

         String handName = fullRobotModel.getHand(robotSide).getName();
         assertSingleWaypointExecuted(handName, desiredPosition, desiredOrientation, scs);
      }
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      ThreadTools.sleep(1000);
      Random random = new Random(873736734567L);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      double scale = getLegLength() / 0.8;

      RobotSide robotSide = RobotSide.LEFT;
      double trajectoryTime = 1.0;
      Point3D position = new Point3D(0.5, robotSide.negateIfRightSide(0.5), 1.0);
      Quaternion orientation = new Quaternion();
      orientation.appendYawRotation(robotSide.negateIfRightSide(-Math.PI / 2.0));
      orientation.appendRollRotation(-Math.PI / 4.0);

      position.scale(scale);

      {
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
         handTrajectoryMessage.setRobotSide(robotSide.toByte());
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().setPosition(new Point3D(0.0, 0.0, 0.0));
         handTrajectoryMessage.getSe3Trajectory().setUseCustomControlFrame(true);
         handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(trajectoryTime, position, orientation, new Vector3D(), new Vector3D()));

         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(position);
         sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
         drcSimulationTestHelper.getSimulationConstructionSet().addStaticLinkGraphics(sphere);

         drcSimulationTestHelper.send(handTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.5);
         assertTrue(success);
      }

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();

      // This should not change the hand pose since the control frame change is compensated by a desireds change.
      {
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
         handTrajectoryMessage.setRobotSide(robotSide.toByte());

         handTrajectoryMessage.getSe3Trajectory().setUseCustomControlFrame(true);
         Point3D framePosition = EuclidCoreRandomTools.nextPoint3D(random, -0.1, 0.1);
         Quaternion frameOrientation = EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(20.0));
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().setPosition(framePosition);
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().setOrientation(frameOrientation);

         ReferenceFrame handBodyFrame = fullRobotModel.getHand(robotSide).getBodyFixedFrame();
         FrameVector3D frameFramePosition = new FrameVector3D(handBodyFrame, framePosition);
         frameFramePosition.changeFrame(worldFrame);
         position.add(frameFramePosition);

         handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(trajectoryTime, position, orientation, new Vector3D(), new Vector3D()));

         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(position);
         sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
         drcSimulationTestHelper.getSimulationConstructionSet().addStaticLinkGraphics(sphere);

         drcSimulationTestHelper.send(handTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 1.5);
         assertTrue(success);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);

      // TODO: add assert to make sure the hand did not move significantly.
   }

   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

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
         FramePoint3D circleCenter = new FramePoint3D(chestFrame);
         circleCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.35);
         circleCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint3D tempPoint = new FramePoint3D();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, circleCenter, taskspaceToJointspaceCalculator, 500);

         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
         handTrajectoryMessage.setRobotSide(robotSide.toByte());
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / (numberOfTrajectoryPoints - 1.0) * 2.0 * Math.PI;
            tempPoint.setIncludingFrame(chestFrame, 0.0, radius * Math.cos(angle), radius * Math.sin(angle));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(worldFrame);
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint);
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion(tempOrientation);
            Vector3D desiredAngularVelocity = new Vector3D();

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));

            SE3TrajectoryPointMessage point = HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
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
         lastFramePoint.set(lastPoint.getTime(), lastPoint.getPosition(), lastPoint.getOrientation(), lastPoint.getLinearVelocity(), lastPoint.getAngularVelocity());
         lastFramePoint.changeFrame(chestFrame);
         lastTrajectoryPoints.put(robotSide, lastFramePoint);

         String handName = fullRobotModel.getHand(robotSide).getName();
         assertNumberOfWaypoints(handName, numberOfTrajectoryPoints + 1, scs);

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage point = handTrajectoryPoints.get(robotSide).removeFirst();
            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(worldFrame);
            framePoint.set(point.getTime(), point.getPosition(), point.getOrientation(), point.getLinearVelocity(), point.getAngularVelocity());
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

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RobotSide robotSide = RobotSide.LEFT;
      String handName = fullRobotModel.getHand(robotSide).getName();
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      {
         int numberOfPoints = RigidBodyTaskspaceControlState.maxPoints;
         HandTrajectoryMessage message = new HandTrajectoryMessage();
         message.setRobotSide(robotSide.toByte());
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         double time = 0.05;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            message.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
            time = time + 0.05;
         }
         drcSimulationTestHelper.send(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         assertTrue(controllerState == RigidBodyControlMode.JOINTSPACE);
         assertNumberOfWaypoints(handName, 0, scs);
      }

      {
         int numberOfPoints = RigidBodyTaskspaceControlState.maxPoints - 1;
         HandTrajectoryMessage message = new HandTrajectoryMessage();
         message.setRobotSide(robotSide.toByte());
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         double time = 0.05;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            message.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
            time = time + 0.05;
         }
         drcSimulationTestHelper.send(message);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndArmTrajectoryMessageTest.findControllerState(handName, scs);
         assertTrue(controllerState == RigidBodyControlMode.TASKSPACE);
         assertNumberOfWaypoints(handName, RigidBodyTaskspaceControlState.maxPoints, drcSimulationTestHelper.getSimulationConstructionSet());
      }
   }

   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

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

      FramePoint3D sphereCenter = new FramePoint3D(chestBodyFixedFrame);
      sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
      sphereCenter.scale(scale);
      double radius = 0.15 * scale;
      FramePoint3D tempPoint = new FramePoint3D();
      TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
      FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);

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
         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint);
      }

      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      int calculatorIndex = 0;
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
         handTrajectoryMessage.setRobotSide(robotSide.toByte());
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(CommonReferenceFrameIds.CHEST_FRAME.getHashId());
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

         handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setMessageId(id);

         if (messageIndex > 0)
         {
            handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setPreviousMessageId(id - 1);
         }
         id++;
         double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion(tempOrientation);
            Vector3D desiredAngularVelocity = new Vector3D();

            double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));

            SE3TrajectoryPointMessage point = HumanoidMessageTools.createSE3TrajectoryPointMessage(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
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
      String nameSpaceRotation = FeedbackControllerToolbox.class.getSimpleName();
      String varnameRotation = handName + "ErrorRotationVector";
      Vector3D rotationError = findVector3d(nameSpaceRotation, varnameRotation, scs);

      String nameSpacePosition = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePosition = handName + "ErrorPosition";
      Vector3D positionError = findVector3d(nameSpacePosition, varnamePosition, scs);

      assertTrue(rotationError.length() < Math.toRadians(15.0));
      assertTrue(positionError.length() < 0.05);

      // check internal desired matches last trajectory point:
      String nameSpacePositionDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePositionDesired = handName + Type.DESIRED.getName() + Space.POSITION.getName();
      Vector3D desiredPosition = findVector3d(nameSpacePositionDesired, varnamePositionDesired, scs);

      String nameSpaceOrientationDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnameOrientationDesired = handName + Type.DESIRED.getName() + Space.ORIENTATION.getName();
      Quaternion desiredOrientation = findQuat4d(nameSpaceOrientationDesired, varnameOrientationDesired, scs);

      lastPoint.changeFrame(worldFrame);
      EuclidCoreTestTools.assertTuple3DEquals(lastPoint.getPositionCopy(), desiredPosition, 0.001);
      EuclidCoreTestTools.assertQuaternionEquals(lastPoint.getOrientationCopy(), desiredOrientation, 0.001);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

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
         FramePoint3D sphereCenter = new FramePoint3D(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         sphereCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint3D tempPoint = new FramePoint3D();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


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
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint);
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
            handTrajectoryMessage.setRobotSide(robotSide.toByte());
            handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
            handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
            handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setMessageId(id);

            if (messageIndex > 0)
            {
               long previousMessageId = id - 1;
               if (messageIndex == numberOfMessages - 1)
                  previousMessageId = id + 100; // Bad ID

               handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
               handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setPreviousMessageId(previousMessageId);
            }
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3D desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               Quaternion desiredOrientation = new Quaternion(tempOrientation);
               Vector3D desiredAngularVelocity = new Vector3D();

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));
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

   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 0.0, 0.5));

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
         FramePoint3D sphereCenter = new FramePoint3D(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         sphereCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint3D tempPoint = new FramePoint3D();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel, robotSide, sphereCenter, taskspaceToJointspaceCalculator, 500);


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
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint);
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
            handTrajectoryMessage.setRobotSide(robotSide.toByte());
            handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setMessageId(id);

            if (messageIndex > 0)
            {
               handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
               handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setPreviousMessageId(id - 1);
            }
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3D desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               Quaternion desiredOrientation = new Quaternion(tempOrientation);
               Vector3D desiredAngularVelocity = new Vector3D();

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));
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
      SideDependentList<FramePose3D> overridingPoses = new SideDependentList<>();
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModel.updateFrames();
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
         FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
         desiredHandPose.changeFrame(worldFrame);

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredHandPose.get(desiredPosition, desiredOrientation);
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide, overrideTrajectoryTime, desiredPosition, desiredOrientation, chestFrame);
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

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
         FramePose3D desiredPose = overridingPoses.get(robotSide);
         desiredPose.changeFrame(worldFrame);

         String handName = fullRobotModel.getHand(robotSide).getName();
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(handName, scs);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.setPosition(desiredPose.getPosition());
         expectedTrajectoryPoint.setOrientation(desiredPose.getOrientation());

         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
      }
   }

   public static FrameQuaternion computeBestOrientationForDesiredPosition(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
         FramePoint3D desiredPosition, TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator, int numberOfIterations)
   {
      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      Twist desiredTwist = new Twist(handFrame, chestFrame, handControlFrame);
      FramePose3D desiredPose = new FramePose3D(desiredPosition.getReferenceFrame());
      desiredPose.setPosition(desiredPosition);
      desiredPose.changeFrame(chestFrame);
      for (int i = 0; i < numberOfIterations; i++)
         taskspaceToJointspaceCalculator.compute(desiredPose, desiredTwist);
      taskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredPose, ReferenceFrame.getWorldFrame());

      FrameQuaternion tempOrientation = new FrameQuaternion(desiredPose.getOrientation());
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

   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

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

         FramePose3D desiredRandomHandPose = new FramePose3D(fullRobotModel.getHandControlFrame(robotSide));
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredRandomHandPose.prependTranslation(RandomGeometry.nextVector3D(random, 0.2));

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, chestFrame);
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

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
      return ((YoInteger) scs.getVariable(bodyName + "TaskspaceControlModule", bodyName + "TaskspaceNumberOfPoints")).getIntegerValue();
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

   public static void assertSingleWaypointExecuted(String bodyName, Point3DReadOnly desiredPosition, QuaternionReadOnly desiredOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(bodyName, 2, scs);

      Point3D controllerDesiredPosition = findControllerDesiredPosition(bodyName, scs);
      EuclidCoreTestTools.assertTuple3DEquals(desiredPosition, controllerDesiredPosition, EPSILON_FOR_DESIREDS);

      Quaternion controllerDesiredOrientation = findControllerDesiredOrientation(bodyName, scs);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
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
