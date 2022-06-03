package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class EndToEndHandTrajectoryMessageTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setUsePefectSensors(true);
   }

   private static final double EPSILON_FOR_DESIREDS = 1.0e-3;

   protected SCS2AvatarTestingSimulation simulationTestHelper;

   /**
    * Method used to scale down trajectories for different robots.
    * 
    * @return shinLength + thighLength of the robot
    */
   public abstract double getLegLength();

   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 1.0;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         OneDoFJointBasics[] armOriginal = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         OneDoFJointBasics[] armClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(chest, hand);
         for (int jointIndex = 0; jointIndex < armOriginal.length; jointIndex++)
         {
            OneDoFJointBasics original = armOriginal[jointIndex];
            OneDoFJointBasics clone = armClone[jointIndex];

            double limitLower = clone.getJointLimitLower();
            double limitUpper = clone.getJointLimitUpper();

            double randomQ = RandomNumbers.nextDouble(random, original.getQ() - 0.2, original.getQ() + 0.2);
            randomQ = MathTools.clamp(randomQ, limitLower, limitUpper);
            clone.setQ(randomQ);
         }

         RigidBodyBasics handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                        trajectoryTime,
                                                                                                        desiredPosition,
                                                                                                        desiredOrientation,
                                                                                                        worldFrame);
         handTrajectoryMessage.setSequenceId(random.nextLong());

         // ROS1 users have these fields set to zero by default which can cause an exception to be thrown even if these fields are not used.
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().getOrientation().setUnsafe(0.0, 0.0, 0.0, 0.0);

         simulationTestHelper.publishToController(handTrajectoryMessage);
         success = simulationTestHelper.simulateNow(3 * controllerDT);
         humanoidReferenceFrames.updateFrames();
         String handName = fullRobotModel.getHand(robotSide).getName();

         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(handTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.STARTED,
                                                           0.0,
                                                           handName,
                                                           statusMessages.remove(0),
                                                           controllerDT);

         success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
         assertTrue(success);

         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);

         assertSingleWaypointExecuted(handName, desiredPosition, desiredOrientation, simulationTestHelper);

         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(handTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           trajectoryTime,
                                                           desiredRandomHandPose,
                                                           handName,
                                                           statusMessages.remove(0),
                                                           1.0e-12,
                                                           controllerDT);
      }

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testForceExecutionWithSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      simulationTestHelper.publishToController(EndToEndTestTools.generateStepsInPlace(simulationTestHelper.getControllerFullRobotModel(), 10));
      success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 1.0;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         OneDoFJointBasics[] armOriginal = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         OneDoFJointBasics[] armClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(chest, hand);
         for (int jointIndex = 0; jointIndex < armOriginal.length; jointIndex++)
         {
            OneDoFJointBasics original = armOriginal[jointIndex];
            OneDoFJointBasics clone = armClone[jointIndex];

            double limitLower = clone.getJointLimitLower();
            double limitUpper = clone.getJointLimitUpper();

            double randomQ = RandomNumbers.nextDouble(random, original.getQ() - 0.2, original.getQ() + 0.2);
            randomQ = MathTools.clamp(randomQ, limitLower, limitUpper);
            clone.setQ(randomQ);
         }

         RigidBodyBasics handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                        trajectoryTime,
                                                                                                        desiredPosition,
                                                                                                        desiredOrientation,
                                                                                                        worldFrame);
         handTrajectoryMessage.setForceExecution(true);
         handTrajectoryMessage.setSequenceId(random.nextLong());

         // ROS1 users have these fields set to zero by default which can cause an exception to be thrown even if these fields are not used.
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().getOrientation().setUnsafe(0.0, 0.0, 0.0, 0.0);

         simulationTestHelper.publishToController(handTrajectoryMessage);
         success = simulationTestHelper.simulateNow(controllerDT);
         humanoidReferenceFrames.updateFrames();
         String handName = fullRobotModel.getHand(robotSide).getName();

         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(handTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.STARTED,
                                                           0.0,
                                                           handName,
                                                           statusMessages.remove(0),
                                                           controllerDT);

         success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
         assertTrue(success);

         humanoidReferenceFrames.updateFrames();
         desiredRandomHandPose.changeFrame(HumanoidReferenceFrames.getWorldFrame());
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);

         assertSingleWaypointExecuted(handName, desiredPosition, desiredOrientation, simulationTestHelper);

         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(handTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           trajectoryTime,
                                                           desiredRandomHandPose,
                                                           handName,
                                                           statusMessages.remove(0),
                                                           1.0e-12,
                                                           controllerDT);
      }

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      ThreadTools.sleep(1000);
      Random random = new Random(873736734567L);
      boolean success = simulationTestHelper.simulateNow(1.0);
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
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().getPosition().set(new Point3D(0.0, 0.0, 0.0));
         handTrajectoryMessage.getSe3Trajectory().setUseCustomControlFrame(true);
         handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                              .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(trajectoryTime, position, orientation, new Vector3D(), new Vector3D()));

         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.appendTranslation(position);
         visualDefinitionFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
         simulationTestHelper.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());

         simulationTestHelper.publishToController(handTrajectoryMessage);
         success = simulationTestHelper.simulateNow(trajectoryTime + 1.5);
         assertTrue(success);
      }

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();

      // This should not change the hand pose since the control frame change is compensated by a desireds change.
      {
         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
         handTrajectoryMessage.setRobotSide(robotSide.toByte());

         handTrajectoryMessage.getSe3Trajectory().setUseCustomControlFrame(true);
         Point3D framePosition = EuclidCoreRandomTools.nextPoint3D(random, -0.1, 0.1);
         Quaternion frameOrientation = EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(20.0));
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().getPosition().set(framePosition);
         handTrajectoryMessage.getSe3Trajectory().getControlFramePose().getOrientation().set((Orientation3DReadOnly) frameOrientation);

         ReferenceFrame handBodyFrame = fullRobotModel.getHand(robotSide).getBodyFixedFrame();
         FrameVector3D frameFramePosition = new FrameVector3D(handBodyFrame, framePosition);
         frameFramePosition.changeFrame(worldFrame);
         position.add(frameFramePosition);

         handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                              .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(trajectoryTime, position, orientation, new Vector3D(), new Vector3D()));

         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.appendTranslation(position);
         visualDefinitionFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
         simulationTestHelper.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());

         simulationTestHelper.publishToController(handTrajectoryMessage);
         success = simulationTestHelper.simulateNow(trajectoryTime + 1.5);
         assertTrue(success);
      }

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);

      // TODO: add assert to make sure the hand did not move significantly.
   }

   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      Random random = new Random(34536);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 1.5;
      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 26;
      double trajectoryTime = (numberOfTrajectoryPoints - 1) * timePerWaypoint;

      SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>();
      SideDependentList<ArrayDeque<SE3TrajectoryPointMessage>> handTrajectoryPoints = new SideDependentList<>(new ArrayDeque<>(), new ArrayDeque<>());
      SideDependentList<FrameSE3TrajectoryPoint> lastTrajectoryPoints = new SideDependentList<>();

      RigidBodyBasics chest = fullRobotModel.getChest();
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;
      boolean useTimeOptimizer = true;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint3D circleCenter = new FramePoint3D(chestFrame);
         circleCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.35);
         circleCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint3D tempPoint = new FramePoint3D();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel,
                                                                                    robotSide,
                                                                                    circleCenter,
                                                                                    taskspaceToJointspaceCalculator,
                                                                                    500);

         HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
         handTrajectoryMessage.setSequenceId(random.nextLong());
         handTrajectoryMessage.setRobotSide(robotSide.toByte());
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / (numberOfTrajectoryPoints - 1.0) * 2.0 * Math.PI;
            tempPoint.setIncludingFrame(chestFrame, 0.0, radius * Math.cos(angle), radius * Math.sin(angle));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(worldFrame);
            if (useTimeOptimizer)
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint);
            else
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(i * timePerWaypoint, tempPoint);
         }

         euclideanTrajectoryPointCalculator.compute(trajectoryTime);

         FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
         trajectoryPoints.addTimeOffset(firstTrajectoryPointTime);

         for (int i = 0; i < trajectoryPoints.getNumberOfTrajectoryPoints(); i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion(tempOrientation);
            Vector3D desiredAngularVelocity = new Vector3D();

            trajectoryPoints.getTrajectoryPoint(i).get(desiredPosition, desiredLinearVelocity);
            double time = trajectoryPoints.getTrajectoryPoint(i).getTime();

            VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
            visualDefinitionFactory.appendTranslation(desiredPosition);
            visualDefinitionFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
            simulationTestHelper.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());

            handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                                 .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time,
                                                                                           desiredPosition,
                                                                                           desiredOrientation,
                                                                                           desiredLinearVelocity,
                                                                                           desiredAngularVelocity));

            SE3TrajectoryPointMessage point = HumanoidMessageTools.createSE3TrajectoryPointMessage(time,
                                                                                                   desiredPosition,
                                                                                                   desiredOrientation,
                                                                                                   desiredLinearVelocity,
                                                                                                   desiredAngularVelocity);
            handTrajectoryPoints.get(robotSide).addLast(point);
         }

         handTrajectoryMessages.put(robotSide, handTrajectoryMessage);

         simulationTestHelper.publishToController(handTrajectoryMessage);
      }

      success = simulationTestHelper.simulateNow(2.0 * controllerDT);
      assertTrue(success);
      fullRobotModel.updateFrames();
      int expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);

      assertEquals(2, statusMessages.size());

      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = fullRobotModel.getHand(robotSide).getName();
         TaskspaceTrajectoryStatusMessage statusMessage = statusMessages.stream().filter(m -> m.getEndEffectorName().toString().equals(handName)).findFirst()
                                                                        .get();
         statusMessages.remove(statusMessage);
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(handTrajectoryMessages.get(robotSide).getSequenceId(),
                                                           TrajectoryExecutionStatus.STARTED,
                                                           0.0,
                                                           handName,
                                                           statusMessage,
                                                           controllerDT);
      }

      for (RobotSide robotSide : RobotSide.values)
      {

         SE3TrajectoryPointMessage lastPoint = handTrajectoryPoints.get(robotSide).peekLast();
         FrameSE3TrajectoryPoint lastFramePoint = new FrameSE3TrajectoryPoint(worldFrame);
         lastFramePoint.set(lastPoint.getTime(),
                            lastPoint.getPosition(),
                            lastPoint.getOrientation(),
                            lastPoint.getLinearVelocity(),
                            lastPoint.getAngularVelocity());
         lastTrajectoryPoints.put(robotSide, lastFramePoint);

         String handName = fullRobotModel.getHand(robotSide).getName();
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(handName, numberOfTrajectoryPoints + 1, simulationTestHelper);

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage point = handTrajectoryPoints.get(robotSide).removeFirst();
            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(worldFrame);
            framePoint.set(point.getTime(), point.getPosition(), point.getOrientation(), point.getLinearVelocity(), point.getAngularVelocity());

            SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findSE3TrajectoryPoint(handName, trajectoryPointIndex, simulationTestHelper);
            SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
            framePoint.get(expectedTrajectoryPoint);

            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
         }
      }

      success = simulationTestHelper.simulateNow(trajectoryTime + firstTrajectoryPointTime + 0.5);
      assertTrue(success);
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = fullRobotModel.getHand(robotSide).getName();
         FrameSE3TrajectoryPoint framePoint = lastTrajectoryPoints.get(robotSide);
         framePoint.changeFrame(worldFrame);

         SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(handName,
                                                                                                                                 simulationTestHelper);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         framePoint.get(expectedTrajectoryPoint);

         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime());
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
      }

      assertEquals(2, statusMessages.size());

      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = fullRobotModel.getHand(robotSide).getName();
         TaskspaceTrajectoryStatusMessage statusMessage = statusMessages.stream().filter(m -> m.getEndEffectorName().toString().equals(handName)).findFirst()
                                                                        .get();
         HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide);
         SE3TrajectoryPointMessage lastTrajectoryPoint = handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast();
         double expectedTimestamp = lastTrajectoryPoint.getTime();
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(handTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           expectedTimestamp,
                                                           lastTrajectoryPoint.getPosition(),
                                                           lastTrajectoryPoint.getOrientation(),
                                                           handName,
                                                           statusMessage,
                                                           1.0e-12,
                                                           controllerDT);
      }

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RobotSide robotSide = RobotSide.LEFT;
      String handName = fullRobotModel.getHand(robotSide).getName();

      {
         int numberOfPoints = RigidBodyTaskspaceControlState.maxPoints;
         HandTrajectoryMessage message = new HandTrajectoryMessage();
         message.setRobotSide(robotSide.toByte());
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         double time = 0.05 + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            message.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                   .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
            time = time + 0.05;
         }
         simulationTestHelper.publishToController(message);

         success = simulationTestHelper.simulateNow(4.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper);
         assertTrue(controllerState == RigidBodyControlMode.JOINTSPACE);
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(handName, 0, simulationTestHelper);
      }

      {
         int numberOfPoints = RigidBodyTaskspaceControlState.maxPoints - 1;
         HandTrajectoryMessage message = new HandTrajectoryMessage();
         message.setRobotSide(robotSide.toByte());
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         double time = 0.05 + RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint;
         for (int pointIdx = 0; pointIdx < numberOfPoints; pointIdx++)
         {
            message.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                   .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
            time = time + 0.05;
         }
         simulationTestHelper.publishToController(message);

         success = simulationTestHelper.simulateNow(4.0 * getRobotModel().getControllerDT());
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper);
         assertTrue(controllerState == RigidBodyControlMode.TASKSPACE);
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(handName, RigidBodyTaskspaceControlState.maxPoints, simulationTestHelper);
      }
   }

   @Test
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 1.0;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double timeDurationForBetweenWaypoints = 0.1;

      ArrayList<HandTrajectoryMessage> handTrajectoryMessages = new ArrayList<>();
      ArrayDeque<FrameSE3TrajectoryPoint> handTrajectoryPoints = new ArrayDeque<>();

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

      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
      {
         if (robotSide == RobotSide.RIGHT)
            pointsOnSphere[i].negate();
         tempPoint.setIncludingFrame(chestBodyFixedFrame, pointsOnSphere[i]);
         tempPoint.add(sphereCenter);
         tempPoint.changeFrame(worldFrame);

         euclideanTrajectoryPointCalculator.appendTrajectoryPoint(timeDurationForBetweenWaypoints * i, tempPoint);
      }

      euclideanTrajectoryPointCalculator.compute(timeDurationForBetweenWaypoints * (numberOfTrajectoryPoints * numberOfMessages - 1));

      FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
      trajectoryPoints.addTimeOffset(firstTrajectoryPointTime);

      int calculatorIndex = 0;
      long id = 4678L;

      double controllerDT = getRobotModel().getControllerDT();
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
         double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.getTrajectoryPoint(calculatorIndex - 1).getTime();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion(tempOrientation);
            Vector3D desiredAngularVelocity = new Vector3D();

            trajectoryPoints.getTrajectoryPoint(calculatorIndex).get(desiredPosition, desiredLinearVelocity);
            double time = trajectoryPoints.getTrajectoryPoint(calculatorIndex).getTime();

            VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
            visualDefinitionFactory.appendTranslation(desiredPosition);
            visualDefinitionFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
            simulationTestHelper.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());

            handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time
                  - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));

            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(worldFrame);
            framePoint.set(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            framePoint.changeFrame(chestBodyFixedFrame);
            handTrajectoryPoints.addLast(framePoint);

            calculatorIndex++;
         }

         handTrajectoryMessages.add(handTrajectoryMessage);
         simulationTestHelper.publishToController(handTrajectoryMessage);
         success = simulationTestHelper.simulateNow(controllerDT);
         assertTrue(success);
      }

      success = simulationTestHelper.simulateNow(controllerDT);
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
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(handName, totalNumberOfPoints, simulationTestHelper);

         double lastPointTime = 0.0;
         fullRobotModel.updateFrames();

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            FrameSE3TrajectoryPoint framePoint = handTrajectoryPoints.removeFirst();

            SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findSE3TrajectoryPoint(handName, trajectoryPointIndex, simulationTestHelper);
            SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
            framePoint.get(expectedTrajectoryPoint);
            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));

            lastPointTime = Math.max(framePoint.getTime(), lastPointTime);
            lastPoint.setIncludingFrame(framePoint);
         }

         success = simulationTestHelper.simulateNow(lastPointTime - timeOffset);
         assertTrue(success);

         timeOffset = lastPointTime;
         totalNumberOfPoints = totalNumberOfPoints - (expectedNumberOfPointsInGenerator - 1);
         firstSegment = false;

         if (expectedPointsInQueue == 0)
            break;
      }

      success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      // check internal tracking is decent:
      String namespaceRotation = FeedbackControllerToolbox.class.getSimpleName();
      String varnameRotation = handName + "ErrorRotationVector";
      Vector3D rotationError = EndToEndTestTools.findVector3D(namespaceRotation, varnameRotation, simulationTestHelper);

      String namespacePosition = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePosition = handName + "ErrorPosition";
      Vector3D positionError = EndToEndTestTools.findVector3D(namespacePosition, varnamePosition, simulationTestHelper);

      assertTrue(rotationError.length() < Math.toRadians(15.0));
      assertTrue(positionError.length() < 0.05);

      // check internal desired matches last trajectory point:
      String namespacePositionDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePositionDesired = handName + Type.DESIRED.getName() + SpaceData3D.POSITION.getName();
      Vector3D desiredPosition = EndToEndTestTools.findVector3D(namespacePositionDesired, varnamePositionDesired, simulationTestHelper);

      String namespaceOrientationDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnameOrientationDesired = handName + Type.DESIRED.getName() + SpaceData3D.ORIENTATION.getName();
      Quaternion desiredOrientation = EndToEndTestTools.findQuaternion(namespaceOrientationDesired, varnameOrientationDesired, simulationTestHelper);

      lastPoint.changeFrame(worldFrame);
      EuclidCoreTestTools.assertTuple3DEquals(lastPoint.getPositionCopy(), desiredPosition, 0.001);
      EuclidCoreTestTools.assertQuaternionEquals(lastPoint.getOrientationCopy(), desiredOrientation, 0.001);

      assertEquals(2 * handTrajectoryMessages.size(), statusMessages.size());
      double startTime = 0.0;

      for (int inputIndex = 0; inputIndex < handTrajectoryMessages.size(); inputIndex++)
      {
         HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(inputIndex);
         Object<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints();

         double endTime = startTime + taskspaceTrajectoryPoints.getLast().getTime();
         if (inputIndex > 0)
            startTime += taskspaceTrajectoryPoints.getFirst().getTime();

         TaskspaceTrajectoryStatusMessage startedStatus = statusMessages.remove(0);
         TaskspaceTrajectoryStatusMessage completedStatus = statusMessages.remove(0);
         long expectedSequenceID = handTrajectoryMessage.getSequenceId();
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                                           TrajectoryExecutionStatus.STARTED,
                                                           startTime,
                                                           handName,
                                                           startedStatus,
                                                           controllerDT);
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           endTime,
                                                           handName,
                                                           completedStatus,
                                                           controllerDT);
         startTime = endTime;
      }

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double timeDurationForBetweenWaypoints = 0.1;

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;

      SideDependentList<ArrayList<HandTrajectoryMessage>> handTrajectoryMessages = new SideDependentList<>(new ArrayList<HandTrajectoryMessage>(),
                                                                                                           new ArrayList<HandTrajectoryMessage>());

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint3D sphereCenter = new FramePoint3D(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         sphereCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint3D tempPoint = new FramePoint3D();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel,
                                                                                    robotSide,
                                                                                    sphereCenter,
                                                                                    taskspaceToJointspaceCalculator,
                                                                                    500);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(worldFrame);
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(i * timeDurationForBetweenWaypoints, tempPoint);
         }

         euclideanTrajectoryPointCalculator.compute(timeDurationForBetweenWaypoints * (numberOfTrajectoryPoints * numberOfMessages - 1));

         FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
         trajectoryPoints.addTimeOffset(firstTrajectoryPointTime);

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
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.getTrajectoryPoint(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3D desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               Quaternion desiredOrientation = new Quaternion(tempOrientation);
               Vector3D desiredAngularVelocity = new Vector3D();

               trajectoryPoints.getTrajectoryPoint(calculatorIndex).get(desiredPosition, desiredLinearVelocity);
               double time = trajectoryPoints.getTrajectoryPoint(calculatorIndex).getTime();

               VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
               visualDefinitionFactory.appendTranslation(desiredPosition);
               visualDefinitionFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
               simulationTestHelper.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());

               handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time
                     - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));
               calculatorIndex++;
            }

            handTrajectoryMessages.get(robotSide).add(handTrajectoryMessage);
            simulationTestHelper.publishToController(handTrajectoryMessage);
            success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = simulationTestHelper.simulateNow(0.05 + getRobotModel().getControllerDT());
      assertTrue(success);

      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = fullRobotModel.getHand(robotSide).getName();
         RigidBodyControlMode controllerState = EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper);
         assertTrue(controllerState == RigidBodyControlMode.JOINTSPACE);
      }
   }

   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             new FlatGroundEnvironment(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      simulationTestHelper.setCamera(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 0.0, 0.5));

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 10;
      int numberOfMessages = 10;
      double timeDurationForBetweenWaypoints = 0.1;

      // This test was originally made for Atlas, a robot with a leg length of ~0.8m
      double scale = getLegLength() / 0.8;

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();

         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         FramePoint3D sphereCenter = new FramePoint3D(chestFrame);
         sphereCenter.set(0.35, robotSide.negateIfRightSide(0.45), -0.45);
         sphereCenter.scale(scale);
         double radius = 0.15 * scale;
         FramePoint3D tempPoint = new FramePoint3D();
         TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = createTaskspaceToJointspaceCalculator(fullRobotModel, robotSide);
         FrameQuaternion tempOrientation = computeBestOrientationForDesiredPosition(fullRobotModel,
                                                                                    robotSide,
                                                                                    sphereCenter,
                                                                                    taskspaceToJointspaceCalculator,
                                                                                    500);

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(radius, numberOfTrajectoryPoints * numberOfMessages);

         for (int i = 0; i < numberOfTrajectoryPoints * numberOfMessages; i++)
         {
            if (robotSide == RobotSide.RIGHT)
               pointsOnSphere[i].negate();
            tempPoint.setIncludingFrame(chestFrame, pointsOnSphere[i]);
            tempPoint.add(sphereCenter);
            tempPoint.changeFrame(worldFrame);
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(i * timeDurationForBetweenWaypoints, tempPoint);
         }

         euclideanTrajectoryPointCalculator.compute(timeDurationForBetweenWaypoints * (numberOfTrajectoryPoints * numberOfMessages - 1));
         FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
         trajectoryPoints.addTimeOffset(firstTrajectoryPointTime);

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
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.getTrajectoryPoint(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               Point3D desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               Quaternion desiredOrientation = new Quaternion(tempOrientation);
               Vector3D desiredAngularVelocity = new Vector3D();

               trajectoryPoints.getTrajectoryPoint(calculatorIndex).get(desiredPosition, desiredLinearVelocity);
               double time = trajectoryPoints.getTrajectoryPoint(calculatorIndex).getTime();

               VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
               visualDefinitionFactory.appendTranslation(desiredPosition);
               visualDefinitionFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
               simulationTestHelper.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());

               handTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time
                     - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));
               calculatorIndex++;
            }

            simulationTestHelper.publishToController(handTrajectoryMessage);
            success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT());
            assertTrue(success);
         }
      }

      success = simulationTestHelper.simulateNow(0.1);
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
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                        overrideTrajectoryTime,
                                                                                                        desiredPosition,
                                                                                                        desiredOrientation,
                                                                                                        chestFrame);
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

         simulationTestHelper.publishToController(handTrajectoryMessage);
         overridingPoses.put(robotSide, desiredHandPose);
      }

      success = simulationTestHelper.simulateNow(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         overridingPoses.get(robotSide).changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
         String handName = fullRobotModel.getHand(robotSide).getName();
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(handName, 2, simulationTestHelper);
      }

      success = simulationTestHelper.simulateNow(overrideTrajectoryTime + 1.0);
      assertTrue(success);
      fullRobotModel.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D desiredPose = overridingPoses.get(robotSide);
         desiredPose.changeFrame(worldFrame);

         String handName = fullRobotModel.getHand(robotSide).getName();
         SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(handName,
                                                                                                                                 simulationTestHelper);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         expectedTrajectoryPoint.setPosition(desiredPose.getPosition());
         expectedTrajectoryPoint.setOrientation(desiredPose.getOrientation());

         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
      }
   }

   public static FrameQuaternion computeBestOrientationForDesiredPosition(FullHumanoidRobotModel fullRobotModel,
                                                                          RobotSide robotSide,
                                                                          FramePoint3DReadOnly desiredPosition,
                                                                          TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator,
                                                                          int numberOfIterations)
   {
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      ReferenceFrame handFrame = hand.getBodyFixedFrame();
      Twist desiredTwist = new Twist(handFrame, chestFrame, handControlFrame);
      FramePose3D desiredPose = new FramePose3D(desiredPosition.getReferenceFrame());
      desiredPose.getPosition().set(desiredPosition);
      desiredPose.changeFrame(chestFrame);
      for (int i = 0; i < numberOfIterations; i++)
         taskspaceToJointspaceCalculator.compute(desiredPose, desiredTwist);
      taskspaceToJointspaceCalculator.getDesiredEndEffectorPoseFromQDesireds(desiredPose, ReferenceFrame.getWorldFrame());

      FrameQuaternion tempOrientation = new FrameQuaternion(desiredPose.getOrientation());
      return tempOrientation;
   }

   public static TaskspaceToJointspaceCalculator createTaskspaceToJointspaceCalculator(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide)
   {
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(robotSide);
      TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator = new TaskspaceToJointspaceCalculator("blop",
                                                                                                            chest,
                                                                                                            hand,
                                                                                                            0.005,
                                                                                                            new YoRegistry("Dummy"));
      taskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
      taskspaceToJointspaceCalculator.setupWithDefaultParameters();
      DMatrixRMaj selectionMatrix = CommonOps_DDRM.identity(6);
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

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 5.0;
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         String handName = fullRobotModel.getHand(robotSide).getName();

         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);

         FramePose3D desiredRandomHandPose = new FramePose3D(fullRobotModel.getHandControlFrame(robotSide));
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredRandomHandPose.prependTranslation(RandomGeometry.nextVector3D(random, 0.2));

         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         ReferenceFrame chestFrame = chest.getBodyFixedFrame();
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                        trajectoryTime,
                                                                                                        desiredPosition,
                                                                                                        desiredOrientation,
                                                                                                        chestFrame);
         handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));

         simulationTestHelper.publishToController(handTrajectoryMessage);

         success = simulationTestHelper.simulateNow(trajectoryTime / 2.0);
         assertTrue(success);

         RigidBodyControlMode controllerState = EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper);
         assertEquals(RigidBodyControlMode.TASKSPACE, controllerState);

         int numberOfJoints = armJoints.length;
         double[] actualJointPositions = new double[numberOfJoints];
         double[] zeroVelocities = new double[numberOfJoints];
         for (int i = 0; i < numberOfJoints; i++)
         {
            actualJointPositions[i] = armJoints[i].getQ();
         }

         simulationTestHelper.publishToController(new StopAllTrajectoryMessage());

         success = simulationTestHelper.simulateNow(0.05);
         assertTrue(success);

         controllerState = EndToEndTestTools.findRigidBodyControlManagerState(handName, simulationTestHelper);
         double[] controllerDesiredJointPositions = EndToEndArmTrajectoryMessageTest.findControllerDesiredPositions(armJoints, simulationTestHelper);
         double[] controllerDesiredJointVelocities = EndToEndArmTrajectoryMessageTest.findControllerDesiredVelocities(armJoints, simulationTestHelper);

         assertEquals(RigidBodyControlMode.JOINTSPACE, controllerState);
         assertArrayEquals(actualJointPositions, controllerDesiredJointPositions, 0.01);
         assertArrayEquals(zeroVelocities, controllerDesiredJointVelocities, 1.0e-10);
      }
   }

   public void testHoldHandWhileWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      boolean success = simulationTestHelper.simulateNow(1.5);
      assertTrue(success);

      PrepareForLocomotionMessage prepareForLocomotionMessage = new PrepareForLocomotionMessage();
      prepareForLocomotionMessage.setPrepareManipulation(false);
      simulationTestHelper.publishToController(prepareForLocomotionMessage);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RobotSide controlledSide = RobotSide.RIGHT;
      ReferenceFrame rightHandControlFrame = fullRobotModel.getHandControlFrame(controlledSide);

      FramePose3D poseToHold = new FramePose3D(rightHandControlFrame);
      poseToHold.changeFrame(ReferenceFrame.getWorldFrame());
      HandTrajectoryMessage message = HumanoidMessageTools.createHandTrajectoryMessage(controlledSide,
                                                                                       0.5,
                                                                                       poseToHold.getPosition(),
                                                                                       poseToHold.getOrientation(),
                                                                                       ReferenceFrame.getWorldFrame());
      simulationTestHelper.publishToController(message);

      assertTrue(simulationTestHelper.simulateNow(1.0));

      FootstepDataListMessage twoStepsInPlace = twoStepsInPlace(fullRobotModel.getSoleFrames());
      simulationTestHelper.publishToController(twoStepsInPlace);

      double walkingDuration = computeWalkingDuration(twoStepsInPlace, getRobotModel().getWalkingControllerParameters());

      int numberOfAssertions = 50;

      for (int i = 0; i < numberOfAssertions; i++)
      {
         assertTrue(simulationTestHelper.simulateNow(walkingDuration / numberOfAssertions));

         FramePose3D currentHandPose = new FramePose3D(rightHandControlFrame);
         currentHandPose.changeFrame(ReferenceFrame.getWorldFrame());
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(poseToHold.getPosition(), currentHandPose.getPosition(), 1.0e-2);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(poseToHold.getOrientation(), currentHandPose.getOrientation(), Math.toRadians(10.0));
      }
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(595161);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry testRegistry = new YoRegistry("testStreaming");

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(), simulationTestingParameters)
                                                               .createAvatarTestingSimulation();
      simulationTestHelper.start();
      simulationTestHelper.addRegistry(testRegistry);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics chestCloned = MultiBodySystemFactories.cloneSubtree(chest, "Cloned");

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      startTime.set(simulationTestHelper.getControllerTime());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      SideDependentList<YoFramePose3D> initialPoses = new SideDependentList<>();
      SideDependentList<YoFramePose3D> finalPoses = new SideDependentList<>();
      SideDependentList<YoFramePose3D> desiredPoses = new SideDependentList<>();
      SideDependentList<YoFixedFrameSpatialVector> desiredVelocities = new SideDependentList<>();

      SubtreeStreams.fromChildren(OneDoFJointBasics.class, chestCloned).forEach(joint -> joint.setQ(nextJointConfiguration(random, 0.6, joint)));
      chestCloned.updateFramesRecursively();

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = robotSide.getCamelCaseName();
         YoFramePose3D initialPose = new YoFramePose3D(prefix + "HandInitialOrientation", worldFrame, testRegistry);
         YoFramePose3D finalPose = new YoFramePose3D(prefix + "HandFinalOrientation", worldFrame, testRegistry);
         YoFramePose3D desiredPose = new YoFramePose3D(prefix + "HandDesiredOrientation", worldFrame, testRegistry);
         YoFixedFrameSpatialVector desiredVelocity = new YoFixedFrameSpatialVector(prefix + "HandDesiredAngularVelocity", worldFrame, testRegistry);
         initialPoses.put(robotSide, initialPose);
         finalPoses.put(robotSide, finalPose);
         desiredPoses.put(robotSide, desiredPose);
         desiredVelocities.put(robotSide, desiredVelocity);

         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         initialPose.setFromReferenceFrame(hand.getBodyFixedFrame());
         finalPose.setFromReferenceFrame(chestCloned.subtreeStream().filter(body -> body.getName().equals(hand.getName() + "Cloned")).findFirst().get()
                                                    .getBodyFixedFrame());
      }

      simulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         private boolean everyOtherTick = false;
         private final OrientationInterpolationCalculator calculator = new OrientationInterpolationCalculator();

         @Override
         public void doControl()
         {
            everyOtherTick = !everyOtherTick;

            if (!everyOtherTick)
               return;

            double timeInTrajectory = simulationTestHelper.getControllerTime() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            for (RobotSide robotSide : RobotSide.values)
            {
               desiredPoses.get(robotSide).interpolate(initialPoses.get(robotSide), finalPoses.get(robotSide), alpha);

               if (alpha <= 0.0 || alpha >= 1.0)
               {
                  desiredVelocities.get(robotSide).setToZero();
               }
               else
               {
                  calculator.computeAngularVelocity(desiredVelocities.get(robotSide).getAngularPart(),
                                                    initialPoses.get(robotSide).getOrientation(),
                                                    finalPoses.get(robotSide).getOrientation(),
                                                    1.0 / trajectoryTime.getValue());
                  desiredVelocities.get(robotSide).getLinearPart().sub(finalPoses.get(robotSide).getPosition(), initialPoses.get(robotSide).getPosition());
                  desiredVelocities.get(robotSide).getLinearPart().scale(1.0 / trajectoryTime.getValue());
               }

               HandTrajectoryMessage message = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                0.0,
                                                                                                desiredPoses.get(robotSide),
                                                                                                desiredVelocities.get(robotSide),
                                                                                                worldFrame);
               message.getSe3Trajectory().setUseCustomControlFrame(true); // This is to force the controller to use the body-fixed frame
               message.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
               message.getSe3Trajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
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

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue());
      assertTrue(success);

      double desiredEpsilon = 6.0e-3;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         SE3TrajectoryPoint currentDesiredTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(hand.getName(),
                                                                                                                                     simulationTestHelper);
         Pose3D controllerDesiredPose = new Pose3D(currentDesiredTrajectoryPoint.getPosition(), currentDesiredTrajectoryPoint.getOrientation());
         SpatialVector controllerDesiredVelocity = new SpatialVector(ReferenceFrame.getWorldFrame(),
                                                                     currentDesiredTrajectoryPoint.getAngularVelocity(),
                                                                     currentDesiredTrajectoryPoint.getLinearVelocity());

         EuclidGeometryTestTools.assertPose3DEquals(desiredPoses.get(robotSide), controllerDesiredPose, desiredEpsilon);
         MecanoTestTools.assertSpatialVectorEquals(desiredVelocities.get(robotSide), controllerDesiredVelocity, desiredEpsilon);

         FramePose3D currentPose = new FramePose3D(hand.getBodyFixedFrame());
         currentPose.changeFrame(worldFrame);
         EuclidGeometryTestTools.assertPose3DGeometricallyEquals("Poor tracking for side: " + robotSide + " position: "
               + currentPose.getPosition().distance(controllerDesiredPose.getPosition()) + ", orientation: "
               + Math.abs(AngleTools.trimAngleMinusPiToPi(currentPose.getOrientation().distance(controllerDesiredPose.getOrientation()))),
                                                                 controllerDesiredPose,
                                                                 currentPose,
                                                                 0.1);
      }

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);

      desiredEpsilon = 1.0e-7;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         SE3TrajectoryPoint currentDesiredTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(hand.getName(),
                                                                                                                                     simulationTestHelper);
         Pose3D controllerDesiredPose = new Pose3D(currentDesiredTrajectoryPoint.getPosition(), currentDesiredTrajectoryPoint.getOrientation());
         SpatialVector controllerDesiredVelocity = new SpatialVector(ReferenceFrame.getWorldFrame(),
                                                                     currentDesiredTrajectoryPoint.getAngularVelocity(),
                                                                     currentDesiredTrajectoryPoint.getLinearVelocity());

         EuclidGeometryTestTools.assertPose3DEquals(desiredPoses.get(robotSide), controllerDesiredPose, desiredEpsilon);
         MecanoTestTools.assertSpatialVectorEquals(desiredVelocities.get(robotSide), controllerDesiredVelocity, desiredEpsilon);

         FramePose3D currentPose = new FramePose3D(hand.getBodyFixedFrame());
         currentPose.changeFrame(worldFrame);
         EuclidCoreTestTools.assertTuple3DEquals("Poor position tracking for side: " + robotSide + " error: "
               + currentPose.getPosition().distance(controllerDesiredPose.getPosition()),
                                                 controllerDesiredPose.getPosition(),
                                                 currentPose.getPosition(),
                                                 3.0e-2);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals("Poor orientation tracking for side: " + robotSide + " error: "
               + Math.abs(AngleTools.trimAngleMinusPiToPi(currentPose.getOrientation().distance(controllerDesiredPose.getOrientation()))),
                                                                 controllerDesiredPose.getOrientation(),
                                                                 currentPose.getOrientation(),
                                                                 0.3);
      }
   }

   public static double nextJointConfiguration(Random random, double percentOfMotionRangeAllowed, OneDoFJointReadOnly joint)
   {
      double jointLimitLower = joint.getJointLimitLower();
      if (Double.isInfinite(jointLimitLower))
         jointLimitLower = -Math.PI;
      double jointLimitUpper = joint.getJointLimitUpper();
      if (Double.isInfinite(jointLimitUpper))
         jointLimitUpper = -Math.PI;
      double rangeReduction = (1.0 - percentOfMotionRangeAllowed) * (jointLimitUpper - jointLimitLower);
      jointLimitLower += 0.5 * rangeReduction;
      jointLimitUpper -= 0.5 * rangeReduction;
      return RandomNumbers.nextDouble(random, jointLimitLower, jointLimitUpper);
   }

   public static void assertSingleWaypointExecuted(String bodyName, Pose3DReadOnly desiredPose, YoVariableHolder scs)
   {
      assertSingleWaypointExecuted(bodyName, desiredPose.getPosition(), desiredPose.getOrientation(), scs);
   }

   public static void assertSingleWaypointExecuted(String bodyName,
                                                   Point3DReadOnly desiredPosition,
                                                   QuaternionReadOnly desiredOrientation,
                                                   YoVariableHolder scs)
   {
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(bodyName, 2, scs);

      Point3DReadOnly controllerDesiredPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(bodyName, scs);
      EuclidCoreTestTools.assertTuple3DEquals(desiredPosition, controllerDesiredPosition, EPSILON_FOR_DESIREDS);

      QuaternionReadOnly controllerDesiredOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(bodyName, scs);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, controllerDesiredOrientation, EPSILON_FOR_DESIREDS);
   }

   public static FootstepDataListMessage twoStepsInPlace(SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      List<FootstepDataMessage> footstepDataList = new ArrayList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = soleFrames.get(robotSide);
         FramePose3D footPose = new FramePose3D(soleFrame);
         footPose.changeFrame(ReferenceFrame.getWorldFrame());
         footstepDataList.add(HumanoidMessageTools.createFootstepDataMessage(robotSide, footPose));
      }

      return HumanoidMessageTools.createFootstepDataListMessage(footstepDataList, -1.0);
   }

   public static double computeWalkingDuration(FootstepDataListMessage message, WalkingControllerParameters parameters)
   {
      double walkingDuration = 0.0;

      Object<FootstepDataMessage> footsteps = message.getFootstepDataList();

      if (footsteps.isEmpty())
         return walkingDuration;

      double defaultSwingTime = selectDefaultIfCustomInvalid(message.getDefaultSwingDuration(), parameters.getDefaultSwingTime());
      double defaultTransferTime = selectDefaultIfCustomInvalid(message.getDefaultTransferDuration(), parameters.getDefaultTransferTime());
      FootstepDataMessage initialFootstep = footsteps.get(0);

      walkingDuration += computeStepDuration(initialFootstep, parameters.getDefaultInitialTransferTime(), defaultSwingTime);

      for (int i = 1; i < footsteps.size(); i++)
      {
         walkingDuration += computeStepDuration(footsteps.get(i), defaultTransferTime, defaultSwingTime);
      }

      walkingDuration += selectDefaultIfCustomInvalid(message.getFinalTransferDuration(), parameters.getDefaultFinalTransferTime());

      return walkingDuration;
   }

   public static double computeStepDuration(FootstepDataMessage message, double defaultTransferDuration, double defaultSwingDuration)
   {
      double stepDuration = selectDefaultIfCustomInvalid(message.getTransferDuration(), defaultTransferDuration);
      stepDuration += selectDefaultIfCustomInvalid(message.getSwingDuration(), defaultSwingDuration);
      return stepDuration;
   }

   private static double selectDefaultIfCustomInvalid(double customValue, double defaultValue)
   {
      if (customValue <= 0.0 || Double.isNaN(customValue))
         return defaultValue;
      else
         return customValue;
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
