package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.robotics.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.jcodec.common.Assert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoVariable;

@Tag("controller-api-2")
public abstract class EndToEndPelvisOrientationTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Vector3D zeroVector = new Vector3D();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private SimulationConstructionSet scs;

   @Test
   public void testGoHome() throws SimulationExceededMaximumTimeException
   {
      double epsilon = 1.0e-4;
      double yaw = Math.toRadians(15.0);
      double trajectoryTime = 0.5;

      Quaternion orientation = new Quaternion();
      orientation.appendYawRotation(yaw);
      ReferenceFrame midFootZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();

      humanoidReferenceFrames.updateFrames();
      FrameQuaternion pelvisOrientation = new FrameQuaternion(midFootZUpGroundFrame, orientation);
      pelvisOrientation.changeFrame(worldFrame);
      PelvisOrientationTrajectoryMessage message = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime, pelvisOrientation);
      drcSimulationTestHelper.publishToController(message);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.25));

      String pelvisName = fullRobotModel.getPelvis().getName();
      EndToEndTestTools.assertCurrentDesiredsMatchWaypoint(pelvisName, message.getSo3Trajectory().getTaskspaceTrajectoryPoints().get(0), epsilon, scs);

      GoHomeMessage goHomeMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.PELVIS, trajectoryTime);
      drcSimulationTestHelper.publishToController(goHomeMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + 0.5));

      humanoidReferenceFrames.updateFrames();
      FrameQuaternion homeOrientation = new FrameQuaternion(midFootZUpGroundFrame, new Quaternion());
      homeOrientation.changeFrame(worldFrame);
      SO3TrajectoryPointMessage home = HumanoidMessageTools.createSO3TrajectoryPointMessage(trajectoryTime, homeOrientation, zeroVector);
      EndToEndTestTools.assertCurrentDesiredsMatchWaypoint(pelvisName, home, epsilon, scs);
   }

   @Test
   public void testSingleTrajectoryPoint() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(346665);
      double controllerDT = getRobotModel().getControllerDT();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      double epsilon = 1.0e-10;
      double yaw = Math.toRadians(5.0);
      double pitch = Math.toRadians(-6.0);
      double roll = Math.toRadians(-5.0);
      double trajectoryTime = 0.5;

      Quaternion orientation = new Quaternion();
      orientation.appendYawRotation(yaw);
      orientation.appendPitchRotation(pitch);
      orientation.appendRollRotation(roll);

      ReferenceFrame midFootZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();
      FrameQuaternion pelvisOrientation = new FrameQuaternion(midFootZUpGroundFrame, orientation);
      pelvisOrientation.changeFrame(worldFrame);

      PelvisOrientationTrajectoryMessage message = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime, pelvisOrientation);
      message.setSequenceId(random.nextLong());
      SO3TrajectoryPointMessage waypoint = message.getSo3Trajectory().getTaskspaceTrajectoryPoints().get(0);
      drcSimulationTestHelper.publishToController(message);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 * controllerDT);

      String pelvisName = fullRobotModel.getPelvis().getName();

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(message.getSequenceId(), TrajectoryExecutionStatus.STARTED, 0.0, pelvisName, statusMessages.remove(0), controllerDT);

      String postFix = "Orientation";
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(pelvisName, postFix, 2, scs);
      EndToEndTestTools.assertWaypointInGeneratorMatches(pelvisName, 1, waypoint, epsilon, scs);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      EndToEndTestTools.assertCurrentDesiredsMatchWaypoint(pelvisName, waypoint, epsilon, scs);

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(message.getSequenceId(), TrajectoryExecutionStatus.COMPLETED, trajectoryTime, null, pelvisOrientation, pelvisName, statusMessages.remove(0), epsilon, controllerDT);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testQueue() throws SimulationExceededMaximumTimeException
   {
      ReferenceFrame midFootZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();
      FrameQuaternion pelvisOrientation = new FrameQuaternion(midFootZUpGroundFrame);
      pelvisOrientation.changeFrame(worldFrame);

      double trajectoryTime = 0.5;
      PelvisOrientationTrajectoryMessage message = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime, pelvisOrientation);

      message.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      message.getSo3Trajectory().getQueueingProperties().setMessageId(1L);

      drcSimulationTestHelper.publishToController(message);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));

      message.getSo3Trajectory().getQueueingProperties().setPreviousMessageId(2L);
      message.getSo3Trajectory().getQueueingProperties().setMessageId(3L);

      drcSimulationTestHelper.publishToController(message);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
   }

   @Test
   public void testWalking() throws SimulationExceededMaximumTimeException
   {
      double epsilon = 1.0e-4;
      int steps = 4;

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      double walkingTime = createWalkingMessage(steps, footsteps, true);
      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingTime + 1.0);

      assertEquals("Control Mode", PelvisOrientationControlMode.WALKING_CONTROLLER, findCurrentControlMode());
      humanoidReferenceFrames.updateFrames();
      ReferenceFrame midFeetZUpFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();
      FrameQuaternion midFeetOrientation = new FrameQuaternion(midFeetZUpFrame);
      midFeetOrientation.changeFrame(worldFrame);
      String pelvisName = fullRobotModel.getPelvis().getName();
      EndToEndTestTools.assertCurrentDesiredsMatch(pelvisName, midFeetOrientation, zeroVector, epsilon, scs);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testWalkingAfterTrajectory() throws SimulationExceededMaximumTimeException
   {
      double epsilon = 3.0e-3;

      drcSimulationTestHelper.getSimulationConstructionSet().getVariable(PelvisOrientationManager.class.getSimpleName(), "doPreparePelvisForLocomotion").setValueFromDouble(0.0);

      assertEquals("Control Mode", PelvisOrientationControlMode.WALKING_CONTROLLER, findCurrentControlMode());
      testSingleTrajectoryPoint();
      assertEquals("Control Mode", PelvisOrientationControlMode.USER, findCurrentControlMode());

      humanoidReferenceFrames.updateFrames();
      ReferenceFrame midFeetZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();

      String pelvisName = fullRobotModel.getPelvis().getName();
      QuaternionReadOnly currentDesired = EndToEndTestTools.findFeedbackControllerDesiredOrientation(pelvisName, scs);
      FrameQuaternion desiredAfterTrajectory = new FrameQuaternion(worldFrame, currentDesired);
      desiredAfterTrajectory.changeFrame(midFeetZUpGroundFrame);

      int steps = 2;
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      double walkingTime = createWalkingMessage(steps, footsteps, false);
      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingTime + 1.5);
      assertEquals("Control Mode", PelvisOrientationControlMode.WALKING_CONTROLLER, findCurrentControlMode());

      humanoidReferenceFrames.updateFrames();
      desiredAfterTrajectory.changeFrame(worldFrame);
      EndToEndTestTools.assertCurrentDesiredsMatch(pelvisName, desiredAfterTrajectory, zeroVector, epsilon, scs);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(159684);
      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      double epsilon = 1.0e-10;
      int numberOfPoints = 23;
      double yawMagnitude = Math.toRadians(5.0);
      double pitchMagnitude = Math.toRadians(-10.0);
      double rollMagnitude = Math.toRadians(-5.0);
      double timePerPoint = 0.1;
      double frequency = 1.0;

      humanoidReferenceFrames.updateFrames();
      ReferenceFrame pelvisFrame = humanoidReferenceFrames.getPelvisFrame();
      FrameQuaternion initialOrientation = new FrameQuaternion(pelvisFrame);
      initialOrientation.changeFrame(worldFrame);

      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage();
      message.setSequenceId(random.nextLong());
      SO3TrajectoryMessage so3Trajectory = message.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));

      for (int point = 0; point < numberOfPoints; point++)
      {
         double time = timePerPoint * (point + 1);
         double factor = Math.sin(2.0 * Math.PI * time * frequency);

         double yaw = yawMagnitude * factor;
         double pitch = pitchMagnitude * factor;
         double roll = rollMagnitude * factor;

         Quaternion orientation = new Quaternion();
         orientation.appendYawRotation(yaw);
         orientation.appendPitchRotation(pitch);
         orientation.appendRollRotation(roll);
         FrameQuaternion frameOrientation = new FrameQuaternion(pelvisFrame, orientation);
         frameOrientation.changeFrame(worldFrame);
         orientation.set(frameOrientation);

         double derivativeFactor = 2.0 * Math.PI * frequency * Math.cos(2.0 * Math.PI * time * frequency);
         double yawRate = yawMagnitude * derivativeFactor;
         double pitchRate = pitchMagnitude * derivativeFactor;
         double rollRate = rollMagnitude * derivativeFactor;
         Vector3D angularVelocity = new Vector3D();
         RotationTools.computeAngularVelocityInBodyFrameFromYawPitchRollAnglesRate(yaw, pitch, roll, yawRate, pitchRate, rollRate, angularVelocity);
         FrameVector3D frameAngularVelcoity = new FrameVector3D(pelvisFrame, angularVelocity);
         frameAngularVelcoity.changeFrame(worldFrame);
         angularVelocity.set(frameAngularVelcoity);

         if (point == numberOfPoints - 1)
            angularVelocity.setToZero();
         SO3TrajectoryPointMessage trajectoryPoint = so3Trajectory.getTaskspaceTrajectoryPoints().add();
         trajectoryPoint.setTime(time);
         trajectoryPoint.getOrientation().set(orientation);
         trajectoryPoint.getAngularVelocity().set(angularVelocity);
      }

      drcSimulationTestHelper.publishToController(message);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * controllerDT);

      String pelvisName = fullRobotModel.getPelvis().getName();
      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(message.getSequenceId(), TrajectoryExecutionStatus.STARTED, 0.0, pelvisName, statusMessages.remove(0), controllerDT);

      String postFix = "Orientation";
      EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(pelvisName, postFix, numberOfPoints + 1, scs);
      for (int point = 1; point < RigidBodyTaskspaceControlState.maxPointsInGenerator; point++)
      {
         SO3TrajectoryPointMessage waypoint = so3Trajectory.getTaskspaceTrajectoryPoints().get(point - 1);
         EndToEndTestTools.assertWaypointInGeneratorMatches(pelvisName, point, waypoint, epsilon, scs);
      }

      double simulationTime = timePerPoint * numberOfPoints + 0.5;
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
      SO3TrajectoryPointMessage waypoint = so3Trajectory.getTaskspaceTrajectoryPoints().get(numberOfPoints - 1);
      EndToEndTestTools.assertCurrentDesiredsMatchWaypoint(pelvisName, waypoint, epsilon, scs);

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(message.getSequenceId(), TrajectoryExecutionStatus.COMPLETED, timePerPoint * numberOfPoints, pelvisName, statusMessages.remove(0), controllerDT);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testWalkingWithUserControl() throws SimulationExceededMaximumTimeException
   {
      double trajectoryTime = 0.5;
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame midFootZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();

      PelvisOrientationTrajectoryMessage message = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(trajectoryTime, desiredOrientation, midFootZUpGroundFrame);
      message.setEnableUserPelvisControlDuringWalking(true);

      assertEquals("Control Mode", PelvisOrientationControlMode.WALKING_CONTROLLER, findCurrentControlMode());
      drcSimulationTestHelper.publishToController(message);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertEquals("Control Mode", PelvisOrientationControlMode.USER, findCurrentControlMode());

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      double walkingTime = createCircularWalkingMessage(8, footsteps, true);
      drcSimulationTestHelper.publishToController(footsteps);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingTime / 2.0);
      assertEquals("Control Mode", PelvisOrientationControlMode.USER, findCurrentControlMode());
   }

   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      double pitch = Math.toRadians(20.0);
      double chestTrajectoryTime = 1.0;
      double epsilon = 2.0E-3;
      Quaternion desiredOrientation = new Quaternion();
      humanoidReferenceFrames.updateFrames();
      ReferenceFrame chestFrame = humanoidReferenceFrames.getChestFrame();

      // first hold the chest in world to avoid feedback effects
      FrameQuaternion chestOrientation = new FrameQuaternion(chestFrame);
      chestOrientation.changeFrame(worldFrame);
      ChestTrajectoryMessage holdChestInWorldMessage = HumanoidMessageTools.createChestTrajectoryMessage(0.0, chestOrientation, worldFrame, worldFrame);
      drcSimulationTestHelper.publishToController(holdChestInWorldMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());

      // now hold the pelvis in chest frame
      PelvisOrientationTrajectoryMessage holdPelvisInChestMessage = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(0.0, desiredOrientation, chestFrame);
      drcSimulationTestHelper.publishToController(holdPelvisInChestMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());

      // finally pitch the chest forward and assert that the pelvis follows
      humanoidReferenceFrames.updateFrames();
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(pitch);
      FrameQuaternion frameChestOrientation = new FrameQuaternion(chestFrame, desiredChestOrientation);
      frameChestOrientation.changeFrame(worldFrame);
      desiredChestOrientation.set(frameChestOrientation);
      ChestTrajectoryMessage chestMessage = HumanoidMessageTools.createChestTrajectoryMessage(chestTrajectoryTime, desiredChestOrientation, worldFrame, worldFrame);
      drcSimulationTestHelper.publishToController(chestMessage);
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(chestTrajectoryTime + 1.0);

      String pelvisName = fullRobotModel.getPelvis().getName();
      EndToEndTestTools.assertCurrentDesiredsMatch(pelvisName, desiredChestOrientation, zeroVector, epsilon, scs);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(54651);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoVariableRegistry testRegistry = new YoVariableRegistry("testStreaming");

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoVariableRegistry(testRegistry);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox()
                                               .getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      YoFrameQuaternion initialOrientation = new YoFrameQuaternion("pelvisInitialOrientation", worldFrame, testRegistry);
      YoFrameQuaternion finalOrientation = new YoFrameQuaternion(  "pelvisFinalOrientation", worldFrame, testRegistry);
      YoFrameQuaternion desiredOrientation = new YoFrameQuaternion("pelvisDesiredOrientation", worldFrame, testRegistry);
      YoFrameVector3D desiredAngularVelocity = new YoFrameVector3D("pelvisDesiredAngularVelocity", worldFrame, testRegistry);

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      initialOrientation.setFromReferenceFrame(pelvis.getBodyFixedFrame());
      finalOrientation.setFromReferenceFrame(pelvis.getBodyFixedFrame());
      finalOrientation.append(EuclidCoreRandomTools.nextQuaternion(random, 0.3));

      drcSimulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
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

            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            desiredOrientation.interpolate(initialOrientation, finalOrientation, alpha);
            if (alpha <= 0.0 || alpha >= 1.0)
               desiredAngularVelocity.setToZero();
            else
               calculator.computeAngularVelocity(desiredAngularVelocity, initialOrientation, finalOrientation, 1.0 / trajectoryTime.getValue());
            PelvisOrientationTrajectoryMessage message = HumanoidMessageTools.createPelvisOrientationTrajectoryMessage(0.0, desiredOrientation, desiredAngularVelocity, worldFrame);
            message.getSo3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
            message.getSo3Trajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
            drcSimulationTestHelper.publishToController(message);
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
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

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 * trajectoryTime.getValue());
      assertTrue(success);

      SO3TrajectoryPoint currentDesiredTrajectoryPoint = EndToEndChestTrajectoryMessageTest.findCurrentDesiredTrajectoryPoint(scs, pelvis);
      double desiredEpsilon = 6.0e-3;

      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, currentDesiredTrajectoryPoint.getOrientation(), desiredEpsilon);
      EuclidCoreTestTools.assertTuple3DEquals(desiredAngularVelocity, currentDesiredTrajectoryPoint.getAngularVelocity(), desiredEpsilon);
      EndToEndChestTrajectoryMessageTest.assertControlErrorIsLow(scs, pelvis, 1.0e-2);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);
      
      currentDesiredTrajectoryPoint = EndToEndChestTrajectoryMessageTest.findCurrentDesiredTrajectoryPoint(scs, pelvis);
      desiredEpsilon = 1.0e-7;
      
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(desiredOrientation, currentDesiredTrajectoryPoint.getOrientation(), desiredEpsilon);
      EuclidCoreTestTools.assertTuple3DEquals(desiredAngularVelocity, currentDesiredTrajectoryPoint.getAngularVelocity(), desiredEpsilon);
      EndToEndChestTrajectoryMessageTest.assertControlErrorIsLow(scs, pelvis, 1.0e-3);
   }

   @SuppressWarnings("unchecked")
   private PelvisOrientationControlMode findCurrentControlMode()
   {
      String managerName = PelvisOrientationManager.class.getSimpleName();
      YoVariable<?> variable = scs.getVariable(managerName, managerName + "CurrentState");
      return ((YoEnum<PelvisOrientationControlMode>) variable).getEnumValue();
   }

   private double createCircularWalkingMessage(int steps, FootstepDataListMessage messageToPack, boolean squareUp)
   {
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingDuration = walkingControllerParameters.getDefaultSwingTime();
      double transferDuration = walkingControllerParameters.getDefaultTransferTime();
      double stepWidth = walkingControllerParameters.getSteppingParameters().getDefaultStepLength() / 2.0;
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame midFootZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();
      double time = walkingControllerParameters.getDefaultInitialTransferTime();
      messageToPack.getFootstepDataList().clear();
      messageToPack.setDefaultSwingDuration(swingDuration);
      messageToPack.setDefaultTransferDuration(transferDuration);
      for (int step = 0; step < steps; step++)
      {
         double yaw = Math.toRadians(25.0) * (step + 1);
         if (squareUp && step == steps - 1)
            yaw = Math.toRadians(25.0) * step;
         FramePoint3D location = new FramePoint3D(midFootZUpGroundFrame);
         location.setY(robotSide.negateIfRightSide(Math.cos(yaw) * stepWidth / 2.0));
         location.setX(robotSide.negateIfRightSide(-Math.sin(yaw) * stepWidth / 2.0));
         location.changeFrame(worldFrame);
         FrameQuaternion orientation = new FrameQuaternion(midFootZUpGroundFrame);
         orientation.appendYawRotation(yaw);
         orientation.changeFrame(worldFrame);
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         messageToPack.getFootstepDataList().add().set(footstep);
         robotSide = robotSide.getOppositeSide();
         time += swingDuration + transferDuration;
      }
      return time;
   }

   private double createWalkingMessage(int steps, FootstepDataListMessage messageToPack, boolean squareUp)
   {
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingDuration = walkingControllerParameters.getDefaultSwingTime();
      double transferDuration = walkingControllerParameters.getDefaultTransferTime();
      double stepLength = 0.6 * walkingControllerParameters.getSteppingParameters().getDefaultStepLength();
      double stepWidth = walkingControllerParameters.getSteppingParameters().getInPlaceWidth();
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame midFootZUpGroundFrame = humanoidReferenceFrames.getMidFootZUpGroundFrame();
      double time = walkingControllerParameters.getDefaultInitialTransferTime();
      messageToPack.getFootstepDataList().clear();
      messageToPack.setDefaultSwingDuration(swingDuration);
      messageToPack.setDefaultTransferDuration(transferDuration);
      for (int step = 0; step < steps; step++)
      {
         FramePoint3D location = new FramePoint3D(midFootZUpGroundFrame);
         if (squareUp && step == steps - 1)
            location.setX(stepLength * step);
         else
            location.setX(stepLength * (step + 1));
         location.setY(robotSide.negateIfRightSide(stepWidth / 2.0));
         location.changeFrame(worldFrame);
         FrameQuaternion orientation = new FrameQuaternion(midFootZUpGroundFrame);
         orientation.changeFrame(worldFrame);
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         messageToPack.getFootstepDataList().add().set(footstep);
         robotSide = robotSide.getOppositeSide();
         time += swingDuration + transferDuration;
      }
      double finalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      time += (finalTransferTime - transferDuration);
      return time;
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest() throws SimulationExceededMaximumTimeException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.8), new Point3D(-7.0, -9.0, 4.0));
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      scs = drcSimulationTestHelper.getSimulationConstructionSet();

      humanoidReferenceFrames.updateFrames();
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      fullRobotModel = null;
      humanoidReferenceFrames = null;
      scs = null;


      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
