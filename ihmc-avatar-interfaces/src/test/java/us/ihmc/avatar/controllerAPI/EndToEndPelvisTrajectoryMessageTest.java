package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import ihmc_common_msgs.msg.dds.FrameInformation;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.CenterOfMassHeightControlState;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.heightPlanning.HeightOffsetHandler;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public abstract class EndToEndPelvisTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double EPSILON_FOR_DESIREDS = 5.0e-4;
   private static final double EPSILON_FOR_HEIGHT = 1.0e-2;

   private static final boolean DEBUG = false;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      runSingleWaypointTest();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void runSingleWaypointTest()
   {
      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePose3D desiredRandomPelvisPose = getRandomPelvisPose(random, pelvis);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();

      desiredRandomPelvisPose.get(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.get(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);
      pelvisTrajectoryMessage.setSequenceId(random.nextLong());

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);
      ThreadTools.sleep(10); // Need to wait a little for the message to make through the intraprocess threads.

      success = simulationTestHelper.simulateNow(controllerDT);
      assertTrue(success);

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.STARTED,
                                                        0.0,
                                                        pelvis.getName(),
                                                        statusMessages.remove(0),
                                                        controllerDT);

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = EndToEndTestTools.findVector3D(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", simulationTestHelper);
      Quaternion midFeetZupOrientation = EndToEndTestTools.findQuaternion(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(),
                                                                          "midFeetZUp",
                                                                          simulationTestHelper);
      fromWorldToMidFeetZUpTransform.set(midFeetZupOrientation, midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      Point2D desiredPosition2d = new Point2D();
      desiredPosition2d.set(desiredPosition.getX(), desiredPosition.getY());
      desiredPosition2d.applyTransform(fromWorldToMidFeetZUpTransform);
      Quaternion desiredOrientationCorrected = new Quaternion(desiredOrientation);
      desiredOrientationCorrected.applyTransform(fromWorldToMidFeetZUpTransform);

      Point3D desiredPositionCorrected = new Point3D(desiredPosition);
      desiredPositionCorrected.setX(desiredPosition2d.getX());
      desiredPositionCorrected.setY(desiredPosition2d.getY());
      desiredOrientation.set(desiredOrientationCorrected);

      success = simulationTestHelper.simulateNow(trajectoryTime + 2.0);
      assertTrue(success);

      String pelvisName = fullRobotModel.getPelvis().getName();
      assertSingleWaypointExecuted(pelvisName,
                                   fullRobotModel,
                                   desiredPositionCorrected,
                                   desiredOrientation,
                                   simulationTestHelper,
                                   isUsingPelvisHeightControlOnly());

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.COMPLETED,
                                                        trajectoryTime,
                                                        desiredPosition,
                                                        desiredOrientation,
                                                        pelvis.getName(),
                                                        statusMessages.remove(0),
                                                        2.0e-4,
                                                        controllerDT);
   }

   @Test
   public void testSingleWaypointAndAbort() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      runSingleWaypointTest();

      StopAllTrajectoryMessage stopAll = new StopAllTrajectoryMessage();
      simulationTestHelper.publishToController(stopAll);

      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSingleWaypointAndWalk() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel,
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePose3D desiredRandomPelvisPose = getRandomPelvisPose(random, pelvis);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();

      desiredRandomPelvisPose.get(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.get(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                             .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(1.0,
                                                                                       desiredPosition,
                                                                                       desiredOrientation,
                                                                                       new Vector3D(),
                                                                                       new Vector3D()));

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);

      success = simulationTestHelper.simulateNow(4.0 * robotModel.getControllerDT());
      assertTrue(success);

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = EndToEndTestTools.findVector3D(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", simulationTestHelper);
      Quaternion midFeetZupOrientation = EndToEndTestTools.findQuaternion(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(),
                                                                          "midFeetZUp",
                                                                          simulationTestHelper);
      fromWorldToMidFeetZUpTransform.set(midFeetZupOrientation, midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      Point2D desiredPosition2d = new Point2D();
      desiredPosition2d.set(desiredPosition.getX(), desiredPosition.getY());
      desiredPosition2d.applyTransform(fromWorldToMidFeetZUpTransform);
      Quaternion desiredOrientationCorrected = new Quaternion(desiredOrientation);
      desiredOrientationCorrected.applyTransform(fromWorldToMidFeetZUpTransform);

      desiredPosition.setX(desiredPosition2d.getX());
      desiredPosition.setY(desiredPosition2d.getY());
      desiredOrientation.set(desiredOrientationCorrected);

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      String pelvisName = fullRobotModel.getPelvis().getName();
      assertSingleWaypointExecuted(pelvisName, fullRobotModel, desiredPosition, desiredOrientation, simulationTestHelper, isUsingPelvisHeightControlOnly());
      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();
      double walkingTime = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, simulationTestHelper.getRootRegistry());
      success = simulationTestHelper.simulateNow(2.0 + walkingTime);
      assertTrue(success);
   }

   private double sendWalkingPacket(DRCRobotModel robotModel,
                                    FullHumanoidRobotModel fullRobotModel,
                                    CommonHumanoidReferenceFrames referenceFrames,
                                    YoRegistry registry)
   {
      referenceFrames.updateFrames();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double stepTime = swingTime + transferTime;

      Vector2D desiredVelocity = new Vector2D(0.15, 0.0);
      int numberOfSteps = 10;
      FootstepDataListMessage footsteps = computeNextFootsteps(numberOfSteps,
                                                               RobotSide.LEFT,
                                                               referenceFrames.getSoleFrames(),
                                                               walkingControllerParameters,
                                                               desiredVelocity);
      footsteps.setDefaultSwingDuration(swingTime);
      footsteps.setDefaultTransferDuration(transferTime);

      simulationTestHelper.publishToController(footsteps);

      int timeWalking = numberOfSteps;
      double timeToCompleteWalking = stepTime * timeWalking;
      return timeToCompleteWalking;
   }

   public static FootstepDataListMessage computeNextFootsteps(int numberOfFootsteps,
                                                              RobotSide supportLeg,
                                                              SideDependentList<? extends ReferenceFrame> soleFrames,
                                                              WalkingControllerParameters walkingControllerParameters,
                                                              Vector2DReadOnly desiredVelocity)
   {
      MutableObject<FootstepDataListMessage> output = new MutableObject<>();
      ContinuousStepGenerator stepGenerator = new ContinuousStepGenerator();
      stepGenerator.configureWith(walkingControllerParameters);
      stepGenerator.setDesiredVelocityProvider(() -> desiredVelocity);
      stepGenerator.setFrameBasedFootPoseProvider(soleFrames);
      stepGenerator.setFootstepMessenger(output::setValue);
      stepGenerator.setNumberOfFootstepsToPlan(numberOfFootsteps);
      stepGenerator.startWalking();
      stepGenerator.update(0.0);

      return output.getValue();
   }

   public void testHeightUsingMultipleWaypoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(453563);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(200);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.setSequenceId(random.nextLong());
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearAngularSelection();
      selectionMatrix6D.clearLinearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix()
                             .set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
      pelvisTrajectoryMessage.getSe3Trajectory().getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getLinearPart()));

      FrameInformation frameInformation = pelvisTrajectoryMessage.getSe3Trajectory().getFrameInformation();
      frameInformation.setTrajectoryReferenceFrameId(MessageTools.toFrameId(midFootZUpGroundFrame));
      frameInformation.setDataReferenceFrameId(MessageTools.toFrameId(midFootZUpGroundFrame));

      pelvisTrajectoryMessage.setEnableUserPelvisControl(true);

      double heightAmp = 0.1;
      double heightFreq = 0.5;
      double finalHeight = 0.0;

      int trajectoryPointIndex = 0;
      for (double time = 0.0; time < trajectoryTime - timePerWaypoint; time += timePerWaypoint)
      {
         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         orientation.setYawPitchRoll(0.0, 0.0, 0.0);
         angularVelocity.set(0.0, 0.0, 0.0);

         double x = pelvisPosition.getX();
         double y = pelvisPosition.getY();
         double z = heightAmp * Math.sin(2.0 * Math.PI * heightFreq * time) + pelvisPosition.getZ() - 0.02;

         double dx = 0.0;
         double dy = 0.0;
         double dz = heightAmp * Math.PI * 2.0 * heightFreq * Math.cos(2.0 * Math.PI * heightFreq * time);

         // set the velocity to zero for the last waypoint
         if (time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                                .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         trajectoryPointIndex++;

         finalHeight = z;
      }
      FramePoint3D expectedDesiredPositionInWorld = new FramePoint3D(midFootZUpGroundFrame, 0.0, 0.0, finalHeight);
      expectedDesiredPositionInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);

      success = simulationTestHelper.simulateNow(3.0 * controllerDT);
      assertTrue(success);

      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.STARTED,
                                                        0.0,
                                                        "pelvisHeight",
                                                        statusMessages.remove(0),
                                                        controllerDT);

      assertCenterOfMassHeightManagerIsInState(simulationTestHelper, PelvisHeightControlMode.USER);
      String bodyName = pelvis.getName();
      assertEquals(numberOfTrajectoryPoints, findControllerNumberOfWaypointsForHeight(simulationTestHelper, pelvis));
      assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsInQueueForHeight(simulationTestHelper, pelvis));

      for (trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator; trajectoryPointIndex++)
      {
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(),
                                     fromMessage.getPosition(),
                                     fromMessage.getOrientation(),
                                     fromMessage.getLinearVelocity(),
                                     fromMessage.getAngularVelocity());
         //         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SE3TrajectoryPoint controllerTrajectoryPoint = findPelvisHeightTrajectoryPoint(pelvis,
                                                                                        bodyName + "Height",
                                                                                        trajectoryPointIndex,
                                                                                        simulationTestHelper);

         //         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);

         // only check a few points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            expectedTrajectoryPoint.set(fromMessage.getTime(),
                                        fromMessage.getPosition(),
                                        fromMessage.getOrientation(),
                                        fromMessage.getLinearVelocity(),
                                        fromMessage.getAngularVelocity());

            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
         }
      }

      success = simulationTestHelper.simulateNow(pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime() + 1.0);
      assertTrue(success);

      pelvisPosition.setToZero(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      assertEquals(finalHeight, pelvisPosition.getZ(), 0.004);
      assertCenterOfMassHeightManagerIsInState(simulationTestHelper, PelvisHeightControlMode.USER);

      expectedDesiredPositionInWorld.setX(Double.NaN);
      expectedDesiredPositionInWorld.setY(Double.NaN);
      assertEquals(1, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.COMPLETED,
                                                        pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime(),
                                                        expectedDesiredPositionInWorld,
                                                        null,
                                                        "pelvisHeight",
                                                        statusMessages.remove(0),
                                                        2.0e-4,
                                                        controllerDT);
   }

   @SuppressWarnings("unchecked")
   private void assertCenterOfMassHeightManagerIsInState(YoVariableHolder yoVariableHolder, PelvisHeightControlMode mode)
   {
      if (isUsingPelvisHeightControlOnly())
      {
         // No need to assert as there is no state machine.
         return;
      }

      YoEnum<PelvisHeightControlMode> centerOfMassHeightManagerState = (YoEnum<PelvisHeightControlMode>) yoVariableHolder.findVariable("CenterOfMassHeightManager",
                                                                                                                                       "CenterOfMassHeightManagerCurrentState");
      assertEquals(mode, centerOfMassHeightManagerState.getEnumValue());
   }

   private boolean isUsingPelvisHeightControlOnly()
   {
      return getRobotModel().getWalkingControllerParameters().usePelvisHeightControllerOnly();
   }

   private SE3TrajectoryPoint findPelvisHeightTrajectoryPoint(RigidBodyBasics rigidBody,
                                                              String bodyName,
                                                              int trajectoryPointIndex,
                                                              YoVariableHolder yoVariableHolder)
   {
      String suffix = "AtWaypoint" + trajectoryPointIndex;
      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      String pelvisZPrefix = rigidBody.getName();
      String positionZTrajectoryName = pelvisZPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String positionZName = pelvisZPrefix + "Position";
      String linearVelocityZName = pelvisZPrefix + "LinearVelocity";

      Point3D position = EndToEndTestTools.findPoint3D(positionZTrajectoryName, positionZName, suffix, yoVariableHolder);

      Vector3D linearVelocity = EndToEndTestTools.findVector3D(positionZTrajectoryName, linearVelocityZName, suffix, yoVariableHolder);

      String timeName = pelvisZPrefix + "Time";
      simpleSE3TrajectoryPoint.setTime(yoVariableHolder.findVariable(positionZTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.getPosition().set(position);
      simpleSE3TrajectoryPoint.getLinearVelocity().set(linearVelocity);

      return simpleSE3TrajectoryPoint;
   }

   @Test
   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(200);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.setEnableUserPelvisControlDuringWalking(true);
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearAngularSelection();
      selectionMatrix6D.clearLinearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix()
                             .set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
      pelvisTrajectoryMessage.getSe3Trajectory().getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getLinearPart()));

      FrameInformation frameInformation = pelvisTrajectoryMessage.getSe3Trajectory().getFrameInformation();
      frameInformation.setTrajectoryReferenceFrameId(MessageTools.toFrameId(midFootZUpGroundFrame));
      frameInformation.setDataReferenceFrameId(MessageTools.toFrameId(midFootZUpGroundFrame));

      pelvisTrajectoryMessage.setEnableUserPelvisControl(true);

      double heightAmp = 0.04;
      double heightFreq = 0.5;
      double finalHeight = 0.0;

      int trajectoryPointIndex = 0;
      for (double time = 0.0; time < trajectoryTime - timePerWaypoint; time += timePerWaypoint)
      {
         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         orientation.setYawPitchRoll(0.0, 0.0, 0.0);
         angularVelocity.set(0.0, 0.0, 0.0);

         double x = pelvisPosition.getX();
         double y = pelvisPosition.getY();
         double z = heightAmp * Math.sin(2.0 * Math.PI * heightFreq * time) + pelvisPosition.getZ() - 0.02;

         double dx = 0.0;
         double dy = 0.0;
         double dz = heightAmp * Math.PI * 2.0 * heightFreq * Math.cos(2.0 * Math.PI * heightFreq * time);

         // set the velocity to zero for the last waypoint
         if (time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                                .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         trajectoryPointIndex++;

         finalHeight = z;
      }

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);

      success = simulationTestHelper.simulateNow(3.0 * robotModel.getControllerDT());
      assertTrue(success);

      assertCenterOfMassHeightManagerIsInState(simulationTestHelper, PelvisHeightControlMode.USER);
      assertEquals(numberOfTrajectoryPoints, findControllerNumberOfWaypointsForHeight(simulationTestHelper, pelvis));
      assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsInQueueForHeight(simulationTestHelper, pelvis));

      for (trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator; trajectoryPointIndex++)
      {
         System.out.println(trajectoryPointIndex);
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(),
                                     fromMessage.getPosition(),
                                     fromMessage.getOrientation(),
                                     fromMessage.getLinearVelocity(),
                                     fromMessage.getAngularVelocity());
         //         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SE3TrajectoryPoint controllerTrajectoryPoint = findPelvisHeightTrajectoryPoint(pelvis, "pelvisHeight", trajectoryPointIndex, simulationTestHelper);

         //         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         System.out.println(expectedTrajectoryPoint.getPositionZ());
         assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);

         // only check a few points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            expectedTrajectoryPoint.set(fromMessage.getTime(),
                                        fromMessage.getPosition(),
                                        fromMessage.getOrientation(),
                                        fromMessage.getLinearVelocity(),
                                        fromMessage.getAngularVelocity());

            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
         }
      }

      referenceFrames.updateFrames();
      double walkingTime = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, simulationTestHelper.getRootRegistry());
      double simTime = Math.max(walkingTime, pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());

      success = simulationTestHelper.simulateNow(simTime + 3.0);
      assertTrue(success);

      pelvisPosition.setToZero(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      assertEquals(finalHeight, pelvisPosition.getZ(), 0.012);
      assertCenterOfMassHeightManagerIsInState(simulationTestHelper, PelvisHeightControlMode.USER);
   }

   @Test
   public void testHeightModeSwitchWhileWalking() throws Exception
   {

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(200);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.setEnableUserPelvisControlDuringWalking(false);
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearAngularSelection();
      selectionMatrix6D.clearLinearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix()
                             .set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
      pelvisTrajectoryMessage.getSe3Trajectory().getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getLinearPart()));

      FrameInformation frameInformation = pelvisTrajectoryMessage.getSe3Trajectory().getFrameInformation();
      frameInformation.setTrajectoryReferenceFrameId(MessageTools.toFrameId(midFootZUpGroundFrame));
      frameInformation.setDataReferenceFrameId(MessageTools.toFrameId(midFootZUpGroundFrame));

      pelvisTrajectoryMessage.setEnableUserPelvisControl(true);

      double heightAmp = 0.1;
      double heightFreq = 0.5;

      int trajectoryPointIndex = 0;
      for (double time = 0.0; time < trajectoryTime - timePerWaypoint; time += timePerWaypoint)
      {
         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         orientation.setYawPitchRoll(0.0, 0.0, 0.0);
         angularVelocity.set(0.0, 0.0, 0.0);

         double x = pelvisPosition.getX();
         double y = pelvisPosition.getY();
         double z = heightAmp * Math.sin(2.0 * Math.PI * heightFreq * time) + pelvisPosition.getZ() - 0.02;

         double dx = 0.0;
         double dy = 0.0;
         double dz = heightAmp * Math.PI * 2.0 * heightFreq * Math.cos(2.0 * Math.PI * heightFreq * time);

         // set the velocity to zero for the last waypoint
         if (time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                                .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         trajectoryPointIndex++;
      }

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);

      success = simulationTestHelper.simulateNow(3.0 * robotModel.getControllerDT());
      assertTrue(success);

      assertCenterOfMassHeightManagerIsInState(simulationTestHelper, PelvisHeightControlMode.USER);
      assertEquals(numberOfTrajectoryPoints, findControllerNumberOfWaypointsForHeight(simulationTestHelper, pelvis));
      assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsInQueueForHeight(simulationTestHelper, pelvis));

      for (trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator; trajectoryPointIndex++)
      {
         System.out.println(trajectoryPointIndex);
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(),
                                     fromMessage.getPosition(),
                                     fromMessage.getOrientation(),
                                     fromMessage.getLinearVelocity(),
                                     fromMessage.getAngularVelocity());
         //         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SE3TrajectoryPoint controllerTrajectoryPoint = findPelvisHeightTrajectoryPoint(pelvis, "pelvisHeight", trajectoryPointIndex, simulationTestHelper);

         //         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         System.out.println(expectedTrajectoryPoint.getPositionZ());
         assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);

         // only check a few points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            expectedTrajectoryPoint.set(fromMessage.getTime(),
                                        fromMessage.getPosition(),
                                        fromMessage.getOrientation(),
                                        fromMessage.getLinearVelocity(),
                                        fromMessage.getAngularVelocity());

            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
         }
      }

      referenceFrames.updateFrames();
      double walkingTime = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, simulationTestHelper.getRootRegistry());
      double simTime = Math.max(walkingTime, pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime());

      success = simulationTestHelper.simulateNow(simTime + 1.0);
      assertTrue(success);

      assertCenterOfMassHeightManagerIsInState(simulationTestHelper, PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   public PelvisTrajectoryMessage generateHoolaHoopTrajectory(ReferenceFrame supportFrame, FramePoint3D pelvisPosition, double radius)
   {
      double trajectoryDuration = 10.0;
      int numberOfWaypoints = 100;
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.setEnableUserPelvisControlDuringWalking(true);
      pelvisTrajectoryMessage.setEnableUserPelvisControl(true);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose3D desiredPose = new FramePose3D(supportFrame);
      FrameVector3D desiredAngularVelocity = new FrameVector3D(supportFrame);
      FrameVector3D desiredLinearVelocity = new FrameVector3D(supportFrame);

      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      Vector3D linearVelocity = new Vector3D();
      Vector3D angularVelocity = new Vector3D();

      YoPolynomial anglePolynomial = new YoPolynomial("hoolaHoopParameterPolynomial", 4, simulationTestHelper.getRootRegistry());
      anglePolynomial.setCubic(0.0, trajectoryDuration, 0.0, 0.0, Math.PI * 10.0, 0.0);

      double t = 0.0;
      double timeBetweenWaypoints = trajectoryDuration / numberOfWaypoints;

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         anglePolynomial.compute(t);

         double angle = anglePolynomial.getValue();
         double angleDot = anglePolynomial.getVelocity();
         if (i == 0 || i == numberOfWaypoints - 1)
         {
            angleDot = 0.0;
         }

         double x = radius * Math.cos(angle);
         double y = radius * Math.sin(angle);
         double z = Math.sin(angle) * 0.03;

         position.set(x, y, z);

         double dx = radius * -Math.sin(angle) * angleDot;
         double dy = radius * Math.cos(angle) * angleDot;
         double dz = Math.cos(angle) * 0.03 * angleDot;

         linearVelocity.set(dx, dy, dz);

         double yaw = Math.cos(angle) * 0.05;
         double pitch = -Math.cos(angle) * 0.1;
         double roll = Math.cos(angle) * 0.1;

         orientation.setYawPitchRoll(yaw, pitch, roll);

         double yawRate = 0.05 * -Math.sin(angle) * angleDot;
         double pitchRate = 0.1 * Math.sin(angle) * angleDot;
         double rollRate = 0.1 * -Math.sin(angle) * angleDot;

         RotationTools.computeAngularVelocityInBodyFrameFromYawPitchRollAnglesRate(yaw, pitch, roll, yawRate, pitchRate, rollRate, angularVelocity);

         pelvisPosition.changeFrame(supportFrame);
         position.add(pelvisPosition);
         desiredPose.setIncludingFrame(supportFrame, position, orientation);
         desiredPose.changeFrame(worldFrame);
         desiredPose.get(position, orientation);

         desiredLinearVelocity.setIncludingFrame(supportFrame, linearVelocity);
         desiredLinearVelocity.changeFrame(worldFrame);
         linearVelocity.set(desiredLinearVelocity);

         desiredAngularVelocity.setIncludingFrame(supportFrame, angularVelocity);
         desiredAngularVelocity.changeFrame(worldFrame);
         angularVelocity.set(desiredAngularVelocity);

         SE3TrajectoryPointMessage trajectoryPointMessage = HumanoidMessageTools.createSE3TrajectoryPointMessage(t
               + 2.0, position, orientation, linearVelocity, angularVelocity);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(trajectoryPointMessage);

         VisualDefinitionFactory factory = new VisualDefinitionFactory();
         factory.appendTranslation(position);
         factory.addSphere(0.01, ColorDefinitions.Black());
         simulationTestHelper.addStaticVisuals(factory.getVisualDefinitions());

         t += timeBetweenWaypoints;
      }

      return pelvisTrajectoryMessage;

   }

   public double getFootLength()
   {
      return 0.15;
   }

   @Test
   public void testSixDoFMovementsOfPelvis()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             new FlatGroundEnvironment(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(Math.toRadians(165.0)));
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      String pelvisName = fullRobotModel.getPelvis().getName();

      ReferenceFrame midFeetFrame = simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame();
      MovingReferenceFrame pelvisFrame = simulationTestHelper.getControllerReferenceFrames().getPelvisFrame();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvisFrame);
      double radius = getFootLength() / 3.0;
      PelvisTrajectoryMessage pelvisTrajectoryMessage = generateHoolaHoopTrajectory(midFeetFrame, pelvisPosition, radius);
      simulationTestHelper.publishToController(pelvisTrajectoryMessage);

      success = simulationTestHelper.simulateNow(4.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      int numberOfTrajectoryPoints = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().size();
      assertNumberOfWaypoints(numberOfTrajectoryPoints, simulationTestHelper);

      // Go through the queued points in the controller and make sure they match the message sent.
      int pointsToCheck = Math.min(numberOfTrajectoryPoints, RigidBodyTaskspaceControlState.maxPointsInGenerator - 1);
      for (int trajectoryPointIndex = 0; trajectoryPointIndex < pointsToCheck; trajectoryPointIndex++)
      {
         SE3TrajectoryPointMessage trajectoryPoint = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(pelvisName, trajectoryPointIndex + 1, simulationTestHelper);

         // Check time
         assertEquals(trajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);

         // Check desired position in midFeedFrame
         FramePoint3D expectedPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), trajectoryPoint.getPosition());
         FrameVector3D expectedLinearVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame(), trajectoryPoint.getLinearVelocity());
         expectedPosition.changeFrame(midFeetFrame);
         expectedLinearVelocity.changeFrame(midFeetFrame);
         EuclidCoreTestTools.assertEquals(expectedPosition, controllerTrajectoryPoint.getPosition(), EPSILON_FOR_DESIREDS);
         EuclidCoreTestTools.assertEquals(expectedLinearVelocity, controllerTrajectoryPoint.getLinearVelocity(), EPSILON_FOR_DESIREDS);

         // Check desired orientation in world frame
         EuclidCoreTestTools.assertEquals(trajectoryPoint.getOrientation(), controllerTrajectoryPoint.getOrientation(), EPSILON_FOR_DESIREDS);
         EuclidCoreTestTools.assertEquals(trajectoryPoint.getAngularVelocity(), controllerTrajectoryPoint.getAngularVelocity(), EPSILON_FOR_DESIREDS);
      }

      SE3TrajectoryPointMessage lastMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast();
      success = simulationTestHelper.simulateNow(lastMessage.getTime() + 1.0);
      assertTrue(success);

      SE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPointInWorldFrame(pelvisName, simulationTestHelper, midFeetFrame);
      EuclidCoreTestTools.assertEquals(lastMessage.getPosition(), controllerTrajectoryPoint.getPosition(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertEquals(lastMessage.getLinearVelocity(), controllerTrajectoryPoint.getLinearVelocity(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertEquals(lastMessage.getOrientation(), controllerTrajectoryPoint.getOrientation(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertEquals(lastMessage.getAngularVelocity(), controllerTrajectoryPoint.getAngularVelocity(), EPSILON_FOR_DESIREDS);
   }

   @Test
   public void testMultipleWaypoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      String pelvisName = fullRobotModel.getPelvis().getName();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 15;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();

      double rotationAmp = Math.toRadians(20.0);
      double pitchFreq = 0.5;

      double heightAmp = 0.05;
      double heightFreq = 0.5;

      for (double time = 0.1; time < trajectoryTime + 0.1; time += timePerWaypoint)
      {
         Quaternion orientation = new Quaternion();
         Vector3D angularVelocity = new Vector3D();

         double pitch = rotationAmp * Math.sin(2.0 * Math.PI * pitchFreq * time);
         double pitchDot = rotationAmp * Math.PI * 2.0 * pitchFreq * Math.cos(2.0 * Math.PI * pitchFreq * time);

         orientation.setYawPitchRoll(0.0, pitch, 0.0);
         angularVelocity.set(0.0, pitchDot, 0.0);

         double x = pelvisPosition.getX();
         double y = pelvisPosition.getY();
         double z = heightAmp * Math.sin(2.0 * Math.PI * heightFreq * time) + pelvisPosition.getZ() - 0.02;

         double dx = 0.0;
         double dy = 0.0;
         double dz = heightAmp * Math.PI * 2.0 * heightFreq * Math.cos(2.0 * Math.PI * heightFreq * time);

         // set the velocity to zero for the last waypoint
         if (time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                                .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
      }

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);
      ThreadTools.sleep(10);

      success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT());
      assertTrue(success);

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();

      success = simulationTestHelper.simulateNow(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(numberOfTrajectoryPoints + 1, simulationTestHelper);

      referenceFrames.updateFrames();
      midFeetZUpFrame.getTransformToDesiredFrame(fromWorldToMidFeetZUpTransform, ReferenceFrame.getWorldFrame());
      fromWorldToMidFeetZUpTransform.invert();

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < Math.min(numberOfTrajectoryPoints,
                                                                         RigidBodyTaskspaceControlState.maxPointsInGenerator - 1); trajectoryPointIndex++)
      {
         // Message is expressed in world frame
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(),
                                     fromMessage.getPosition(),
                                     fromMessage.getOrientation(),
                                     fromMessage.getLinearVelocity(),
                                     fromMessage.getAngularVelocity());
         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);

         // Controller is expressed in mid feet z-up frame
         SE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(pelvisName, trajectoryPointIndex + 1, simulationTestHelper);

         assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityX(), controllerTrajectoryPoint.getLinearVelocityX(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityY(), controllerTrajectoryPoint.getLinearVelocityY(), EPSILON_FOR_DESIREDS);

         // only check a few orientation points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            expectedTrajectoryPoint.set(fromMessage.getTime(),
                                        fromMessage.getPosition(),
                                        fromMessage.getOrientation(),
                                        fromMessage.getLinearVelocity(),
                                        fromMessage.getAngularVelocity());
            EuclidCoreTestTools.assertEquals(expectedTrajectoryPoint.getOrientation(), controllerTrajectoryPoint.getOrientation(), EPSILON_FOR_DESIREDS);
            EuclidCoreTestTools.assertEquals(expectedTrajectoryPoint.getAngularVelocity(),
                                             controllerTrajectoryPoint.getAngularVelocity(),
                                             EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
            System.out.println(expectedTrajectoryPoint.getPositionZ() + " : " + controllerTrajectoryPoint.getPositionZ());
            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), 0.008); //something is wrong with the frame change, just check we are within 6mm
         }
      }

      success = simulationTestHelper.simulateNow(trajectoryTime);
      assertTrue(success);

      referenceFrames.updateFrames();
      midFeetZUpFrame.getTransformToDesiredFrame(fromWorldToMidFeetZUpTransform, ReferenceFrame.getWorldFrame());

      // Message is expressed in world frame
      SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast();
      SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
      expectedTrajectoryPoint.set(fromMessage.getTime(),
                                  fromMessage.getPosition(),
                                  fromMessage.getOrientation(),
                                  fromMessage.getLinearVelocity(),
                                  fromMessage.getAngularVelocity());

      // Controller is expressed in mid feet z-up frame - this method changes it to world frame
      SE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPointInWorldFrame(pelvisName, simulationTestHelper, midFeetZUpFrame);

      // Not check the height on purpose as it is non-trivial.
      assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
      assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertEquals(expectedTrajectoryPoint.getLinearVelocity(), controllerTrajectoryPoint.getLinearVelocity(), EPSILON_FOR_DESIREDS);

      expectedTrajectoryPoint.set(fromMessage.getTime(),
                                  fromMessage.getPosition(),
                                  fromMessage.getOrientation(),
                                  fromMessage.getLinearVelocity(),
                                  fromMessage.getAngularVelocity());
      EuclidCoreTestTools.assertEquals(expectedTrajectoryPoint.getOrientation(), controllerTrajectoryPoint.getOrientation(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertEquals(expectedTrajectoryPoint.getAngularVelocity(), controllerTrajectoryPoint.getAngularVelocity(), EPSILON_FOR_DESIREDS);
   }

   @Test
   @SuppressWarnings("unchecked")
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 5.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePose3D desiredRandomPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomPelvisPose.getPosition().set(RandomGeometry.nextPoint3D(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() - 0.15);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();

      desiredRandomPelvisPose.get(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      desiredRandomPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomPelvisPose.get(desiredPosition, desiredOrientation);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
         System.out.println(desiredOrientation);
      }

      String managerName = PelvisOrientationManager.class.getSimpleName();
      YoEnum<PelvisOrientationControlMode> orientationControlMode = (YoEnum<PelvisOrientationControlMode>) simulationTestHelper.findVariable(managerName,
                                                                                                                                             managerName
                                                                                                                                                   + "CurrentState");

      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);

      simulationTestHelper.publishToController(pelvisTrajectoryMessage);

      assertEquals(PelvisOrientationControlMode.WALKING_CONTROLLER, orientationControlMode.getEnumValue());
      success = simulationTestHelper.simulateNow(trajectoryTime / 2.0);
      assertTrue(success);

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      simulationTestHelper.publishToController(stopAllTrajectoryMessage);

      assertEquals(PelvisOrientationControlMode.USER, orientationControlMode.getEnumValue());
      assertFalse(findControllerStopBooleanForXY(simulationTestHelper));
      if (!isUsingPelvisHeightControlOnly())
      {
         assertFalse(findControllerStopBooleanForHeight(simulationTestHelper));
      }
      String pelvisName = fullRobotModel.getPelvis().getName();
      QuaternionReadOnly controllerDesiredOrientationBeforeStop = EndToEndTestTools.findFeedbackControllerDesiredOrientation(pelvisName, simulationTestHelper);
      Point3DReadOnly controllerDesiredPelvisHeightBeforeStop = EndToEndTestTools.findFeedbackControllerDesiredPosition(pelvisName, simulationTestHelper);
      Point2D controllerDesiredXYBeforeStop = findControllerDesiredPositionXY(simulationTestHelper);

      success = simulationTestHelper.simulateNow(0.05);
      assertTrue(success);

      assertEquals(PelvisOrientationControlMode.WALKING_CONTROLLER, orientationControlMode.getEnumValue());
      assertTrue(findControllerStopBooleanForXY(simulationTestHelper));
      Point3DReadOnly controllerDesiredPelvisHeightAfterStop = EndToEndTestTools.findFeedbackControllerDesiredPosition(pelvisName, simulationTestHelper);
      QuaternionReadOnly controllerDesiredOrientationAfterStop = EndToEndTestTools.findFeedbackControllerDesiredOrientation(pelvisName, simulationTestHelper);
      Point2D controllerDesiredXYAfterStop = findControllerDesiredPositionXY(simulationTestHelper);

      EuclidCoreTestTools.assertEquals(controllerDesiredOrientationBeforeStop, controllerDesiredOrientationAfterStop, 1.0e-2);
      EuclidCoreTestTools.assertEquals("", controllerDesiredXYBeforeStop, controllerDesiredXYAfterStop, 1.0e-2);
      //checking pelvis hieght only
      assertEquals(controllerDesiredPelvisHeightBeforeStop.getZ(), controllerDesiredPelvisHeightAfterStop.getZ(), 1.0e-2);
   }

   @Test
   public void testSingleWaypointThenManualChange() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(1.0));

      String namespace = HeightOffsetHandler.class.getSimpleName();
      YoDouble offsetHeight = (YoDouble) simulationTestHelper.findVariable(namespace, "offsetHeightAboveGround");

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      MovingReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvisFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double initialPelvisHeight = pelvisPosition.getZ();

      Random random = new Random(4929L);
      for (int i = 0; i < 5; i++)
      {
         double offset1 = 0.06 * 2.0 * (random.nextDouble() - 0.5);
         double offset2 = 0.06 * 2.0 * (random.nextDouble() - 0.5);

         // Move pelvis using YoVariable
         offsetHeight.set(offset1);
         assertTrue(simulationTestHelper.simulateNow(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         Assert.assertEquals(initialPelvisHeight + offset1, pelvisPosition.getZ(), 0.01);

         // Move pelvis through message
         double desiredHeight = initialPelvisHeight + offset2;
         FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         desiredPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
         desiredPelvisPose.setZ(desiredHeight);
         desiredPelvisPose.get(desiredPosition, desiredOrientation);
         PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(0.5, desiredPosition, desiredOrientation);
         simulationTestHelper.publishToController(pelvisTrajectoryMessage);
         assertTrue(simulationTestHelper.simulateNow(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         Assert.assertEquals(desiredHeight, pelvisPosition.getZ(), 0.01);
      }
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(595161);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry testRegistry = new YoRegistry("testStreaming");

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getRootRegistry().addChild(testRegistry);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.5);
      assertTrue(success);
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = simulationTestHelper.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      Pose3D randomPose = EuclidGeometryRandomTools.nextPose3D(random, 0.05, 0.3);

      YoFramePose3D initialPose = new YoFramePose3D("pelvisInitialPose", worldFrame, testRegistry);
      YoFramePose3D finalPose = new YoFramePose3D("pelvisFinalPose", worldFrame, testRegistry);
      YoFramePose3D desiredPose = new YoFramePose3D("pelvisDesiredPose", worldFrame, testRegistry);
      YoFixedFrameSpatialVector desiredVelocity = new YoFixedFrameSpatialVector("pelvisDesiredVelocity", worldFrame, testRegistry);

      initialPose.setFromReferenceFrame(pelvis.getBodyFixedFrame());
      finalPose.setFromReferenceFrame(pelvis.getBodyFixedFrame());
      finalPose.prependTranslation(randomPose.getPosition());
      finalPose.appendRotation(randomPose.getOrientation());

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

            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            desiredPose.interpolate(initialPose, finalPose, alpha);

            if (alpha <= 0.0 || alpha >= 1.0)
            {
               desiredVelocity.setToZero();
            }
            else
            {
               calculator.computeAngularVelocity(desiredVelocity.getAngularPart(),
                                                 initialPose.getOrientation(),
                                                 finalPose.getOrientation(),
                                                 1.0 / trajectoryTime.getValue());
               desiredVelocity.getLinearPart().sub(finalPose.getPosition(), initialPose.getPosition());
               desiredVelocity.getLinearPart().scale(1.0 / trajectoryTime.getValue());
            }

            PelvisTrajectoryMessage message = HumanoidMessageTools.createPelvisTrajectoryMessage(0.0, desiredPose, desiredVelocity);
            message.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
            message.getSe3Trajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
            simulationTestHelper.publishToController(message);
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

      CommonHumanoidReferenceFrames humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();
      humanoidReferenceFrames.updateFrames();
      ReferenceFrame supportFrame = humanoidReferenceFrames.getMidFeetZUpFrame();

      humanoidReferenceFrames.updateFrames();
      FramePoint3D controllerDesiredPosition = new FramePoint3D(supportFrame, findControllerDesiredPositionXY(simulationTestHelper));
      controllerDesiredPosition.changeFrame(worldFrame);
      controllerDesiredPosition.setZ(EndToEndTestTools.findYoDouble(CenterOfMassHeightControlState.class.getSimpleName(),
                                                                    "desiredCoMHeightFromTrajectory",
                                                                    simulationTestHelper)
                                                      .getValue());
      FrameVector3D controllerDesiredLinearVelocity = new FrameVector3D(supportFrame, findControllerDesiredLinearVelocityXY(simulationTestHelper));
      controllerDesiredLinearVelocity.changeFrame(worldFrame);
      controllerDesiredLinearVelocity.setZ(EndToEndTestTools.findYoDouble("pelvisHeightOffsetMultipleWaypointsTrajectoryGenerator",
                                                                          "pelvisHeightOffsetSubTrajectoryCurrentVelocity",
                                                                          simulationTestHelper)
                                                            .getValue());
      SO3TrajectoryPoint currentDesiredTrajectoryPoint = EndToEndChestTrajectoryMessageTest.findCurrentDesiredTrajectoryPoint(simulationTestHelper, pelvis);

      EuclidCoreTestTools.assertEquals(desiredPose.getPosition(), controllerDesiredPosition, 5.0e-4);
      EuclidCoreTestTools.assertEquals(desiredVelocity.getLinearPart(), controllerDesiredLinearVelocity, 1.0e-7);
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(desiredPose.getOrientation(), currentDesiredTrajectoryPoint.getOrientation(), 5.0e-3);
      EuclidCoreTestTools.assertEquals(desiredVelocity.getAngularPart(), currentDesiredTrajectoryPoint.getAngularVelocity(), 1.0e-7);

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      controllerDesiredPosition = new FramePoint3D(supportFrame, findControllerDesiredPositionXY(simulationTestHelper));
      controllerDesiredPosition.changeFrame(worldFrame);
      controllerDesiredPosition.setZ(EndToEndTestTools.findYoDouble(CenterOfMassHeightControlState.class.getSimpleName(),
                                                                    "desiredCoMHeightFromTrajectory",
                                                                    simulationTestHelper)
                                                      .getValue());
      controllerDesiredLinearVelocity = new FrameVector3D(supportFrame, findControllerDesiredLinearVelocityXY(simulationTestHelper));
      controllerDesiredLinearVelocity.changeFrame(worldFrame);
      controllerDesiredLinearVelocity.setZ(EndToEndTestTools.findYoDouble("pelvisHeightOffsetMultipleWaypointsTrajectoryGenerator",
                                                                          "pelvisHeightOffsetSubTrajectoryCurrentVelocity",
                                                                          simulationTestHelper)
                                                            .getValue());
      currentDesiredTrajectoryPoint = EndToEndChestTrajectoryMessageTest.findCurrentDesiredTrajectoryPoint(simulationTestHelper, pelvis);

      EuclidCoreTestTools.assertEquals(desiredPose.getPosition(), controllerDesiredPosition, 1.0e-6);
      EuclidCoreTestTools.assertEquals(desiredVelocity.getLinearPart(), controllerDesiredLinearVelocity, 1.0e-7);
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(desiredPose.getOrientation(), currentDesiredTrajectoryPoint.getOrientation(), 1.0e-7);
      EuclidCoreTestTools.assertEquals(desiredVelocity.getAngularPart(), currentDesiredTrajectoryPoint.getAngularVelocity(), 1.0e-7);
   }

   public static Point2D findControllerDesiredPositionXY(YoVariableHolder yoVariableHolder)
   {
      String pelvisPrefix = "pelvisOffset";
      String trajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String currentPositionVarNamePrefix = pelvisPrefix + "CurrentPosition";

      return EndToEndTestTools.findPoint2D(trajectoryName, currentPositionVarNamePrefix, yoVariableHolder);
   }

   public static Vector2D findControllerDesiredLinearVelocityXY(YoVariableHolder yoVariableHolder)
   {
      String pelvisPrefix = "pelvisOffset";
      String trajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String currentPositionVarNamePrefix = pelvisPrefix + "CurrentVelocity";

      return EndToEndTestTools.findVector2D(trajectoryName, currentPositionVarNamePrefix, yoVariableHolder);
   }

   public static double findCurrentPelvisHeight(YoVariableHolder yoVariableHolder)
   {
      return yoVariableHolder.findVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
   }

   public static Vector3D findControllerDesiredLinearVelocity(String bodyName, YoVariableHolder yoVariableHolder)
   {
      String pelvisPrefix = "pelvisOffset";
      String trajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String currentLinearVelocityVarNamePrefix = pelvisPrefix + "CurrentVelocity";

      Vector3D linearVelocity = EndToEndTestTools.findVector3D(trajectoryName, currentLinearVelocityVarNamePrefix, yoVariableHolder);
      Vector3DReadOnly linearVelocityZ = EndToEndTestTools.findFeedbackControllerDesiredLinearVelocity(bodyName, yoVariableHolder);
      linearVelocity.setZ(linearVelocityZ.getZ());

      return linearVelocity;
   }

   public static boolean findControllerStopBooleanForXY(YoVariableHolder yoVariableHolder)
   {
      return ((YoBoolean) yoVariableHolder.findVariable(PelvisICPBasedTranslationManager.class.getSimpleName(),
                                                        "isPelvisTranslationalTrajectoryStopped")).getBooleanValue();
   }

   public static boolean findControllerStopBooleanForHeight(YoVariableHolder yoVariableHolder)
   {
      return ((YoBoolean) yoVariableHolder.findVariable(HeightOffsetHandler.class.getSimpleName(), "isPelvisOffsetHeightTrajectoryStopped")).getBooleanValue();
   }

   public static int findControllerNumberOfWaypointsForXY(YoVariableHolder yoVariableHolder)
   {
      String pelvisPrefix = "pelvisOffset";
      String positionTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((YoInteger) yoVariableHolder.findVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfCommandsInQueueForXY(YoVariableHolder yoVariableHolder)
   {
      String namespace = PelvisICPBasedTranslationManager.class.getSimpleName();
      String queuedCommandsVariableName = "PelvisXYTranslationNumberOfQueuedCommands";

      int numberOfCommands = (int) ((YoLong) yoVariableHolder.findVariable(namespace, queuedCommandsVariableName)).getLongValue();
      return numberOfCommands;
   }

   public static int findControllerNumberOfWaypointsForHeight(YoVariableHolder yoVariableHolder, RigidBodyBasics rigidBody)
   {
      //      String pelvisPrefix = "pelvisHeight";
      //      String offsetHeightTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      //      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";
      String bodyName = rigidBody.getName();
      YoInteger pelvisHeightTaskspaceNumberOfPoints = (YoInteger) yoVariableHolder.findVariable(bodyName + "TaskspaceControlModule",
                                                                                                bodyName + "PositionTaskspaceNumberOfPoints");
      int numberOfWaypoints = pelvisHeightTaskspaceNumberOfPoints.getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsInQueueForHeight(YoVariableHolder yoVariableHolder, RigidBodyBasics rigidBody)
   {
      String bodyName = rigidBody.getName();
      String offsetHeightTrajectoryName = bodyName + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = bodyName + "NumberOfWaypoints";
      int numberOfWaypoints = ((YoInteger) yoVariableHolder.findVariable(offsetHeightTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static SE3TrajectoryPoint findTrajectoryPoint(String bodyName, int trajectoryPointIndex, YoVariableHolder yoVariableHolder)
   {
      String suffix = "AtWaypoint" + trajectoryPointIndex;
      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();

      String pelvisXYPrefix = "pelvisOffset";
      String positionXYTrajectoryName = pelvisXYPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String positionXYName = pelvisXYPrefix + "Position";
      String linearVelocityXYName = pelvisXYPrefix + "LinearVelocity";

      Point3D position = EndToEndTestTools.findPoint3D(positionXYTrajectoryName, positionXYName, suffix, yoVariableHolder);
      Vector3D linearVelocity = EndToEndTestTools.findVector3D(positionXYTrajectoryName, linearVelocityXYName, suffix, yoVariableHolder);

      String timeName = pelvisXYPrefix + "Time";
      simpleSE3TrajectoryPoint.setTime(yoVariableHolder.findVariable(positionXYTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.getPosition().set(position);
      simpleSE3TrajectoryPoint.getLinearVelocity().set(linearVelocity);

      if (trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator)
      {
         String positionZTrajectoryName = bodyName + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
         String positionZName = bodyName + "Position";
         String linearVelocityZName = bodyName + "LinearVelocity";

         Point3D pelvisHeightPoint = EndToEndTestTools.findPoint3D(positionZTrajectoryName, positionZName, suffix, yoVariableHolder);
         double zHeight = pelvisHeightPoint.getZ();
         position.setZ(zHeight);

         double zLinearVelocity = EndToEndTestTools.findVector3D(positionZTrajectoryName, linearVelocityZName, suffix, yoVariableHolder).getZ();
         linearVelocity.setZ(zLinearVelocity);

         String orientationTrajectoryName = bodyName + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
         String orientationName = bodyName + "Orientation";
         String angularVelocityName = bodyName + "AngularVelocity";

         simpleSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) EndToEndTestTools.findQuaternion(orientationTrajectoryName, orientationName, suffix, yoVariableHolder));
         simpleSE3TrajectoryPoint.getAngularVelocity().set(EndToEndTestTools.findVector3D(orientationTrajectoryName, angularVelocityName, suffix, yoVariableHolder));
      }
      else
      {
         simpleSE3TrajectoryPoint.setTimeToNaN();
      }

      return simpleSE3TrajectoryPoint;
   }

   public static SE3TrajectoryPoint findCurrentDesiredTrajectoryPointInWorldFrame(String bodyName, YoVariableHolder yoVariableHolder, ReferenceFrame supportFrame)
   {
      SE3TrajectoryPoint simpleSE3TrajectoryPoint = new SE3TrajectoryPoint();
      // This will be in "support frame" we would like it in world frame.
      FramePoint2D positionXY = new FramePoint2D(supportFrame, findControllerDesiredPositionXY(yoVariableHolder));
      positionXY.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D position = new Point3D(positionXY.getX(), positionXY.getY(), Double.NaN);
      Point3DReadOnly positionZ = EndToEndTestTools.findFeedbackControllerDesiredPosition(bodyName, yoVariableHolder);
      position.setZ(positionZ.getZ());
      simpleSE3TrajectoryPoint.getPosition().set(position);
      simpleSE3TrajectoryPoint.getOrientation().set((Orientation3DReadOnly) EndToEndTestTools.findFeedbackControllerDesiredOrientation(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.getLinearVelocity().set(findControllerDesiredLinearVelocity(bodyName, yoVariableHolder));
      simpleSE3TrajectoryPoint.getAngularVelocity().set(EndToEndTestTools.findFeedbackControllerDesiredAngularVelocity(bodyName, yoVariableHolder));
      return simpleSE3TrajectoryPoint;
   }

   public static void assertSingleWaypointExecuted(String bodyName,
                                                   FullHumanoidRobotModel fullRobotModel,
                                                   Point3D desiredPosition,
                                                   Quaternion desiredOrientation,
                                                   YoVariableHolder yoVariableHolder,
                                                   boolean isUsingPelvisHeightControl)
   {
      assertNumberOfWaypoints(2, yoVariableHolder);

      Point2D desiredControllerXY = findControllerDesiredPositionXY(yoVariableHolder);
      assertEquals(desiredPosition.getX(), desiredControllerXY.getX(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getY(), desiredControllerXY.getY(), EPSILON_FOR_DESIREDS);

      QuaternionReadOnly desiredControllerOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(bodyName, yoVariableHolder);
      EuclidCoreTestTools.assertEquals(desiredOrientation, desiredControllerOrientation, EPSILON_FOR_DESIREDS);

      if (isUsingPelvisHeightControl)
      {
         double actualDesiredZ = EndToEndTestTools.findFeedbackControllerDesiredPosition(bodyName, yoVariableHolder).getZ();
         assertEquals(desiredPosition.getZ(), actualDesiredZ, EPSILON_FOR_DESIREDS);
      }
      else
      {
         // Hard to figure out how to verify the desired there
         //      trajOutput = scs.getVariable("pelvisHeightOffsetSubTrajectoryCubicPolynomialTrajectoryGenerator", "pelvisHeightOffsetSubTrajectoryCurrentValue").getValueAsDouble();
         //      assertEquals(desiredPosition.getZ(), trajOutput, EPSILON_FOR_DESIREDS);
         // Ending up doing a rough check on the actual height
         MovingReferenceFrame pelvisFrame = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint();
         FramePoint3D pelvisPosition = new FramePoint3D(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         double pelvisHeight = pelvisPosition.getZ();
         assertEquals(desiredPosition.getZ(), pelvisHeight, EPSILON_FOR_HEIGHT);
      }
   }

   public static void assertNumberOfWaypoints(int expectedNumberOfWaypoints, YoVariableHolder yoVariableHolder)
   {

      //The controller queues waypoints, got to check both banks of points
      if (expectedNumberOfWaypoints > RigidBodyTaskspaceControlState.maxPointsInGenerator)
      {
         assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsForXY(yoVariableHolder));

         //not the most intuitive, the pelvis xy manager queues the command itself
         assertEquals(1, findControllerNumberOfCommandsInQueueForXY(yoVariableHolder));
      }
      else
      {
         assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForXY(yoVariableHolder));
      }

      //      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForOrientation(scs));
      //      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForHeight(scs));
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

   protected FramePose3D getRandomPelvisPose(Random random, RigidBodyBasics pelvis)
   {
      FramePose3D desiredRandomPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomPelvisPose.getPosition().set(RandomGeometry.nextPoint3D(random, 0.05, 0.05, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() + getZOffset());
      return desiredRandomPelvisPose;
   }

   public double getZOffset()
   {
      return -0.1;
   }
}
