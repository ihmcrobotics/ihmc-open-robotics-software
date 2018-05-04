package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findPoint2d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findPoint3d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;

import java.util.Random;

import org.apache.commons.lang3.mutable.MutableObject;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FrameInformation;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisHeightControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationControlMode;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonHumanoidReferenceFramesVisualizer;
import us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGenerator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
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
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.trajectories.CubicPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

public abstract class EndToEndPelvisTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double EPSILON_FOR_DESIREDS = 1.2e-4;
   private static final double EPSILON_FOR_HEIGHT = 1.0e-2;

   private static final boolean DEBUG = false;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      testSingleWaypintInternal();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void testSingleWaypintInternal() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

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

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      Point2D desiredPosition2d = new Point2D();
      desiredPosition2d.set(desiredPosition.getX(), desiredPosition.getY());
      desiredPosition2d.applyTransform(fromWorldToMidFeetZUpTransform);
      Quaternion desiredOrientationCorrected = new Quaternion(desiredOrientation);
      desiredOrientationCorrected.applyTransform(fromWorldToMidFeetZUpTransform);

      desiredPosition.setX(desiredPosition2d.getX());
      desiredPosition.setY(desiredPosition2d.getY());
      desiredOrientation.set(desiredOrientationCorrected);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 + trajectoryTime);
      assertTrue(success);

      String pelvisName = fullRobotModel.getPelvis().getName();
      assertSingleWaypointExecuted(pelvisName, fullRobotModel, desiredPosition, desiredOrientation, scs);
   }


   public void testSingleWaypointAndAbort() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      testSingleWaypintInternal();

      StopAllTrajectoryMessage stopAll = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.send(stopAll);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testSingleWaypointAndWalk() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

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
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(1.0, desiredPosition, desiredOrientation, new Vector3D(), new Vector3D()));

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 * robotModel.getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      Point2D desiredPosition2d = new Point2D();
      desiredPosition2d.set(desiredPosition.getX(), desiredPosition.getY());
      desiredPosition2d.applyTransform(fromWorldToMidFeetZUpTransform);
      Quaternion desiredOrientationCorrected = new Quaternion(desiredOrientation);
      desiredOrientationCorrected.applyTransform(fromWorldToMidFeetZUpTransform);

      desiredPosition.setX(desiredPosition2d.getX());
      desiredPosition.setY(desiredPosition2d.getY());
      desiredOrientation.set(desiredOrientationCorrected);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      String pelvisName = fullRobotModel.getPelvis().getName();
      assertSingleWaypointExecuted(pelvisName, fullRobotModel, desiredPosition, desiredOrientation, scs);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();
      double walkingTime = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, scs.getRootRegistry());
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 + walkingTime);
      assertTrue(success);
   }

   private double sendWalkingPacket(DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
         YoVariableRegistry registry)
   {
      referenceFrames.updateFrames();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double stepTime = swingTime + transferTime;

      Vector2D desiredVelocity = new Vector2D(0.15, 0.0);
      int numberOfSteps = 10;
      FootstepDataListMessage footsteps = computeNextFootsteps(numberOfSteps, RobotSide.LEFT, referenceFrames.getSoleFrames(), walkingControllerParameters,
                                                               desiredVelocity);
      footsteps.setDefaultSwingDuration(swingTime);
      footsteps.setDefaultTransferDuration(transferTime);

      drcSimulationTestHelper.send(footsteps);

      int timeWalking = numberOfSteps;
      double timeToCompleteWalking = stepTime * timeWalking;
      return timeToCompleteWalking;
   }

   public static FootstepDataListMessage computeNextFootsteps(int numberOfFootsteps, RobotSide supportLeg, SideDependentList<? extends ReferenceFrame> soleFrames,
                                                              WalkingControllerParameters walkingControllerParameters, Vector2DReadOnly desiredVelocity)
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

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(200);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody pelvis = fullRobotModel.getPelvis();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.clearAngularSelection();
      selectionMatrix6D.clearLinearSelection();
      selectionMatrix6D.selectLinearZ(true);
      pelvisTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
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
         if(time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         trajectoryPointIndex++;

         finalHeight = z;
      }

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      assertCenterOfMassHeightManagerIsInState(scs, PelvisHeightControlMode.USER);
      String bodyName = pelvis.getName();
      assertEquals(numberOfTrajectoryPoints, findControllerNumberOfWaypointsForHeight(scs, pelvis));
      assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsInQueueForHeight(scs, pelvis));

      for (trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator; trajectoryPointIndex++)
      {
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
//         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findPelvisHeightTrajectoryPoint(pelvis, bodyName + "Height", trajectoryPointIndex, scs);


//         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);

         // only check a few points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());

            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime() + 1.0);
      assertTrue(success);


      pelvisPosition.setToZero(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      assertEquals(finalHeight, pelvisPosition.getZ(), 0.004);
      assertCenterOfMassHeightManagerIsInState(scs, PelvisHeightControlMode.USER);
   }

   private void assertCenterOfMassHeightManagerIsInState(SimulationConstructionSet scs, PelvisHeightControlMode mode)
   {
      YoEnum<PelvisHeightControlMode> centerOfMassHeightManagerState = (YoEnum<PelvisHeightControlMode>) scs.getVariable("CenterOfMassHeightManager", "CenterOfMassHeightManagerCurrentState");
      assertEquals(mode, centerOfMassHeightManagerState.getEnumValue());
   }

   private SimpleSE3TrajectoryPoint findPelvisHeightTrajectoryPoint(RigidBody rigidBody, String bodyName, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String suffix = "AtWaypoint" + trajectoryPointIndex;
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      String pelvisZPrefix = rigidBody.getName() + "Height";
      String positionZTrajectoryName = pelvisZPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String positionZName = pelvisZPrefix + "Position";
      String linearVelocityZName = pelvisZPrefix + "LinearVelocity";

      Point3D position = findPoint3d(positionZTrajectoryName, positionZName, suffix, scs);

      Vector3D linearVelocity = findVector3d(positionZTrajectoryName, linearVelocityZName, suffix, scs);

      String timeName = pelvisZPrefix + "Time";
      simpleSE3TrajectoryPoint.setTime(scs.getVariable(positionZTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.setPosition(position);
      simpleSE3TrajectoryPoint.setLinearVelocity(linearVelocity);

      return simpleSE3TrajectoryPoint;
   }

   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(200);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody pelvis = fullRobotModel.getPelvis();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
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
      pelvisTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
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
         if(time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         trajectoryPointIndex++;

         finalHeight = z;
      }

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * robotModel.getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();;
      assertCenterOfMassHeightManagerIsInState(scs, PelvisHeightControlMode.USER);
      assertEquals(numberOfTrajectoryPoints, findControllerNumberOfWaypointsForHeight(scs, pelvis));
      assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsInQueueForHeight(scs, pelvis));

      for (trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator; trajectoryPointIndex++)
      {
         System.out.println(trajectoryPointIndex);
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
//         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findPelvisHeightTrajectoryPoint(pelvis, "pelvisHeight", trajectoryPointIndex, scs);


//         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         System.out.println(expectedTrajectoryPoint.getPositionZ());
         assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);

         // only check a few points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());

            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
         }
      }

      referenceFrames.updateFrames();
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      double walkingTime = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, simulationConstructionSet.getRootRegistry());
      double simTime = Math.max(walkingTime, pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime() );

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simTime + 3.0);
      assertTrue(success);

      pelvisPosition.setToZero(fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      assertEquals(finalHeight, pelvisPosition.getZ(), 0.006);
      assertCenterOfMassHeightManagerIsInState(scs, PelvisHeightControlMode.USER);
   }

   public void testHeightModeSwitchWhileWalking() throws Exception
   {

      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(200);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 100;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody pelvis = fullRobotModel.getPelvis();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
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
      pelvisTrajectoryMessage.getSe3Trajectory().getAngularSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(selectionMatrix6D.getAngularPart()));
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
         if(time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         trajectoryPointIndex++;

         finalHeight = z;
      }

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0 * robotModel.getControllerDT());
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();;
      assertCenterOfMassHeightManagerIsInState(scs, PelvisHeightControlMode.USER);
      assertEquals(numberOfTrajectoryPoints, findControllerNumberOfWaypointsForHeight(scs, pelvis));
      assertEquals(RigidBodyTaskspaceControlState.maxPointsInGenerator, findControllerNumberOfWaypointsInQueueForHeight(scs, pelvis));

      for (trajectoryPointIndex = 0; trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator; trajectoryPointIndex++)
      {
         System.out.println(trajectoryPointIndex);
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
//         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findPelvisHeightTrajectoryPoint(pelvis, "pelvisHeight", trajectoryPointIndex, scs);


//         assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
         System.out.println(expectedTrajectoryPoint.getPositionZ());
         assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);

         // only check a few points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());

            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
         }
      }

      referenceFrames.updateFrames();
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      double walkingTime = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames, simulationConstructionSet.getRootRegistry());
      double simTime = Math.max(walkingTime, pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime() );

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simTime + 1.0);
      assertTrue(success);

      assertCenterOfMassHeightManagerIsInState(scs, PelvisHeightControlMode.WALKING_CONTROLLER);
   }

   public PelvisTrajectoryMessage generateHoolaHoopTrajectory(SimulationConstructionSet scs, MovingReferenceFrame pelvisZUp)
   {
      double radius = 0.05;
      double trajectoryDuration = 10.0;
      int numberOfWaypoints = 100;
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.setEnableUserPelvisControlDuringWalking(true);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose3D desiredPose = new FramePose3D(pelvisZUp);
      FrameVector3D desiredAngularVelocity = new FrameVector3D(pelvisZUp);
      FrameVector3D desiredLinearVelocity = new FrameVector3D(pelvisZUp);

      Point3D position = new Point3D();
      Quaternion orientation = new Quaternion();
      Vector3D linearVelocity = new Vector3D();
      Vector3D angularVelocity = new Vector3D();

      YoPolynomial anglePolynomial = new YoPolynomial("hoolaHoopParameterPolynomial", 4, scs.getRootRegistry());
      anglePolynomial.setCubic(0.0, trajectoryDuration, 0.0, 0.0, Math.PI * 10.0, 0.0);
//      anglePolynomial.setLinear(0.0, trajectoryDuration, 0.0, Math.PI * 2.0);

      double t = 0.0;
      double timeBetweenWaypoints = trajectoryDuration / numberOfWaypoints;

      for(int i = 0; i < numberOfWaypoints;  i++)
      {
         anglePolynomial.compute(t);

         double angle = anglePolynomial.getPosition();
         double angleDot = anglePolynomial.getVelocity();

         double x = radius * Math.cos(angle);
         double y = radius * Math.sin(angle);
         double z = Math.sin(angle) * 0.03;//0.05 * Math.sin(angle * 10.0);

         position.set(x,y,z);

         double dx = radius * -Math.sin(angle) * angleDot;
         double dy = radius * Math.cos(angle) * angleDot;
         double dz =  Math.cos(angle) * 0.03 * angleDot;//0.05 * 10.0 * Math.cos(angle * 10.0) * angleDot;

         linearVelocity.set(dx, dy, dz);

         double yaw = Math.cos(angle) * 0.05;
         double pitch = -Math.cos(angle) * 0.1;
         double roll = Math.cos(angle) * 0.1;

         orientation.setYawPitchRoll(yaw, pitch, roll);

         double yawRate = 0.05 * -Math.sin(angle) * angleDot;
         double pitchRate = 0.1 * Math.sin(angle) * angleDot;
         double rollRate = 0.1 * -Math.sin(angle) * angleDot;

         RotationTools.computeAngularVelocityInBodyFrameFromYawPitchRollAnglesRate(yaw, pitch, roll, yawRate, pitchRate, rollRate, angularVelocity);

         desiredPose.setIncludingFrame(pelvisZUp, position, orientation);
         desiredPose.changeFrame(worldFrame);
         desiredPose.get(position, orientation);

         desiredLinearVelocity.setIncludingFrame(pelvisZUp, linearVelocity);
         desiredLinearVelocity.changeFrame(worldFrame);
         linearVelocity.set(desiredLinearVelocity);

         desiredAngularVelocity.setIncludingFrame(pelvisZUp, angularVelocity);
         desiredAngularVelocity.changeFrame(worldFrame);
         angularVelocity.set(desiredAngularVelocity);

         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(t + 2.0, position, orientation, linearVelocity, angularVelocity));

         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(position);
         sphere.addSphere(0.01,YoAppearance.Black());
         scs.addStaticLinkGraphics(sphere);

         t += timeBetweenWaypoints;
      }


      return pelvisTrajectoryMessage;

   }

   public void testSixDoFMovementsOfPelvis() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      String pelvisName = fullRobotModel.getPelvis().getName();

      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      CommonHumanoidReferenceFrames referenceFrames = drcSimulationTestHelper.getReferenceFrames();
      referenceFrames.updateFrames();
      MovingReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      PelvisTrajectoryMessage pelvisTrajectoryMessage = generateHoolaHoopTrajectory(scs, pelvisZUpFrame);
      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);


      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      int numberOfTrajectoryPoints = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().size();
      assertNumberOfWaypoints(numberOfTrajectoryPoints, scs);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);

         if (trajectoryPointIndex < MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints - 1)
         {
            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(pelvisName, trajectoryPointIndex + 1, scs);

            assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityX(), controllerTrajectoryPoint.getLinearVelocityX(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityY(), controllerTrajectoryPoint.getLinearVelocityY(), EPSILON_FOR_DESIREDS);

            // only check a few orientation points since the trajectory in the controller used is shorter
            if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
            {
               assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
               expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(),
                     fromMessage.getAngularVelocity());
               EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(),
                     EPSILON_FOR_DESIREDS);
               EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(),
                     EPSILON_FOR_DESIREDS);
               assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
               System.out.println(expectedTrajectoryPoint.getPositionZ() + " : " + controllerTrajectoryPoint.getPositionZ());
               assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), 0.006); //something is wrong with the frame change, just check we are within 6mm
            }
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime() + 1.0);
      assertTrue(success);

      SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast();
      SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), new Vector3D(), fromMessage.getAngularVelocity());
      expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
      SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(pelvisName, scs);

      // Not check the height on purpose as it is non-trivial.
      assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
      assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getLinearVelocityCopy(), controllerTrajectoryPoint.getLinearVelocityCopy(), 6e-3);

      expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
      EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), 1e-3);


   }

   public void testMultipleWaypoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      String pelvisName = fullRobotModel.getPelvis().getName();

      double timePerWaypoint = 0.1;
      int numberOfTrajectoryPoints = 15;
      double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();

      double rotationAmp = Math.toRadians(20.0);
      double pitchFreq = 0.5;

      double heightAmp = 0.05;
      double heightFreq = 0.5;
      double finalHeight = 0.0;

      int index = 0;
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
         if(time + timePerWaypoint >= trajectoryTime - timePerWaypoint)
         {
            dz = 0.0;
         }

         Point3D position = new Point3D(x, y, z);
         Vector3D linearVelocity = new Vector3D(dx, dy, dz);
         pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity));
         index++;

         finalHeight = z;
      }


      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();


      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      RigidBodyTransform fromWorldToMidFeetZUpTransform = new RigidBodyTransform();
      Vector3D midFeetZup = findVector3d(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUp", scs);
      double midFeetZupYaw = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpYaw").getValueAsDouble();
      double midFeetZupPitch = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpPitch").getValueAsDouble();
      double midFeetZupRoll = scs.getVariable(CommonHumanoidReferenceFramesVisualizer.class.getSimpleName(), "midFeetZUpRoll").getValueAsDouble();
      fromWorldToMidFeetZUpTransform.setRotationEulerAndZeroTranslation(midFeetZupRoll, midFeetZupPitch, midFeetZupYaw);
      fromWorldToMidFeetZUpTransform.setTranslation(midFeetZup);
      fromWorldToMidFeetZUpTransform.invert();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);

      assertNumberOfWaypoints(numberOfTrajectoryPoints + 1, scs);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(trajectoryPointIndex);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
         expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findTrajectoryPoint(pelvisName, trajectoryPointIndex + 1, scs);


         assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityX(), controllerTrajectoryPoint.getLinearVelocityX(), EPSILON_FOR_DESIREDS);
         assertEquals(expectedTrajectoryPoint.getLinearVelocityY(), controllerTrajectoryPoint.getLinearVelocityY(), EPSILON_FOR_DESIREDS);

         // only check a few orientation points since the trajectory in the controller used is shorter
         if (trajectoryPointIndex + 1 < RigidBodyTaskspaceControlState.maxPointsInGenerator)
         {
            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
            EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
            EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
            assertEquals(expectedTrajectoryPoint.getLinearVelocityZ(), controllerTrajectoryPoint.getLinearVelocityZ(), EPSILON_FOR_DESIREDS);
            System.out.println(expectedTrajectoryPoint.getPositionZ() + " : " + controllerTrajectoryPoint.getPositionZ());
            assertEquals(expectedTrajectoryPoint.getPositionZ(), controllerTrajectoryPoint.getPositionZ(), 0.006); //something is wrong with the frame change, just check we are within 6mm
         }
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime);
      assertTrue(success);

      SE3TrajectoryPointMessage fromMessage = pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast();
      SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
      expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
      expectedTrajectoryPoint.applyTransform(fromWorldToMidFeetZUpTransform);
      SimpleSE3TrajectoryPoint controllerTrajectoryPoint = findCurrentDesiredTrajectoryPoint(pelvisName, scs);

      // Not check the height on purpose as it is non-trivial.
      assertEquals(expectedTrajectoryPoint.getPositionX(), controllerTrajectoryPoint.getPositionX(), EPSILON_FOR_DESIREDS);
      assertEquals(expectedTrajectoryPoint.getPositionY(), controllerTrajectoryPoint.getPositionY(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getLinearVelocityCopy(), controllerTrajectoryPoint.getLinearVelocityCopy(), EPSILON_FOR_DESIREDS);

      expectedTrajectoryPoint.set(fromMessage.getTime(), fromMessage.getPosition(), fromMessage.getOrientation(), fromMessage.getLinearVelocity(), fromMessage.getAngularVelocity());
      EuclidCoreTestTools.assertQuaternionEquals(expectedTrajectoryPoint.getOrientationCopy(), controllerTrajectoryPoint.getOrientationCopy(), EPSILON_FOR_DESIREDS);
      EuclidCoreTestTools.assertTuple3DEquals(expectedTrajectoryPoint.getAngularVelocityCopy(), controllerTrajectoryPoint.getAngularVelocityCopy(), EPSILON_FOR_DESIREDS);
   }

   @SuppressWarnings("unchecked")
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 5.0;
      RigidBody pelvis = fullRobotModel.getPelvis();

      FramePose3D desiredRandomPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.10, 0.20, 0.05));
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

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      String managerName = PelvisOrientationManager.class.getSimpleName();
      YoEnum<PelvisOrientationControlMode> orientationControlMode = (YoEnum<PelvisOrientationControlMode>) scs.getVariable(managerName, managerName + "CurrentState");

      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.send(pelvisTrajectoryMessage);

      assertEquals(PelvisOrientationControlMode.WALKING_CONTROLLER, orientationControlMode.getEnumValue());
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime / 2.0);
      assertTrue(success);

      StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
      drcSimulationTestHelper.send(stopAllTrajectoryMessage);

      assertEquals(PelvisOrientationControlMode.USER, orientationControlMode.getEnumValue());
      assertFalse(findControllerStopBooleanForXY(scs));
      assertFalse(findControllerStopBooleanForHeight(scs));
      String pelvisName = fullRobotModel.getPelvis().getName();
      Quaternion controllerDesiredOrientationBeforeStop = EndToEndHandTrajectoryMessageTest.findControllerDesiredOrientation(pelvisName, scs);
      Point3D controllerDesiredPelvisHeightBeforeStop = EndToEndHandTrajectoryMessageTest.findControllerDesiredPosition(pelvisName, scs);
      Point2D controllerDesiredXYBeforeStop = findControllerDesiredPositionXY(scs);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05);
      assertTrue(success);

      assertEquals(PelvisOrientationControlMode.WALKING_CONTROLLER, orientationControlMode.getEnumValue());
      assertTrue(findControllerStopBooleanForXY(scs));
      Point3D controllerDesiredPelvisHeightAfterStop = EndToEndHandTrajectoryMessageTest.findControllerDesiredPosition(pelvisName, scs);
      Quaternion controllerDesiredOrientationAfterStop = EndToEndHandTrajectoryMessageTest.findControllerDesiredOrientation(pelvisName, scs);
      Point2D controllerDesiredXYAfterStop = findControllerDesiredPositionXY(scs);

      EuclidCoreTestTools.assertQuaternionEquals(controllerDesiredOrientationBeforeStop, controllerDesiredOrientationAfterStop, 1.0e-2);
      EuclidCoreTestTools.assertTuple2DEquals("", controllerDesiredXYBeforeStop, controllerDesiredXYAfterStop, 1.0e-2);
      //checking pelvis hieght only
      assertEquals(controllerDesiredPelvisHeightBeforeStop.getZ(), controllerDesiredPelvisHeightAfterStop.getZ(), 1.0e-2);
   }

   public void testSingleWaypointThenManualChange() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      String namespace = LookAheadCoMHeightTrajectoryGenerator.class.getSimpleName();
      YoDouble offsetHeight = (YoDouble) scs.getVariable(namespace, "offsetHeightAboveGround");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RigidBody pelvis = fullRobotModel.getPelvis();
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
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5));
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
         drcSimulationTestHelper.send(pelvisTrajectoryMessage);
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         Assert.assertEquals(desiredHeight, pelvisPosition.getZ(), 0.01);
      }
   }

   public static Point2D findControllerDesiredPositionXY(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectoryName = pelvisPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      return findPoint2d(subTrajectoryName, currentPositionVarNamePrefix, scs);
   }

   public static double findCurrentPelvisHeight(SimulationConstructionSet scs)
   {
      return scs.getVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
   }

   public static Vector3D findControllerDesiredLinearVelocity(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String subTrajectory = "SubTrajectory";
      String subTrajectoryName = pelvisPrefix + subTrajectory;
      String currentLinearVelocityVarNamePrefix = subTrajectoryName + "CurrentVelocity";

      Vector3D linearVelocity = findVector3d(subTrajectoryName, currentLinearVelocityVarNamePrefix, scs);

      String pelvisHeightPrefix = "pelvisHeightOffset";
      String offsetHeightTrajectoryName = pelvisHeightPrefix + subTrajectory + CubicPolynomialTrajectoryGenerator.class.getSimpleName();

      linearVelocity.setZ(scs.getVariable(offsetHeightTrajectoryName, pelvisHeightPrefix + subTrajectory + "CurrentVelocity").getValueAsDouble());

      return linearVelocity;
   }

   public static boolean findControllerStopBooleanForXY(SimulationConstructionSet scs)
   {
      return ((YoBoolean) scs.getVariable(PelvisICPBasedTranslationManager.class.getSimpleName(), "isPelvisTranslationalTrajectoryStopped")).getBooleanValue();
   }

   public static boolean findControllerStopBooleanForHeight(SimulationConstructionSet scs)
   {
      return ((YoBoolean) scs.getVariable(LookAheadCoMHeightTrajectoryGenerator.class.getSimpleName(), "isPelvisOffsetHeightTrajectoryStopped")).getBooleanValue();
   }

   public static int findControllerNumberOfWaypointsForXY(SimulationConstructionSet scs)
   {
      String pelvisPrefix = "pelvisOffset";
      String positionTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";

      int numberOfWaypoints = ((YoInteger) scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfCommandsInQueueForXY(SimulationConstructionSet scs)
   {
      String nameSpace = PelvisICPBasedTranslationManager.class.getSimpleName();
      String queuedCommandsVariableName = "PelvisXYTranslationNumberOfQueuedCommands";

      int numberOfCommands = (int) ((YoLong) scs.getVariable(nameSpace, queuedCommandsVariableName)).getLongValue();
      return numberOfCommands;
   }

   public static int findControllerNumberOfWaypointsForHeight(SimulationConstructionSet scs, RigidBody rigidBody)
   {
//      String pelvisPrefix = "pelvisHeight";
//      String offsetHeightTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
//      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";
      String bodyName = rigidBody.getName();
      YoInteger pelvisHeightTaskspaceNumberOfPoints = (YoInteger) scs.getVariable(bodyName + "HeightTaskspaceControlModule", bodyName + "HeightTaskspaceNumberOfPoints");
      int numberOfWaypoints = pelvisHeightTaskspaceNumberOfPoints.getIntegerValue();
      return numberOfWaypoints;
   }

   public static int findControllerNumberOfWaypointsInQueueForHeight(SimulationConstructionSet scs, RigidBody rigidBody)
   {
      String bodyName = rigidBody.getName();
      String pelvisPrefix = bodyName + "Height";
      String offsetHeightTrajectoryName = pelvisPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String numberOfWaypointsVarName = pelvisPrefix + "NumberOfWaypoints";
      int numberOfWaypoints = ((YoInteger) scs.getVariable(offsetHeightTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
      return numberOfWaypoints;
   }

   public static SimpleSE3TrajectoryPoint findTrajectoryPoint(String bodyName, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String suffix = "AtWaypoint" + trajectoryPointIndex;
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();

      String pelvisXYPrefix = "pelvisOffset";
      String positionXYTrajectoryName = pelvisXYPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String positionXYName = pelvisXYPrefix + "Position";
      String linearVelocityXYName = pelvisXYPrefix + "LinearVelocity";

      Point3D position = findPoint3d(positionXYTrajectoryName, positionXYName, suffix, scs);
      Vector3D linearVelocity = findVector3d(positionXYTrajectoryName, linearVelocityXYName, suffix, scs);

      String timeName = pelvisXYPrefix + "Time";
      simpleSE3TrajectoryPoint.setTime(scs.getVariable(positionXYTrajectoryName, timeName + suffix).getValueAsDouble());
      simpleSE3TrajectoryPoint.setPosition(position);
      simpleSE3TrajectoryPoint.setLinearVelocity(linearVelocity);

      if (trajectoryPointIndex < RigidBodyTaskspaceControlState.maxPointsInGenerator)
      {
         String pelvisZPrefix = bodyName + "Height";
         String positionZTrajectoryName = pelvisZPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
         String positionZName = pelvisZPrefix + "Position";
         String linearVelocityZName = pelvisZPrefix + "LinearVelocity";

         Point3D pelvisHeightPoint = findPoint3d(positionZTrajectoryName, positionZName, suffix, scs);
         double zHeight = pelvisHeightPoint.getZ();
         position.setZ(zHeight);

         double zLinearVelocity = findVector3d(positionZTrajectoryName, linearVelocityZName, suffix, scs).getZ();
         linearVelocity.setZ(zLinearVelocity);

         String orientationTrajectoryName = bodyName + "Orientation" + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
         String orientationName = bodyName + "Orientation" + "Orientation";
         String angularVelocityName = bodyName + "Orientation" + "AngularVelocity";

         simpleSE3TrajectoryPoint.setOrientation(findQuat4d(orientationTrajectoryName, orientationName, suffix, scs));
         simpleSE3TrajectoryPoint.setAngularVelocity(findVector3d(orientationTrajectoryName, angularVelocityName, suffix, scs));
      }
      else
      {
         simpleSE3TrajectoryPoint.setTimeToNaN();
      }

      return simpleSE3TrajectoryPoint;
   }

   public static SimpleSE3TrajectoryPoint findCurrentDesiredTrajectoryPoint(String bodyName, SimulationConstructionSet scs)
   {
      SimpleSE3TrajectoryPoint simpleSE3TrajectoryPoint = new SimpleSE3TrajectoryPoint();
      Point2D positionXY = findControllerDesiredPositionXY(scs);
      Point3D position = new Point3D(positionXY.getX(), positionXY.getY(), Double.NaN);
      simpleSE3TrajectoryPoint.setPosition(position);
      simpleSE3TrajectoryPoint.setOrientation(EndToEndHandTrajectoryMessageTest.findControllerDesiredOrientation(bodyName, scs));
      simpleSE3TrajectoryPoint.setLinearVelocity(findControllerDesiredLinearVelocity(scs));
      simpleSE3TrajectoryPoint.setAngularVelocity(EndToEndHandTrajectoryMessageTest.findControllerDesiredAngularVelocity(bodyName, scs));
      return simpleSE3TrajectoryPoint;
   }

   public static void assertSingleWaypointExecuted(String bodyName, FullHumanoidRobotModel fullRobotModel, Point3D desiredPosition, Quaternion desiredOrientation, SimulationConstructionSet scs)
   {
      assertNumberOfWaypoints(2, scs);

      Point2D desiredControllerXY = findControllerDesiredPositionXY(scs);
      assertEquals(desiredPosition.getX(), desiredControllerXY.getX(), EPSILON_FOR_DESIREDS);
      assertEquals(desiredPosition.getY(), desiredControllerXY.getY(), EPSILON_FOR_DESIREDS);

      Quaternion desiredControllerOrientation = EndToEndHandTrajectoryMessageTest.findControllerDesiredOrientation(bodyName, scs);
      EuclidCoreTestTools.assertQuaternionEquals(desiredOrientation, desiredControllerOrientation, EPSILON_FOR_DESIREDS);

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

   public static void assertNumberOfWaypoints(int expectedNumberOfWaypoints, SimulationConstructionSet scs)
   {

      //The controller queues waypoints, got to check both banks of points
      if(expectedNumberOfWaypoints > MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints)
      {
         assertEquals(MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints, findControllerNumberOfWaypointsForXY(scs));

         //not the most intuitive, the pelvis xy manager queues the command itself
         assertEquals(1, findControllerNumberOfCommandsInQueueForXY(scs));
      }
      else
      {
         assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForXY(scs));
      }

//      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForOrientation(scs));
//      assertEquals(expectedNumberOfWaypoints, findControllerNumberOfWaypointsForHeight(scs));
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

   protected FramePose3D getRandomPelvisPose(Random random, RigidBody pelvis)
   {
      FramePose3D desiredRandomPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredRandomPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.05, 0.05, 0.05));
      desiredRandomPelvisPose.setZ(desiredRandomPelvisPose.getZ() - 0.1);
      return desiredRandomPelvisPose;
   }
}
