package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.controlModules.foot.LegSingularityAndKneeCollapseAvoidanceControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.SE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;

@Tag("controller-api")
public abstract class EndToEndFootTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private DRCSimulationTestHelper drcSimulationTestHelper;

   //the z height the foot comes off the ground before starting the trajectory
   protected double getLiftOffHeight()
   {
      return 0.15;
   }

   //the center of the trajectory
   protected Point3D getCircleCenterFromAnkle(RobotSide robotSide)
   {
      return new Point3D(0.0, 0.0, 0.15);
   }

   //how far out does the trajectory span from the circle center
   protected Vector3D getCircleRadius()
   {
      return new Vector3D(0.15, 0.15, 0.08);
   }

   //get a random position, if the radius and center are well tunes, this should be kinematically feasible
   private Point3D getRandomPositionInSphere(Random random, RobotSide robotSide)
   {
      Point3D circleCenterFromAnkle = getCircleCenterFromAnkle(robotSide);
      Vector3D circleRadius = getCircleRadius();

      Point3D supportingVertex = new Point3D(EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
      supportingVertex.scale(circleRadius.getX(), circleRadius.getY(), circleRadius.getZ());
      supportingVertex.scale(1.0 / supportingVertex.distanceFromOrigin());
      supportingVertex.scale(circleRadius.getX(), circleRadius.getY(), circleRadius.getZ());
      supportingVertex.add(circleCenterFromAnkle);

      Point3D next = new Point3D();
      next.interpolate(circleCenterFromAnkle, supportingVertex, random.nextDouble());
      return next;
   }

   //The first step in the test, send a FootTrajectoryMessage with a single point and simulate
   private boolean pickupFoot(RobotSide robotSide, RigidBodyBasics foot) throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double timeToPickupFoot = 1.0;

      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, getLiftOffHeight());
      footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide,
                                                                                                     timeToPickupFoot,
                                                                                                     desiredPosition,
                                                                                                     desiredOrientation);
      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToPickupFoot
            + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
   }

   //Put the foot back on the ground, this doesn't have any special ground checks, it's just easier to read this way
   private boolean putFootOnGround(RobotSide robotSide, RigidBodyBasics foot, FramePose3D desiredPose) throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double trajectoryTime = 1.0;

      desiredPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredPose.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide,
                                                                                                     trajectoryTime,
                                                                                                     desiredPosition,
                                                                                                     desiredOrientation);
      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      FootLoadBearingMessage loadBearingMessage = new FootLoadBearingMessage();
      loadBearingMessage.setRobotSide(robotSide.toByte());
      loadBearingMessage.setLoadBearingRequest(LoadBearingRequest.LOAD.toByte());
      drcSimulationTestHelper.publishToController(loadBearingMessage);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.2 + trajectoryTime
            + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
   }

   //moves the foot to a position using getRandomPositionInSphere
   private void moveFootToRandomPosition(Random random, RobotSide robotSide, RigidBodyBasics foot, SimulationConstructionSet scs)
         throws SimulationExceededMaximumTimeException
   {
      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      double trajectoryTime = 1.0;
      String bodyName = foot.getName();

      FramePose3D desiredRandomFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 0.3));
      desiredRandomFootPose.setPosition(getRandomPositionInSphere(random, robotSide));
      desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredRandomFootPose);
      footTrajectoryMessage.setSequenceId(random.nextLong());

      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      boolean result = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime
            + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(result);

      assertEquals(2, statusMessages.size());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(footTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.STARTED,
                                                        0.0,
                                                        foot.getName(),
                                                        statusMessages.remove(0),
                                                        getRobotModel().getControllerDT());
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(footTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.COMPLETED,
                                                        trajectoryTime,
                                                        desiredRandomFootPose,
                                                        foot.getName(),
                                                        statusMessages.remove(0),
                                                        1.0e-12,
                                                        getRobotModel().getControllerDT());

      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(bodyName, desiredRandomFootPose, scs);
   }

   //Picks up a foot, moves that foot to a position, and puts it down. Done using both sides
   @Test
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));

         // Now we can do the usual test. Asserts happen in the method
         moveFootToRandomPosition(random, robotSide, foot, scs);

         // Without forgetting to put the foot back on the ground
         assertTrue(putFootOnGround(robotSide, foot, initialFootPosition));
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   //Picks up a foot and puts it down. Done using both sides
   @Test
   public void testPickUpAndPutDown() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));

         // Without forgetting to put the foot back on the ground
         assertTrue(putFootOnGround(robotSide, foot, initialFootPosition));
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   //picks up a foot, moves it around in a ribbon shape, then puts the foot down, Done using both sides
   @Test
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      Random random = new Random(546547);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModel.updateFrames();
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));

         // Making the subscriber after having picked up the foot.
         List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
         drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         double timePerWaypoint = 0.1;
         int numberOfTrajectoryPoints = 26;
         double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

         String footName = foot.getName();
         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint3D circleCenter = new FramePoint3D(ankleFrame);
         circleCenter.set(getCircleCenterFromAnkle(robotSide));

         FrameQuaternion tempOrientation = new FrameQuaternion(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         FrameEuclideanTrajectoryPointList trajectoryPoints = createTrajectory(robotSide,
                                                                               foot,
                                                                               firstTrajectoryPointTime,
                                                                               numberOfTrajectoryPoints,
                                                                               1,
                                                                               trajectoryTime);
         List<FootTrajectoryMessage> footTrajectoryMessages = new ArrayList<>();
         ArrayDeque<FrameSE3TrajectoryPoint> frameSE3TrajectoryPoints = sendFootTrajectoryMessages(random,
                                                                                                   scs,
                                                                                                   robotSide,
                                                                                                   1,
                                                                                                   trajectoryPoints,
                                                                                                   footTrajectoryMessages);

         FootTrajectoryMessage footTrajectoryMessage = footTrajectoryMessages.remove(0);
         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(footTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.STARTED,
                                                           0.0,
                                                           footName,
                                                           statusMessages.remove(0),
                                                           controllerDT);

         int expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);

         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(footName, numberOfTrajectoryPoints + 1, scs);

         FrameSE3TrajectoryPoint lastFramePoint = frameSE3TrajectoryPoints.peekLast();
         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            FrameSE3TrajectoryPoint framePoint = frameSE3TrajectoryPoints.removeFirst();

            SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findSE3TrajectoryPoint(footName, trajectoryPointIndex, scs);
            SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
            framePoint.get(expectedTrajectoryPoint);

            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + firstTrajectoryPointTime);
         assertTrue(success);

         SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(footName, scs);
         SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
         lastFramePoint.get(expectedTrajectoryPoint);

         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime());
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));

         assertEquals(1, statusMessages.size());
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(footTrajectoryMessage.getSequenceId(),
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           lastFramePoint.getTime(),
                                                           lastFramePoint.getPosition(),
                                                           lastFramePoint.getOrientation(),
                                                           footName,
                                                           statusMessages.remove(0),
                                                           1.0e-12,
                                                           controllerDT);

         // Without forgetting to put the foot back on the ground
         putFootOnGround(robotSide, foot, initialFootPosition);
      }

      drcSimulationTestHelper.createVideo(robotModel.getSimpleRobotName(), 2);
   }

   //moves each foot to a single position using a custom control point
   @Test
   public void testCustomControlPoint() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(4.0, 0.0, 0.0), new Point3D(10.0, 0.0, -0.1));
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      RobotSide robotSide = RobotSide.LEFT;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      ReferenceFrame footFixedFrame = fullRobotModel.getFoot(robotSide).getBodyFixedFrame();

      RigidBodyTransform controlFrameTransform = new RigidBodyTransform();
      controlFrameTransform.setRotationEuler(Math.PI / 4.0, 0.0, Math.PI / 2.0);
      controlFrameTransform.setTranslation(-0.2, 0.2, -0.1);
      ReferenceFrame controlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("ControlFrame", footFixedFrame, controlFrameTransform);

      RigidBodyTransform controlFrameToWorldFrame = controlFrame.getTransformToWorldFrame();
      Graphics3DObject controlFrameGraphics = new Graphics3DObject();
      controlFrameGraphics.transform(controlFrameToWorldFrame);
      controlFrameGraphics.addCoordinateSystem(0.2);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addStaticLinkGraphics(controlFrameGraphics);

      ReferenceFrame trajectoryFrame = referenceFrames.getSoleFrame(robotSide.getOppositeSide());
      FramePose3D desiredPose = new FramePose3D(controlFrame);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPose.setZ(desiredPose.getZ() + getLiftOffHeight());
      desiredPose.changeFrame(trajectoryFrame);

      double trajectoryTime = 0.5;
      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPose);
      footTrajectoryMessage.getSe3Trajectory().setUseCustomControlFrame(true);
      footTrajectoryMessage.getSe3Trajectory().getControlFramePose().setOrientation(new Quaternion(controlFrameTransform.getRotation()));
      footTrajectoryMessage.getSe3Trajectory().getControlFramePose().setPosition(new Point3D(controlFrameTransform.getTranslation()));
      footTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(trajectoryFrame));

      drcSimulationTestHelper.publishToController(footTrajectoryMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime));

      // Since the control frame is moved down below the foot this assert makes sure the singularity escape uses the desired ankle position, not the desired control point position.
      String namePrefix = fullRobotModel.getFoot(robotSide).getName();
      String className = LegSingularityAndKneeCollapseAvoidanceControlModule.class.getSimpleName();
      YoBoolean singularityEscape = (YoBoolean) scs.getVariable(namePrefix + className, namePrefix + "IsSwingSingularityAvoidanceUsed");
      assertFalse("Singularity escape should not be active.", singularityEscape.getBooleanValue());

      drcSimulationTestHelper.createVideo(robotModel.getSimpleRobotName(), 2);
   }

   //picks up the left foot, moves the foot around a sphere (ribbons yawed around the circle center)
   @Test
   public void testQueuedMessages() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));
      double controllerDT = getRobotModel().getControllerDT();

      Random random = new Random(18721);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);

      // First need to pick up the foot:
      assertTrue(pickupFoot(robotSide, foot));

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      drcSimulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);

      // Now we can do the usual test.
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPointsPerMessage = 20;
      int numberOfMessages = 10;
      double trajectoryTime = 12.0;

      ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      FrameQuaternion tempOrientation = new FrameQuaternion(ankleFrame);
      tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FrameEuclideanTrajectoryPointList trajectoryPoints = createTrajectory(robotSide,
                                                                            foot,
                                                                            firstTrajectoryPointTime,
                                                                            numberOfTrajectoryPointsPerMessage,
                                                                            numberOfMessages,
                                                                            trajectoryTime);
      List<FootTrajectoryMessage> footTrajectoryMessages = new ArrayList<>();
      ArrayDeque<FrameSE3TrajectoryPoint> footTrajectoryPoints = sendFootTrajectoryMessages(random,
                                                                                            scs,
                                                                                            robotSide,
                                                                                            numberOfMessages,
                                                                                            trajectoryPoints,
                                                                                            footTrajectoryMessages);

      double timeOffset = 0.0;
      int totalNumberOfPoints = numberOfMessages * numberOfTrajectoryPointsPerMessage + 1;
      boolean firstSegment = true;
      FrameSE3TrajectoryPoint lastPoint = new FrameSE3TrajectoryPoint();
      String footName = fullRobotModel.getFoot(robotSide).getName();

      while (true)
      {
         int expectedNumberOfPointsInGenerator = Math.min(totalNumberOfPoints, RigidBodyTaskspaceControlState.maxPointsInGenerator);
         if (firstSegment)
            expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPointsPerMessage + 1);
         int expectedPointsInQueue = totalNumberOfPoints - expectedNumberOfPointsInGenerator;
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(footName, totalNumberOfPoints, scs);

         double lastPointTime = 0.0;
         fullRobotModel.updateFrames();

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            FrameSE3TrajectoryPoint framePoint = footTrajectoryPoints.removeFirst();

            SE3TrajectoryPoint controllerTrajectoryPoint = EndToEndTestTools.findSE3TrajectoryPoint(footName, trajectoryPointIndex, scs);
            SE3TrajectoryPoint expectedTrajectoryPoint = new SE3TrajectoryPoint();
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

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);

      // check internal desired matches last trajectory point:
      String nameSpacePositionDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePositionDesired = footName + Type.DESIRED.getName() + Space.POSITION.getName();
      Vector3D currentDesiredPosition = EndToEndTestTools.findVector3D(nameSpacePositionDesired, varnamePositionDesired, scs);

      String nameSpaceOrientationDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnameOrientationDesired = footName + Type.DESIRED.getName() + Space.ORIENTATION.getName();
      Quaternion currentDesiredOrientation = EndToEndTestTools.findQuaternion(nameSpaceOrientationDesired, varnameOrientationDesired, scs);

      EuclidCoreTestTools.assertTuple3DEquals(lastPoint.getPositionCopy(), currentDesiredPosition, 0.001);
      EuclidCoreTestTools.assertQuaternionEquals(lastPoint.getOrientationCopy(), currentDesiredOrientation, 0.001);

      assertEquals(2 * footTrajectoryMessages.size(), statusMessages.size());
      double startTime = 0.0;

      for (int inputIndex = 0; inputIndex < footTrajectoryMessages.size(); inputIndex++)
      {
         FootTrajectoryMessage footTrajectoryMessage = footTrajectoryMessages.get(inputIndex);
         Object<SE3TrajectoryPointMessage> taskspaceTrajectoryPoints = footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints();

         double endTime = startTime + taskspaceTrajectoryPoints.getLast().getTime();
         if (inputIndex > 0)
            startTime += taskspaceTrajectoryPoints.getFirst().getTime();

         TaskspaceTrajectoryStatusMessage startedStatus = statusMessages.remove(0);
         TaskspaceTrajectoryStatusMessage completedStatus = statusMessages.remove(0);
         long expectedSequenceID = footTrajectoryMessage.getSequenceId();
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                                           TrajectoryExecutionStatus.STARTED,
                                                           startTime,
                                                           footName,
                                                           startedStatus,
                                                           controllerDT);
         EndToEndTestTools.assertTaskspaceTrajectoryStatus(expectedSequenceID,
                                                           TrajectoryExecutionStatus.COMPLETED,
                                                           endTime,
                                                           footName,
                                                           completedStatus,
                                                           controllerDT);
         startTime = endTime;
      }

      drcSimulationTestHelper.createVideo(robotModel.getSimpleRobotName(), 2);
   }

   //takes the trajectory points created from the eucledian traj generator and divides them up by the number of messages desired and sends them to the controller
   private ArrayDeque<FrameSE3TrajectoryPoint> sendFootTrajectoryMessages(Random random, SimulationConstructionSet scs, RobotSide robotSide,
                                                                          int numberOfMessages, FrameEuclideanTrajectoryPointList trajectoryPoints)
         throws SimulationExceededMaximumTimeException
   {
      return sendFootTrajectoryMessages(random, scs, robotSide, numberOfMessages, trajectoryPoints, new ArrayList<>());
   }

   private ArrayDeque<FrameSE3TrajectoryPoint> sendFootTrajectoryMessages(Random random, SimulationConstructionSet scs, RobotSide robotSide,
                                                                          int numberOfMessages, FrameEuclideanTrajectoryPointList trajectoryPoints,
                                                                          List<FootTrajectoryMessage> messagesToPack)
         throws SimulationExceededMaximumTimeException
   {
      ArrayDeque<FrameSE3TrajectoryPoint> footTrajectoryPoints = new ArrayDeque<>();
      int numberOfTrajectoryPointsPerMessage = (int) Math.ceil(trajectoryPoints.getNumberOfTrajectoryPoints() / (double) numberOfMessages);
      int calculatorIndex = 0;
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage();
         footTrajectoryMessage.setSequenceId(random.nextLong());
         footTrajectoryMessage.setRobotSide(robotSide.toByte());
         footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setMessageId(id);
         if (messageIndex > 0)
         {
            footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
            footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setPreviousMessageId(id - 1);
         }
         id++;
         double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.getTrajectoryPoint(calculatorIndex - 1).getTime();

         for (int i = 0; i < numberOfTrajectoryPointsPerMessage; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();

            trajectoryPoints.getTrajectoryPoint(calculatorIndex).get(desiredPosition, desiredLinearVelocity);
            double time = trajectoryPoints.getTrajectoryPoint(calculatorIndex).getTime();

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(time
                  - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity));

            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(ReferenceFrame.getWorldFrame());
            framePoint.set(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            footTrajectoryPoints.addLast(framePoint);

            calculatorIndex++;
         }

         messagesToPack.add(footTrajectoryMessage);
         drcSimulationTestHelper.publishToController(footTrajectoryMessage);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);
      }

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
      assertTrue(success);
      return footTrajectoryPoints;
   }

   //picks the foot up, sends queued messages, the last with the wrong previous queued message ID. (Should see sysout about this) Checks that the number of waypoints is cleared, then puts the foot back on the ground. Done for both sides
   @Test
   public void testQueueWithWrongPreviousId() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getParentJoint().getFrameAfterJoint());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FrameQuaternion tempOrientation = new FrameQuaternion(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         FrameEuclideanTrajectoryPointList trajectoryPoints = createTrajectory(robotSide,
                                                                               foot,
                                                                               firstTrajectoryPointTime,
                                                                               numberOfTrajectoryPoints,
                                                                               numberOfMessages,
                                                                               trajectoryTime);

         sendFootTrajectoryMessages(new Random(923752), scs, robotSide, numberOfMessages, trajectoryPoints);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage();
         footTrajectoryMessage.setRobotSide(robotSide.toByte());
         footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setMessageId(100);
         footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
         footTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setPreviousMessageId(500); //not the right ID
         for (int i = 0; i < 5; i++)
         {
            footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                                 .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(i, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
         }
         drcSimulationTestHelper.publishToController(footTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2);
         assertTrue(success);

         String bodyName = fullRobotModel.getFoot(robotSide).getName();
         EndToEndTestTools.assertTotalNumberOfWaypointsInTaskspaceManager(bodyName, 1, scs);

         assertTrue(putFootOnGround(robotSide, foot, initialFootPosition));
      }
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(595161);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoVariableRegistry testRegistry = new YoVariableRegistry("testStreaming");

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoVariableRegistry(testRegistry);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      RobotSide robotSide = RobotSide.LEFT;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);

      assertTrue(pickupFoot(robotSide, foot));

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox()
                                               .getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      Pose3D randomPose = EuclidGeometryRandomTools.nextPose3D(random, 0.1, 0.6);
      randomPose.setZ(RandomNumbers.nextDouble(random, 0.1, 0.25));

      String prefix = robotSide.getCamelCaseName();
      YoFramePose3D initialPose = new YoFramePose3D(prefix + "FootInitialOrientation", worldFrame, testRegistry);
      YoFramePose3D finalPose = new YoFramePose3D(prefix + "FootFinalOrientation", worldFrame, testRegistry);
      YoFramePose3D desiredPose = new YoFramePose3D(prefix + "FootDesiredOrientation", worldFrame, testRegistry);
      YoFixedFrameSpatialVector desiredVelocity = new YoFixedFrameSpatialVector(prefix + "FootDesiredAngularVelocity", worldFrame, testRegistry);

      initialPose.setFromReferenceFrame(foot.getBodyFixedFrame());
      finalPose.setFromReferenceFrame(foot.getBodyFixedFrame());
      finalPose.prependTranslation(randomPose.getPosition());
      finalPose.appendRotation(randomPose.getOrientation());

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

            FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, 0.0, desiredPose, desiredVelocity, worldFrame);
            message.getSe3Trajectory().setUseCustomControlFrame(true);
            message.getSe3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
            message.getSe3Trajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
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

      double desiredEpsilon = 6.0e-3;

      SE3TrajectoryPoint currentDesiredTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(foot.getName(), scs);
      Pose3D controllerDesiredPose = new Pose3D(currentDesiredTrajectoryPoint.getPosition(), currentDesiredTrajectoryPoint.getOrientation());
      SpatialVector controllerDesiredVelocity = new SpatialVector(ReferenceFrame.getWorldFrame(),
                                                                  currentDesiredTrajectoryPoint.getAngularVelocity(),
                                                                  currentDesiredTrajectoryPoint.getLinearVelocity());

      EuclidGeometryTestTools.assertPose3DEquals(desiredPose, controllerDesiredPose, desiredEpsilon);
      MecanoTestTools.assertSpatialVectorEquals(desiredVelocity, controllerDesiredVelocity, desiredEpsilon);

      FramePose3D currentPose = new FramePose3D(foot.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);
      EuclidGeometryTestTools.assertPose3DGeometricallyEquals("Poor tracking for side: " + robotSide + " position: "
            + currentPose.getPositionDistance(controllerDesiredPose) + ", orientation: "
            + Math.abs(AngleTools.trimAngleMinusPiToPi(currentPose.getOrientationDistance(controllerDesiredPose))), controllerDesiredPose, currentPose, 5.0e-3);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);

      currentDesiredTrajectoryPoint = EndToEndTestTools.findFeedbackControllerCurrentDesiredSE3TrajectoryPoint(foot.getName(), scs);
      controllerDesiredPose = new Pose3D(currentDesiredTrajectoryPoint.getPosition(), currentDesiredTrajectoryPoint.getOrientation());
      controllerDesiredVelocity = new SpatialVector(ReferenceFrame.getWorldFrame(),
                                                    currentDesiredTrajectoryPoint.getAngularVelocity(),
                                                    currentDesiredTrajectoryPoint.getLinearVelocity());

      desiredEpsilon = 1.0e-7;
      EuclidGeometryTestTools.assertPose3DEquals(desiredPose, controllerDesiredPose, desiredEpsilon);
      MecanoTestTools.assertSpatialVectorEquals(desiredVelocity, controllerDesiredVelocity, desiredEpsilon);

      currentPose = new FramePose3D(foot.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);
      EuclidGeometryTestTools.assertPose3DGeometricallyEquals("Poor tracking for side: " + robotSide + " position: "
            + currentPose.getPositionDistance(controllerDesiredPose) + ", orientation: "
            + Math.abs(AngleTools.trimAngleMinusPiToPi(currentPose.getOrientationDistance(controllerDesiredPose))), controllerDesiredPose, currentPose, 1.0e-3);
   }

   //Creates a trajectory for the foot, depending on the number of iterations, it looks like a ribbon or a sphere
   private FrameEuclideanTrajectoryPointList createTrajectory(RobotSide robotSide, RigidBodyBasics foot, double firstTrajectoryPointTime,
                                                              int numberOfTrajectoryPoints, int numberOfIterations, double trajectoryTime)
   {
      ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      FramePoint3D circleCenter = new FramePoint3D(ankleFrame);
      circleCenter.set(getCircleCenterFromAnkle(robotSide));

      FramePoint3D tempPoint = new FramePoint3D();
      FrameQuaternion tempOrientation1 = new FrameQuaternion(ankleFrame);
      tempOrientation1.changeFrame(ReferenceFrame.getWorldFrame());
      Vector3D circleRadius = getCircleRadius();

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();
      double timeTick = trajectoryTime / (numberOfTrajectoryPoints * numberOfIterations - 1);
      for (int messageIndex = 0; messageIndex < numberOfIterations; messageIndex++)
      {
         double rot = messageIndex / (numberOfIterations - 1.0) * 1.0 * Math.PI;
         if (Double.isNaN(rot))
         {
            rot = 0;
         }

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
            if (robotSide == RobotSide.LEFT)
               tempPoint.setIncludingFrame(ankleFrame,
                                           circleRadius.getX() * Math.sin(angle) * Math.sin(rot),
                                           circleRadius.getY() * Math.sin(angle) * Math.cos(rot),
                                           circleRadius.getZ() * Math.sin(2.0 * angle));
            else
               tempPoint.setIncludingFrame(ankleFrame,
                                           circleRadius.getX() * Math.sin(2.0 * angle) * Math.sin(rot),
                                           circleRadius.getY() * Math.sin(angle),
                                           circleRadius.getZ() * Math.sin(2.0 * angle) * Math.cos(rot));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint((i + messageIndex * numberOfTrajectoryPoints) * timeTick, tempPoint);
         }
      }
      euclideanTrajectoryPointCalculator.compute(trajectoryTime);

      FrameEuclideanTrajectoryPointList trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
      trajectoryPoints.addTimeOffset(firstTrajectoryPointTime);
      return trajectoryPoints;
   }

   //Picks a foot up, Sends queued messages, then sends a single point to test overriding the queue. Done for both feet
   @Test
   public void testQueueStoppedWithOverrideMessage() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(564574L);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));

         // send the queued messages.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPointsPerMessage = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         FrameEuclideanTrajectoryPointList trajectoryPoints = createTrajectory(robotSide,
                                                                               foot,
                                                                               firstTrajectoryPointTime,
                                                                               numberOfTrajectoryPointsPerMessage,
                                                                               1,
                                                                               trajectoryTime);
         sendFootTrajectoryMessages(random, scs, robotSide, numberOfMessages, trajectoryPoints);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);

         //send a single trajectory point message in override mode
         moveFootToRandomPosition(random, robotSide, foot, scs);

         // Without forgetting to put the foot back on the ground
         putFootOnGround(robotSide, foot, initialFootPosition);
      }
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
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
