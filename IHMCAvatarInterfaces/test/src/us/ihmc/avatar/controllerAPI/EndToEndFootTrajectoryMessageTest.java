package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findPoint3d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findQuat4d;
import static us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest.findVector3d;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.LegSingularityAndKneeCollapseAvoidanceControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Space;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerDataReadOnly.Type;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndFootTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final double EPSILON_FOR_DESIREDS = 1.0e-10;

   private final CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 41.5)
   @Test
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters,
            getRobotModel());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.5, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper
               .simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double trajectoryTime = 1.0;
         FramePose desiredRandomFootPose = new FramePose(foot.getBodyFixedFrame());
         desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
         desiredRandomFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
         desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         desiredRandomFootPose.getPose(desiredPosition, desiredOrientation);
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

         String bodyName = fullRobotModel.getFoot(robotSide).getName();
         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(bodyName, desiredPosition, desiredOrientation, scs);
         //         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.prependTranslation(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper
               .simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 41.5)
   @Test
   public void testCustomControlPoint() throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(4.0, 0.0, 0.0), new Point3D(10.0, 0.0, -0.1));
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      RobotSide robotSide = RobotSide.LEFT;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      ReferenceFrame ankleFrame = referenceFrames.getFootFrame(robotSide);
      ReferenceFrame footFixedFrame = fullRobotModel.getFoot(robotSide).getBodyFixedFrame();

      RigidBodyTransform controlFrameTransform = new RigidBodyTransform();
      controlFrameTransform.setRotationEuler(Math.PI / 4.0, 0.0, Math.PI / 2.0);
      controlFrameTransform.setTranslation(-0.2, 0.2, -0.1);
      ReferenceFrame controlFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("ControlFrame", footFixedFrame, controlFrameTransform);

      RigidBodyTransform controlFrameToWorldFrame = controlFrame.getTransformToWorldFrame();
      Graphics3DObject controlFrameGraphics = new Graphics3DObject();
      controlFrameGraphics.transform(controlFrameToWorldFrame);
      controlFrameGraphics.addCoordinateSystem(0.2);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addStaticLinkGraphics(controlFrameGraphics);

      ReferenceFrame trajectoryFrame = referenceFrames.getSoleFrame(robotSide.getOppositeSide());
      FramePose desiredPose = new FramePose(controlFrame);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPose.setZ(desiredPose.getZ() + 0.15);
      desiredPose.changeFrame(trajectoryFrame);

      double trajectoryTime = 0.5;
      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPose);
      footTrajectoryMessage.setUseCustomControlFrame(true);
      footTrajectoryMessage.setControlFrameOrientation(new Quaternion(controlFrameTransform.getRotationMatrix()));
      footTrajectoryMessage.setControlFramePosition(new Point3D(controlFrameTransform.getTranslationVector()));
      footTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrame(trajectoryFrame);

      drcSimulationTestHelper.send(footTrajectoryMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime));

      // Since the control frame is moved down below the foot this assert makes sure the singularity escape uses the desired ankle position, not the desired control point position.
      String namePrefix = fullRobotModel.getFoot(robotSide).getName();
      String className = LegSingularityAndKneeCollapseAvoidanceControlModule.class.getSimpleName();
      YoBoolean singularityEscape = (YoBoolean) scs.getVariable(namePrefix + className, namePrefix + "IsSwingSingularityAvoidanceUsed");
      assertFalse("Singularity escape should not be active.", singularityEscape.getBooleanValue());
   }

   @ContinuousIntegrationTest(estimatedDuration = 46.8)
   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters,
            getRobotModel());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         double timePerWaypoint = 0.1;
         int numberOfTrajectoryPoints = 25;
         double trajectoryTime = numberOfTrajectoryPoints * timePerWaypoint;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
         ArrayDeque<SE3TrajectoryPointMessage> trajectoryPointQueue = new ArrayDeque<>();

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / (numberOfTrajectoryPoints - 1.0) * 2.0 * Math.PI;
            tempPoint.setIncludingFrame(ankleFrame, 0.0, radiusY * Math.cos(angle), radiusZ * Math.sin(2.0 * angle));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(i).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            footTrajectoryMessage.setTrajectoryPoint(i, time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
                  ReferenceFrame.getWorldFrame());

            SE3TrajectoryPointMessage point = new SE3TrajectoryPointMessage(time, desiredPosition, desiredOrientation, desiredLinearVelocity,
                  desiredAngularVelocity);
            trajectoryPointQueue.addLast(point);
         }

         drcSimulationTestHelper.send(footTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * getRobotModel().getControllerDT());
         assertTrue(success);
         fullRobotModel.updateFrames();
         int expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);

         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         String footName = fullRobotModel.getFoot(robotSide).getName();
         EndToEndHandTrajectoryMessageTest.assertNumberOfWaypoints(footName, numberOfTrajectoryPoints + 1, scs);

         SE3TrajectoryPointMessage lastPoint = trajectoryPointQueue.peekLast();
         FrameSE3TrajectoryPoint lastFramePoint = new FrameSE3TrajectoryPoint(worldFrame);
         lastFramePoint.set(lastPoint.time, lastPoint.position, lastPoint.orientation, lastPoint.linearVelocity, lastPoint.angularVelocity);

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            SE3TrajectoryPointMessage point = trajectoryPointQueue.removeFirst();
            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(worldFrame);
            framePoint.set(point.time, point.position, point.orientation, point.linearVelocity, point.angularVelocity);

            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = EndToEndHandTrajectoryMessageTest.findTrajectoryPoint(footName, trajectoryPointIndex, scs);
            SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
            framePoint.get(expectedTrajectoryPoint);

            assertEquals(expectedTrajectoryPoint.getTime(), controllerTrajectoryPoint.getTime(), EPSILON_FOR_DESIREDS);
            assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + firstTrajectoryPointTime);
         assertTrue(success);

         SimpleSE3TrajectoryPoint controllerTrajectoryPoint = EndToEndHandTrajectoryMessageTest.findCurrentDesiredTrajectoryPoint(footName, scs);
         SimpleSE3TrajectoryPoint expectedTrajectoryPoint = new SimpleSE3TrajectoryPoint();
         lastFramePoint.get(expectedTrajectoryPoint);

         controllerTrajectoryPoint.setTime(expectedTrajectoryPoint.getTime());
         assertTrue(expectedTrajectoryPoint.epsilonEquals(controllerTrajectoryPoint, 0.01));

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.prependTranslation(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper
               .simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 108.9)
   @Test
   public void testQueuedMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters,
            getRobotModel());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RobotSide robotSide = RobotSide.LEFT;

      // First need to pick up the foot:
      RigidBody foot = fullRobotModel.getFoot(robotSide);
      FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
      footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPoints = 20;
      int numberOfMessages = 10;
      double trajectoryTime = 12.0;

      ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      FramePoint circleCenter = new FramePoint(ankleFrame);
      circleCenter.setZ(0.15);
      double radiusXY = 0.15;
      double radiusZ = 0.08;
      FramePoint tempPoint = new FramePoint();
      FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
      tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      List<FootTrajectoryMessage> messages = new ArrayList<>();
      ArrayDeque<FrameSE3TrajectoryPoint> footTrajectoryPoints = new ArrayDeque<>();

      EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         double rot = messageIndex / (numberOfMessages - 1.0) * 1.0 * Math.PI;

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
            if (robotSide == RobotSide.LEFT)
               tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(angle) * Math.sin(rot), radiusXY * Math.sin(angle) * Math.cos(rot),
                     radiusZ * Math.sin(2.0 * angle));
            else
               tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(2.0 * angle) * Math.sin(rot), radiusXY * Math.sin(angle),
                     radiusZ * Math.sin(2.0 * angle) * Math.cos(rot));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
         }
      }

      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

      int calculatorIndex = 0;
      long id = 4678L;

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
         footTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;
         double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();
            tempOrientation.getQuaternion(desiredOrientation);

            double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

            Graphics3DObject sphere = new Graphics3DObject();
            sphere.translate(desiredPosition);
            sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
            scs.addStaticLinkGraphics(sphere);

            footTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity,
                  desiredAngularVelocity, ReferenceFrame.getWorldFrame());

            FrameSE3TrajectoryPoint framePoint = new FrameSE3TrajectoryPoint(ReferenceFrame.getWorldFrame());
            framePoint.set(time, desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity);
            footTrajectoryPoints.addLast(framePoint);

            calculatorIndex++;
         }

         messages.add(footTrajectoryMessage);
         drcSimulationTestHelper.send(footTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);
      }

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);

      double timeOffset = 0.0;
      int totalNumberOfPoints = numberOfMessages * numberOfTrajectoryPoints + 1;
      boolean firstSegment = true;
      FrameSE3TrajectoryPoint lastPoint = new FrameSE3TrajectoryPoint();
      String footName = fullRobotModel.getFoot(robotSide).getName();

      while (true)
      {
         int expectedNumberOfPointsInGenerator = Math.min(totalNumberOfPoints, RigidBodyTaskspaceControlState.maxPointsInGenerator);
         if (firstSegment)
            expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);
         int expectedPointsInQueue = totalNumberOfPoints - expectedNumberOfPointsInGenerator;
         EndToEndHandTrajectoryMessageTest.assertNumberOfWaypoints(footName, totalNumberOfPoints, scs);

         double lastPointTime = 0.0;
         fullRobotModel.updateFrames();

         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            FrameSE3TrajectoryPoint framePoint = footTrajectoryPoints.removeFirst();

            SimpleSE3TrajectoryPoint controllerTrajectoryPoint = EndToEndHandTrajectoryMessageTest.findTrajectoryPoint(footName, trajectoryPointIndex, scs);
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

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);

      // check internal desired matches last trajectory point:
      String nameSpacePositionDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnamePositionDesired = footName + Type.DESIRED.getName() + Space.POSITION.getName();
      Vector3D currentDesiredPosition = findVector3d(nameSpacePositionDesired, varnamePositionDesired, scs);

      String nameSpaceOrientationDesired = FeedbackControllerToolbox.class.getSimpleName();
      String varnameOrientationDesired = footName + Type.DESIRED.getName() + Space.ORIENTATION.getName();
      Quaternion currentDesiredOrientation = findQuat4d(nameSpaceOrientationDesired, varnameOrientationDesired, scs);

      EuclidCoreTestTools.assertTuple3DEquals(lastPoint.getPositionCopy().getPoint(), currentDesiredPosition, 0.001);
      EuclidCoreTestTools.assertQuaternionEquals(lastPoint.getOrientationCopy().getQuaternion(), currentDesiredOrientation, 0.001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 32.1)
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters,
            getRobotModel());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusXY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         List<FootTrajectoryMessage> messages = new ArrayList<>();

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            double rot = messageIndex / (numberOfMessages - 1.0) * 1.0 * Math.PI;

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
               if (robotSide == RobotSide.LEFT)
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(angle) * Math.sin(rot), radiusXY * Math.sin(angle) * Math.cos(rot),
                        radiusZ * Math.sin(2.0 * angle));
               else
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(2.0 * angle) * Math.sin(rot), radiusXY * Math.sin(angle),
                        radiusZ * Math.sin(2.0 * angle) * Math.cos(rot));
               tempPoint.add(circleCenter);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
            }
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            footTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
            {
               long previousMessageId = id - 1;
               if (messageIndex == numberOfMessages - 1)
                  previousMessageId = id + 100; // Bad ID

               footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, previousMessageId);
            }
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               footTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity,
                     desiredAngularVelocity, ReferenceFrame.getWorldFrame());
               calculatorIndex++;
            }

            messages.add(footTrajectoryMessage);
            drcSimulationTestHelper.send(footTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05 + getRobotModel().getControllerDT());
         assertTrue(success);

         String bodyName = fullRobotModel.getFoot(robotSide).getName();
         EndToEndHandTrajectoryMessageTest.assertNumberOfWaypoints(bodyName, 1, scs);
         //         assertNumberOfWaypoints(robotSide, 1, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.prependTranslation(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);
         trajectoryTime = 0.5;
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper
               .simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 38.0)
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getClass().getSimpleName(), selectedLocation, simulationTestingParameters,
            getRobotModel());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         // First need to pick up the foot:
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose footPoseCloseToActual = new FramePose(foot.getBodyFixedFrame());
         footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
         footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
         Point3D desiredPosition = new Point3D();
         Quaternion desiredOrientation = new Quaternion();
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);

         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 0.0, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);

         // Now we can do the usual test.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPoints = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
         FramePoint circleCenter = new FramePoint(ankleFrame);
         circleCenter.set(0.0, robotSide.negateIfRightSide(0.0), 0.10);
         double radiusXY = 0.15;
         double radiusZ = 0.08;
         FramePoint tempPoint = new FramePoint();
         FrameOrientation tempOrientation = new FrameOrientation(ankleFrame);
         tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

         List<FootTrajectoryMessage> messages = new ArrayList<>();

         EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            double rot = messageIndex / (numberOfMessages - 1.0) * 1.0 * Math.PI;

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
               if (robotSide == RobotSide.LEFT)
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(angle) * Math.sin(rot), radiusXY * Math.sin(angle) * Math.cos(rot),
                        radiusZ * Math.sin(2.0 * angle));
               else
                  tempPoint.setIncludingFrame(ankleFrame, radiusXY * Math.sin(2.0 * angle) * Math.sin(rot), radiusXY * Math.sin(angle),
                        radiusZ * Math.sin(2.0 * angle) * Math.cos(rot));
               tempPoint.add(circleCenter);
               tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
               euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint.getPoint());
            }
         }

         euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
         euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();

         int calculatorIndex = 0;
         long id = 4678L;

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPoints);
            footTrajectoryMessage.setUniqueId(id);
            if (messageIndex > 0)
               footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
            id++;
            double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

            for (int i = 0; i < numberOfTrajectoryPoints; i++)
            {
               desiredPosition = new Point3D();
               Vector3D desiredLinearVelocity = new Vector3D();
               desiredOrientation = new Quaternion();
               Vector3D desiredAngularVelocity = new Vector3D();
               tempOrientation.getQuaternion(desiredOrientation);

               double time = trajectoryPoints.get(calculatorIndex).get(desiredPosition, desiredLinearVelocity);

               Graphics3DObject sphere = new Graphics3DObject();
               sphere.translate(desiredPosition);
               sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
               scs.addStaticLinkGraphics(sphere);

               footTrajectoryMessage.setTrajectoryPoint(i, time - timeToSubtract, desiredPosition, desiredOrientation, desiredLinearVelocity,
                     desiredAngularVelocity, ReferenceFrame.getWorldFrame());
               calculatorIndex++;
            }

            messages.add(footTrajectoryMessage);
            drcSimulationTestHelper.send(footTrajectoryMessage);
            success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
            assertTrue(success);
         }

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);

         trajectoryTime = 1.0;
         FramePose desiredRandomFootPose = new FramePose(foot.getBodyFixedFrame());
         Random random = new Random(545L);
         desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
         desiredRandomFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
         desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

         desiredRandomFootPose.getPose(desiredPosition, desiredOrientation);
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
         assertTrue(success);

         String bodyName = fullRobotModel.getFoot(robotSide).getName();
         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(bodyName, desiredPosition, desiredOrientation, scs);
         //         assertSingleWaypointExecuted(robotSide, desiredPosition, desiredOrientation, scs);

         // Without forgetting to put the foot back on the ground
         footPoseCloseToActual.prependTranslation(0.0, 0.0, -0.15);
         footPoseCloseToActual.getPose(desiredPosition, desiredOrientation);
         trajectoryTime = 0.5;
         footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
         drcSimulationTestHelper.send(footTrajectoryMessage);

         success = drcSimulationTestHelper
               .simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
         assertTrue(success);
      }
   }

   public static Point3D findControllerDesiredPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentPositionVarNamePrefix = subTrajectoryName + "CurrentPosition";

      return findPoint3d(subTrajectoryName, currentPositionVarNamePrefix, scs);
   }

   public static Quaternion findControllerDesiredOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentOrientationVarNamePrefix = subTrajectoryName + "CurrentOrientation";

      return findQuat4d(subTrajectoryName, currentOrientationVarNamePrefix, scs);
   }

   public static Vector3D findControllerDesiredLinearVelocity(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentLinearVelocityVarNamePrefix = subTrajectoryName + "CurrentVelocity";

      return findVector3d(subTrajectoryName, currentLinearVelocityVarNamePrefix, scs);
   }

   public static Vector3D findControllerDesiredAngularVelocity(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String subTrajectoryName = footPrefix + "SubTrajectory";
      String currentAngularVelocityVarNamePrefix = subTrajectoryName + "CurrentAngularVelocity";

      return findVector3d(subTrajectoryName, currentAngularVelocityVarNamePrefix, scs);
   }

   public static int findNumberOfWaypointsForOrientation(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String numberOfWaypointsVarName = footPrefix + "NumberOfWaypoints";
      String orientationTrajectoryName = footPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();
      return ((YoInteger) scs.getVariable(orientationTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static int findNumberOfWaypointsForPosition(RobotSide robotSide, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String numberOfWaypointsVarName = footPrefix + "NumberOfWaypoints";
      String positionTrajectoryName = footPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      return ((YoInteger) scs.getVariable(positionTrajectoryName, numberOfWaypointsVarName)).getIntegerValue();
   }

   public static SimpleSE3TrajectoryPoint findTrajectoryPoint(RobotSide robotSide, int trajectoryPointIndex, SimulationConstructionSet scs)
   {
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String footPrefix = sidePrefix + "FootMoveViaWaypoints";
      String positionTrajectoryName = footPrefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String orientationTrajectoryName = footPrefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      String suffix = "AtWaypoint" + trajectoryPointIndex;

      String timeName = footPrefix + "Time";
      String positionName = footPrefix + "Position";
      String orientationName = footPrefix + "Orientation";
      String linearVelocityName = footPrefix + "LinearVelocity";
      String angularVelocityName = footPrefix + "AngularVelocity";

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
