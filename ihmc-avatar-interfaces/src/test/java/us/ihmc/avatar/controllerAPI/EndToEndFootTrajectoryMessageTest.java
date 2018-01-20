package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
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
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPoint;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.variable.YoBoolean;

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
   
   //This can be negative if you want to push the foot into the ground hard
   protected double getDesiredTouchDownHeightInWorld()
   {
      return 0.0;
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
      
      Point3D min = new Point3D();
      Point3D max = new Point3D();
      
      min.sub(circleCenterFromAnkle, circleRadius);
      max.add(circleCenterFromAnkle, circleRadius);
      
      return RandomGeometry.nextPoint3D(random, min, max);
   }
   
   //The first step in the test, send a FootTrajectoryMessage with a single point and simulate
   private boolean pickupFoot(RobotSide robotSide, RigidBody foot) throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double timeToPickupFoot = 1.0;

      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, getLiftOffHeight());
      footPoseCloseToActual.changeFrame(ReferenceFrame.getWorldFrame());
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, timeToPickupFoot, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);

      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeToPickupFoot + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
   }
   
   //Put the foot back on the ground, this doesn't have any special ground checks, it's just easier to read this way
   private boolean putFootOnGround(RobotSide robotSide, RigidBody foot, FramePose3D desiredPose) throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double trajectoryTime = 1.0;
      
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPose.setZ(getDesiredTouchDownHeightInWorld());
      desiredPose.get(desiredPosition, desiredOrientation);
      
      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);
      
      return drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.2 + trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
   }

   //moves the foot to a position using getRandomPositionInSphere
   private void moveFootToRandomPosition(Random random, RobotSide robotSide, RigidBody foot, SimulationConstructionSet scs)
         throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double trajectoryTime = 1.0;
      String bodyName = foot.getName();
      
      FramePose3D desiredRandomFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredRandomFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredRandomFootPose.setPosition(getRandomPositionInSphere(random, robotSide));
      desiredRandomFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredRandomFootPose.get(desiredPosition, desiredOrientation);
      FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);

      drcSimulationTestHelper.send(footTrajectoryMessage);

      boolean result = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(trajectoryTime + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(result);
      
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(bodyName, desiredPosition, desiredOrientation, scs);
   }
   

   //Picks up a foot, moves that foot to a position, and puts it down. Done using both sides
   @ContinuousIntegrationTest(estimatedDuration = 41.5)
   @Test(timeout = 30000)
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         
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
   @ContinuousIntegrationTest(estimatedDuration = 41.5)
   @Test(timeout = 70000)
   public void testPickUpAndPutDown() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));
      
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         
         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));
         
         // Without forgetting to put the foot back on the ground
         assertTrue(putFootOnGround(robotSide, foot, initialFootPosition));
      }
      
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }
   
   
   //picks up a foot, moves it around in a ribbon shape, then puts the foot down, Done using both sides
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         fullRobotModel.updateFrames();
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
         
         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));

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

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = createTrajectory(robotSide, foot, firstTrajectoryPointTime, numberOfTrajectoryPoints, 1, trajectoryTime);
         
         ArrayDeque<FrameSE3TrajectoryPoint> frameSE3TrajectoryPoints = sendQueuedFootTrajectoryMessages(scs, robotSide, 1, trajectoryPoints);

         int expectedNumberOfPointsInGenerator = Math.min(RigidBodyTaskspaceControlState.maxPointsInGenerator, numberOfTrajectoryPoints + 1);

         EndToEndHandTrajectoryMessageTest.assertNumberOfWaypoints(footName, numberOfTrajectoryPoints + 1, scs);

         FrameSE3TrajectoryPoint lastFramePoint = frameSE3TrajectoryPoints.peekLast();
         for (int trajectoryPointIndex = 1; trajectoryPointIndex < expectedNumberOfPointsInGenerator; trajectoryPointIndex++)
         {
            FrameSE3TrajectoryPoint framePoint = frameSE3TrajectoryPoints.removeFirst();

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
        putFootOnGround(robotSide, foot, initialFootPosition);
      }
      
      drcSimulationTestHelper.createVideo(robotModel.getSimpleRobotName(), 2);
   }

   //moves each foot to a single position using a custom control point
   public void testCustomControlPoint() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
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
      ReferenceFrame controlFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("ControlFrame", footFixedFrame, controlFrameTransform);

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
      
      drcSimulationTestHelper.createVideo(robotModel.getSimpleRobotName(), 2);
   }
   
   //picks up the left foot, moves the foot around a sphere (ribbons yawed around the circle center)
   public void testQueuedMessages() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      RobotSide robotSide = RobotSide.LEFT;
      RigidBody foot = fullRobotModel.getFoot(robotSide);
      
      // First need to pick up the foot:
      assertTrue(pickupFoot(robotSide, foot));

      // Now we can do the usual test.
      double firstTrajectoryPointTime = 0.5;
      int numberOfTrajectoryPointsPerMessage = 20;
      int numberOfMessages = 10;
      double trajectoryTime = 12.0;

      ReferenceFrame ankleFrame = foot.getParentJoint().getFrameAfterJoint();
      FrameQuaternion tempOrientation = new FrameQuaternion(ankleFrame);
      tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = createTrajectory(robotSide, foot, firstTrajectoryPointTime, numberOfTrajectoryPointsPerMessage, numberOfMessages, trajectoryTime);
      ArrayDeque<FrameSE3TrajectoryPoint> footTrajectoryPoints = sendQueuedFootTrajectoryMessages(scs, robotSide, numberOfMessages, trajectoryPoints);

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

      EuclidCoreTestTools.assertTuple3DEquals(lastPoint.getPositionCopy(), currentDesiredPosition, 0.001);
      EuclidCoreTestTools.assertQuaternionEquals(lastPoint.getOrientationCopy(), currentDesiredOrientation, 0.001);
      
      drcSimulationTestHelper.createVideo(robotModel.getSimpleRobotName(), 2);
   }

   //takes the trajectory points created from the eucledian traj generator and divides them up by the number of messages desired and sends them to the controller
   private ArrayDeque<FrameSE3TrajectoryPoint> sendQueuedFootTrajectoryMessages(SimulationConstructionSet scs, RobotSide robotSide, int numberOfMessages,
         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints) throws SimulationExceededMaximumTimeException
   {
      ArrayDeque<FrameSE3TrajectoryPoint> footTrajectoryPoints = new ArrayDeque<>();
      int numberOfTrajectoryPointsPerMessage = (int) Math.ceil(trajectoryPoints.size() / (double) numberOfMessages);
      int calculatorIndex = 0;
      long id = 4678L;
      List<FootTrajectoryMessage> messages = new ArrayList<>();

      for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
      {
         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, numberOfTrajectoryPointsPerMessage);
         footTrajectoryMessage.setUniqueId(id);
         if (messageIndex > 0)
            footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, id - 1);
         id++;
         double timeToSubtract = messageIndex == 0 ? 0.0 : trajectoryPoints.get(calculatorIndex - 1).getTime();

         for (int i = 0; i < numberOfTrajectoryPointsPerMessage; i++)
         {
            Point3D desiredPosition = new Point3D();
            Vector3D desiredLinearVelocity = new Vector3D();
            Quaternion desiredOrientation = new Quaternion();
            Vector3D desiredAngularVelocity = new Vector3D();

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
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);
      }

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
      assertTrue(success);
      return footTrajectoryPoints;
   }
   
   //picks the foot up, sends queued messages, the last with the wrong previous queued message ID. (Should see sysout about this) Checks that the number of waypoints is cleared, then puts the foot back on the ground. Done for both sides
   @ContinuousIntegrationTest(estimatedDuration = 32.1)
   @Test(timeout = 30000)
   public void testQueueWithWrongPreviousId() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
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

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = createTrajectory(robotSide, foot, firstTrajectoryPointTime,
               numberOfTrajectoryPoints, numberOfMessages, trajectoryTime);

         sendQueuedFootTrajectoryMessages(scs, robotSide, numberOfMessages, trajectoryPoints);
         
         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage(robotSide, 5);
         footTrajectoryMessage.setUniqueId(100);
         footTrajectoryMessage.setExecutionMode(ExecutionMode.QUEUE, 500); //not the right ID
         for(int i = 0; i < 5; i++)
         {
            footTrajectoryMessage.setTrajectoryPoint(i, i, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D(), ReferenceFrame.getWorldFrame());
         }
         drcSimulationTestHelper.send(footTrajectoryMessage);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 2);
         assertTrue(success);
         
         String bodyName = fullRobotModel.getFoot(robotSide).getName();
         EndToEndHandTrajectoryMessageTest.assertNumberOfWaypoints(bodyName, 1, scs);
         
         assertTrue(putFootOnGround(robotSide, foot, initialFootPosition));
      }
   }

   //Creates a trajectory for the foot, depending on the number of iterations, it looks like a ribbon or a sphere
   private RecyclingArrayList<FrameEuclideanTrajectoryPoint> createTrajectory(RobotSide robotSide, RigidBody foot, double firstTrajectoryPointTime,
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

      for (int messageIndex = 0; messageIndex < numberOfIterations; messageIndex++)
      {
         double rot = messageIndex / (numberOfIterations - 1.0) * 1.0 * Math.PI;
         if(Double.isNaN(rot))
         {
            rot = 0;
         }

         for (int i = 0; i < numberOfTrajectoryPoints; i++)
         {
            double angle = i / ((double) numberOfTrajectoryPoints) * 2.0 * Math.PI;
            if (robotSide == RobotSide.LEFT)
               tempPoint.setIncludingFrame(ankleFrame, circleRadius.getX() * Math.sin(angle) * Math.sin(rot), circleRadius.getY() * Math.sin(angle) * Math.cos(rot),
                     circleRadius.getZ() * Math.sin(2.0 * angle));
            else
               tempPoint.setIncludingFrame(ankleFrame, circleRadius.getX() * Math.sin(2.0 * angle) * Math.sin(rot), circleRadius.getY() * Math.sin(angle),
                     circleRadius.getZ() * Math.sin(2.0 * angle) * Math.cos(rot));
            tempPoint.add(circleCenter);
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(tempPoint);
         }
      }

      euclideanTrajectoryPointCalculator.computeTrajectoryPointTimes(firstTrajectoryPointTime, trajectoryTime);
      euclideanTrajectoryPointCalculator.computeTrajectoryPointVelocities(true);

      RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = euclideanTrajectoryPointCalculator.getTrajectoryPoints();
      return trajectoryPoints;
   }

   //Picks a foot up, Sends queued messages, then sends a single point to test overriding the queue. Done for both feet
   public void testQueueStoppedWithOverrideMessage() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(564574L);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.0, 0.0, 0.5), new Point3D(6.0, 2.0, 2.0));

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
         
         // First need to pick up the foot:
         assertTrue(pickupFoot(robotSide, foot));
         
         // send the queued messages.
         double firstTrajectoryPointTime = 0.5;
         int numberOfTrajectoryPointsPerMessage = 20;
         int numberOfMessages = 10;
         double trajectoryTime = 12.0;

         RecyclingArrayList<FrameEuclideanTrajectoryPoint> trajectoryPoints = createTrajectory(robotSide, foot, firstTrajectoryPointTime, numberOfTrajectoryPointsPerMessage, 1, trajectoryTime);
         sendQueuedFootTrajectoryMessages(scs, robotSide, numberOfMessages, trajectoryPoints);

         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT());
         assertTrue(success);

         //send a single trajectory point message in override mode
         moveFootToRandomPosition(random, robotSide, foot, scs);

         // Without forgetting to put the foot back on the ground
         putFootOnGround(robotSide, foot, initialFootPosition);
      }
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
