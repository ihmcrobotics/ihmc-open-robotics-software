package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDebrisEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RemoveSingleDebrisBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.humanoidRobot.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.PoseReferenceFrame;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCRemoveSingleDebrisBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = true;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double ROBOT_POSITION_TOLERANCE = 0.05;
   private final double POSITION_ERROR_MARGIN = 0.05;
   private final double ANGLE_ERROR_MARGIN = 0.05;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();

   private DoubleYoVariable yoTime;
   private SDFRobot robot;
   private SDFFullRobotModel fullRobotModel;
   private DRCRobotModel drcRobotModel;

   private ReferenceFrame midFeetZUpFrame;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   private void setUpDrcTestHelper()
   {
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            Vector3d additionalOffset = new Vector3d(0.0, 0.0, 0.0);
            double yaw = 0.0;
            OffsetAndYawRobotInitialSetup offsetAndYawRobotInitialSetup = new OffsetAndYawRobotInitialSetup(additionalOffset, yaw);
            return offsetAndYawRobotInitialSetup;
         }
      };

      testEnvironment.createDebrisContactController();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null, startingLocation,
            simulationTestingParameters, getRobotModel());

      yoTime = drcBehaviorTestHelper.getRobot().getYoTime();

      robot = drcBehaviorTestHelper.getRobot();
      drcRobotModel = getRobotModel();
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      midFeetZUpFrame = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
   }
   
   private void putArmsAtCompactHomePositionAndPrepareSimulation() throws SimulationExceededMaximumTimeException
   {
      //angles in AtlasDefaultArmConfigurations
      //right arm
      robot.getOneDegreeOfFreedomJoint("r_arm_shz").setQ(-0.1);
      robot.getOneDegreeOfFreedomJoint("r_arm_shx").setQ(1.6);
      robot.getOneDegreeOfFreedomJoint("r_arm_ely").setQ(1.9);
      robot.getOneDegreeOfFreedomJoint("r_arm_elx").setQ(-2.1);
      robot.getOneDegreeOfFreedomJoint("r_arm_wry").setQ(0.0);
      robot.getOneDegreeOfFreedomJoint("r_arm_wrx").setQ(0.55);
      //left arm
      robot.getOneDegreeOfFreedomJoint("l_arm_shz").setQ(0.1);
      robot.getOneDegreeOfFreedomJoint("l_arm_shx").setQ(-1.6);
      robot.getOneDegreeOfFreedomJoint("l_arm_ely").setQ(1.9);
      robot.getOneDegreeOfFreedomJoint("l_arm_elx").setQ(2.1);
      robot.getOneDegreeOfFreedomJoint("l_arm_wry").setQ(0.0);
      robot.getOneDegreeOfFreedomJoint("l_arm_wrx").setQ(-0.55);

      double[] rightArmDefaultConfigurationJointAngles = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, RobotSide.RIGHT);
      double[] leftArmDefaultConfigurationJointAngles = getRobotModel().getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, RobotSide.LEFT);

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.2));

      drcBehaviorTestHelper.send(new HandPosePacket(RobotSide.RIGHT, 0.5, rightArmDefaultConfigurationJointAngles));
      drcBehaviorTestHelper.send(new HandPosePacket(RobotSide.LEFT, 0.5, leftArmDefaultConfigurationJointAngles));
      drcBehaviorTestHelper.updateRobotModel();

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      drcBehaviorTestHelper.updateRobotModel();
   }
   
   private void assertHandsAreAtHomePosition()
   {
      //Right wrist Position
      ReferenceFrame rightHandReferenceFrame = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM).getBodyFixedFrame();
      FramePose rightHandPose = new FramePose(rightHandReferenceFrame);
      rightHandPose.changeFrame(midFeetZUpFrame);
      if (DEBUG)
      {
         System.out.println("right hand pose");
         System.out.println(rightHandPose);
      }
      FramePose rightHandExpectedPose = new FramePose(midFeetZUpFrame);
      rightHandExpectedPose.setPosition(0.39, -0.24, 1.02);
      rightHandExpectedPose.setOrientation(1.80, 0.0, 0.0);
      assertTrue(rightHandPose.epsilonEquals(rightHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      //Left wrist Position
      ReferenceFrame leftHandReferenceFrame = fullRobotModel.getEndEffector(RobotSide.LEFT, LimbName.ARM).getBodyFixedFrame();
      FramePose leftHandPose = new FramePose(leftHandReferenceFrame);
      leftHandPose.changeFrame(midFeetZUpFrame);
      if (DEBUG)
      {
         System.out.println("left hand pose");
         System.out.println(leftHandPose);
      }
      FramePose leftHandExpectedPose = new FramePose(midFeetZUpFrame);
      leftHandExpectedPose.setPosition(0.39, 0.24, 1.02);
      leftHandExpectedPose.setOrientation(-1.80, 0.0, 0.0);
      assertTrue(leftHandPose.epsilonEquals(leftHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));
   }

   private void assertTheDebrisIsNoMoreInFrontOfTheRobot(ContactableSelectableBoxRobot debrisRobot, double safeDistance)
   {
      //check debris Position in world
      RigidBodyTransform debrisTransformAfterRemove = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransformAfterRemove);
      FramePose debrisPoseAfterRemove = new FramePose(worldFrame, debrisTransformAfterRemove);
      assertTrue(debrisPoseAfterRemove.getY() < safeDistance);
   }

   private void assertRobotMovedCloserToTheDebris(RigidBodyTransform debrisTransformBeforeRemove)
   {
      //check that the robot got closer to the debris before it was removed
      FramePose debrisPoseBeforeRemove = new FramePose(worldFrame, debrisTransformBeforeRemove);
      debrisPoseBeforeRemove.changeFrame(midFeetZUpFrame);
      assertTrue(debrisPoseBeforeRemove.getX() <= (0.80 + ROBOT_POSITION_TOLERANCE));
   }

   private void assertPelvisAndChestAreAtHomePosition()
   {
      //Chest orientation
      ReferenceFrame chestReferenceFrame = fullRobotModel.getChest().getBodyFixedFrame();
      FramePose chestPose = new FramePose(chestReferenceFrame);
      chestPose.changeFrame(midFeetZUpFrame);
      if (DEBUG)
      {
         System.out.println("chest pose");
         System.out.println(chestPose);
      }
      FramePose chestExpectedPose = new FramePose(midFeetZUpFrame);
      chestExpectedPose.setPosition(-0.11, 0.0, 1.25);
      chestExpectedPose.setOrientation(0.0, 0.0, 0.0);
      assertTrue(chestPose.epsilonEquals(chestExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      //pelvis pose
      ReferenceFrame pelvisReferenceFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FramePose pelvisPose = new FramePose(pelvisReferenceFrame);
      pelvisPose.changeFrame(midFeetZUpFrame);
      if (DEBUG)
      {
         System.out.println("pelvis pose");
         System.out.println(pelvisPose);
      }

      FramePose pelvisExpectedPose = new FramePose(midFeetZUpFrame);
      pelvisExpectedPose.setPosition(-0.04, 0.0, 0.76);
      pelvisExpectedPose.setOrientation(0.0, 0.0, 0.0);
      assertTrue(pelvisPose.epsilonEquals(pelvisExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));
   }

   private RigidBodyTransform getDebrisPoseInWorldAndSetBehaviorInput(final RemoveSingleDebrisBehavior removeSingleDebrisBehavior,
         ContactableSelectableBoxRobot debrisRobot)
   {
      //this offset is very important because debrisTransform sent from the UI have the origin at the bottom of the debris, whereas here the robots have their origin at the center of the debris
      double zOffsetToHaveOriginAtDebrisBottom = testEnvironment.getDebrisLength() / 2.0;

      RigidBodyTransform debrisTransformBeforeRemove = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransformBeforeRemove);

      PoseReferenceFrame debrisReferenceFrame = new PoseReferenceFrame("debrisReferenceFrame", worldFrame);
      debrisReferenceFrame.setPoseAndUpdate(debrisTransformBeforeRemove);

      FramePose debrisPose = new FramePose(debrisReferenceFrame);
      debrisPose.setZ(-zOffsetToHaveOriginAtDebrisBottom);
      debrisPose.changeFrame(worldFrame);

      FrameVector graspVector = new FrameVector(debrisReferenceFrame);
      graspVector.set(-1.0, 0.0, 0.0);
      graspVector.changeFrame(worldFrame);

      FramePoint graspVectorPosition = new FramePoint(debrisReferenceFrame);
      graspVectorPosition.setZ(0.7 - zOffsetToHaveOriginAtDebrisBottom);
      graspVectorPosition.changeFrame(worldFrame);

      debrisPose.getRigidBodyTransform(debrisTransformBeforeRemove);

      removeSingleDebrisBehavior.setInputs(debrisTransformBeforeRemove, graspVectorPosition.getPointCopy(), graspVector.getVectorCopy());

      return debrisTransformBeforeRemove;
   }
   
   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testRemovingStandingDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      testEnvironment.addStandingDebris(1.0, -0.2, Math.toRadians(-20.0));
      
      setUpDrcTestHelper();
      
      putArmsAtCompactHomePositionAndPrepareSimulation();
      
      final RemoveSingleDebrisBehavior removeSingleDebrisBehavior = new RemoveSingleDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, drcBehaviorTestHelper.getReferenceFrames(), yoTime, drcRobotModel);

      removeSingleDebrisBehavior.initialize();

      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform debrisTransformBeforeRemove = getDebrisPoseInWorldAndSetBehaviorInput(removeSingleDebrisBehavior, debrisRobot);

      assertTrue(removeSingleDebrisBehavior.hasInputBeenSet());
      
      RobotSide graspingSide = removeSingleDebrisBehavior.getSideToUse();
      
      assertTrue(graspingSide == RobotSide.RIGHT);
      
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(removeSingleDebrisBehavior, 35.0);
      fullRobotModel.updateFrames();

      assertTrue(removeSingleDebrisBehavior.isDone());


      assertRobotMovedCloserToTheDebris(debrisTransformBeforeRemove);

      double safeDistance = -0.70;
      assertTheDebrisIsNoMoreInFrontOfTheRobot(debrisRobot, safeDistance);

      assertHandsAreAtHomePosition();

      assertPelvisAndChestAreAtHomePosition();

      BambooTools.reportTestFinishedMessage();
   }
   
   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testRemovingHorizontalDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      testEnvironment.addHorizontalDebrisLeaningOnTwoBoxes(new Point3d(1.0, 0.0, 0.6), Math.toRadians(0.0), Math.toRadians(90.0));      
      setUpDrcTestHelper();
      
      putArmsAtCompactHomePositionAndPrepareSimulation();
      
      final RemoveSingleDebrisBehavior removeSingleDebrisBehavior = new RemoveSingleDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, drcBehaviorTestHelper.getReferenceFrames(), yoTime, drcRobotModel);
      
      removeSingleDebrisBehavior.initialize();
      
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);
      
      RigidBodyTransform debrisTransformBeforeRemove = getDebrisPoseInWorldAndSetBehaviorInput(removeSingleDebrisBehavior, debrisRobot);
      
      assertTrue(removeSingleDebrisBehavior.hasInputBeenSet());
      
      RobotSide graspingSide = removeSingleDebrisBehavior.getSideToUse();
      
      assertTrue(graspingSide == RobotSide.RIGHT);
      
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(removeSingleDebrisBehavior, 35.0);
      fullRobotModel.updateFrames();
      
      assertTrue(removeSingleDebrisBehavior.isDone());
      
      
      assertRobotMovedCloserToTheDebris(debrisTransformBeforeRemove);
      
      double safeDistance = -0.80;
      assertTheDebrisIsNoMoreInFrontOfTheRobot(debrisRobot, safeDistance);
      
      assertHandsAreAtHomePosition();
      
      assertPelvisAndChestAreAtHomePosition();
      
      BambooTools.reportTestFinishedMessage();
   }
   
   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testRemovingLeaningOnAWallDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      testEnvironment.addVerticalDebrisLeaningAgainstAWall(1.0, -0.35, Math.toRadians(-20.0), Math.toRadians(22.0));      
      setUpDrcTestHelper();
      
      putArmsAtCompactHomePositionAndPrepareSimulation();
      
      final RemoveSingleDebrisBehavior removeSingleDebrisBehavior = new RemoveSingleDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, drcBehaviorTestHelper.getReferenceFrames(), yoTime, drcRobotModel);
      
      removeSingleDebrisBehavior.initialize();
      
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);
      
      RigidBodyTransform debrisTransformBeforeRemove = getDebrisPoseInWorldAndSetBehaviorInput(removeSingleDebrisBehavior, debrisRobot);
      
      assertTrue(removeSingleDebrisBehavior.hasInputBeenSet());
      
      RobotSide graspingSide = removeSingleDebrisBehavior.getSideToUse();
      
      assertTrue(graspingSide == RobotSide.RIGHT);
      
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(removeSingleDebrisBehavior, 35.0);
      fullRobotModel.updateFrames();
      
      assertTrue(removeSingleDebrisBehavior.isDone());
      
      assertRobotMovedCloserToTheDebris(debrisTransformBeforeRemove);
      
      double safeDistance = -0.80;
      assertTheDebrisIsNoMoreInFrontOfTheRobot(debrisRobot, safeDistance);
      
      assertHandsAreAtHomePosition();
      
      assertPelvisAndChestAreAtHomePosition();
      
      BambooTools.reportTestFinishedMessage();
   }
}
