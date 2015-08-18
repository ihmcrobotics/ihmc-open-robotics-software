package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDebrisEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspPieceOfDebrisBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.partNames.LimbName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCGraspPieceOfDebrisBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = true;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private ReferenceFrame midFeetZUpFrame;

   private final double POSITION_ERROR_MARGIN = 0.10;
   private final double ANGLE_ERROR_MARGIN = 0.5;

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();

   private DoubleYoVariable yoTime;

   private SDFRobot robot;
   private SDFFullHumanoidRobotModel fullRobotModel;

   @Before
   public void setUp()
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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
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
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

      midFeetZUpFrame = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();
   }

   private void getDebrisPoseInWorldAndSetGraspPose(final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior, RobotSide graspingSide)
   {
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);

      //this offset is very important because debrisTransform sent from the UI have the origin at the bottom of the debris, whereas here the robots have their origin at the center of the debris
      double zOffsetToHaveOriginAtDebrisBottom = testEnvironment.getDebrisLength() / 2.0;

      RigidBodyTransform debrisTransform = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransform);

      PoseReferenceFrame debrisReferenceFrame = new PoseReferenceFrame("debrisReferenceFrame", worldFrame);
      debrisReferenceFrame.setPoseAndUpdate(debrisTransform);

      FramePose debrisPose = new FramePose(debrisReferenceFrame);
      debrisPose.setZ(-zOffsetToHaveOriginAtDebrisBottom);
      debrisPose.changeFrame(worldFrame);

      FrameVector graspVector = new FrameVector(debrisReferenceFrame);
      graspVector.set(-1.0, 0.0, 0.0);
      graspVector.changeFrame(worldFrame);

      FramePoint graspVectorPosition = new FramePoint(debrisReferenceFrame);
      graspVectorPosition.setZ(0.6 - zOffsetToHaveOriginAtDebrisBottom);
      graspVectorPosition.changeFrame(worldFrame);

      debrisPose.getRigidBodyTransform(debrisTransform);

      graspPieceOfDebrisBehavior.setInput(debrisTransform, graspVectorPosition.getPointCopy(), graspVector.getVectorCopy(), graspingSide);
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

   private void assertChestAndPelvisAreCloseToHomePosition()
   {
    //Chest orientation
    ReferenceFrame chestReferenceFrame = fullRobotModel.getChest().getBodyFixedFrame();
    FramePose chestPose = new FramePose(chestReferenceFrame);
    chestPose.changeFrame(worldFrame);
    if (DEBUG)
    {
       System.out.println("chest pose");
       System.out.println(chestPose);
    }
    FramePose chestExpectedPose = new FramePose(worldFrame);
    chestExpectedPose.setPosition(-0.16, -0.02, 1.29);
    chestExpectedPose.setOrientation(0.0, 0.0, 0.0);
    assertTrue(chestPose.epsilonEquals(chestExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

    //pelvis pose
    ReferenceFrame pelvisReferenceFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
    FramePose pelvisPose = new FramePose(pelvisReferenceFrame);
    pelvisPose.changeFrame(worldFrame);
    if (DEBUG)
    {
       System.out.println("pelvis pose");
       System.out.println(pelvisPose);
    }

    FramePose pelvisExpectedPose = new FramePose(worldFrame);
    pelvisExpectedPose.setPosition(-0.09, -0.01, 0.79);
    pelvisExpectedPose.setOrientation(0.0, 0.0, 0.0);
    assertTrue(pelvisPose.epsilonEquals(pelvisExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));
   }

   private void assertHandsAreInTheGoodSpot(RobotSide testedSide)
   {
    //tested wrist Position
    ReferenceFrame testedSideHandReferenceFrame = fullRobotModel.getEndEffector(testedSide, LimbName.ARM).getBodyFixedFrame();
    FramePose testedSideHandPose = new FramePose(testedSideHandReferenceFrame);
    testedSideHandPose.changeFrame(worldFrame);
    if (DEBUG)
    {
       System.out.println("testedSide hand pose");
       System.out.println(testedSideHandPose);
    }
    FramePose testSideHandExpectedPose = new FramePose(worldFrame);
    testSideHandExpectedPose.setPosition(0.47, testedSide.negateIfRightSide(0.21), 1.20);
    testSideHandExpectedPose.setOrientation(1.56, 0.0, 0.13);
    assertTrue(testedSideHandPose.epsilonEquals(testSideHandExpectedPose, POSITION_ERROR_MARGIN, 2 * Math.PI)); // here orientation doesnt matter

    //oppositeSide wrist Position
    ReferenceFrame oppositeSideHandReferenceFrame = fullRobotModel.getEndEffector(testedSide.getOppositeSide(), LimbName.ARM).getBodyFixedFrame();
    FramePose oppositeSideHandPose = new FramePose(oppositeSideHandReferenceFrame);
    oppositeSideHandPose.changeFrame(worldFrame);
    if (DEBUG)
    {
       System.out.println("oppositeSide hand pose");
       System.out.println(oppositeSideHandPose);
    }
    FramePose oppositeSideHandExpectedPose = new FramePose(worldFrame);
    oppositeSideHandExpectedPose.setPosition(0.33, testedSide.getOppositeSide().negateIfRightSide(0.24), 1.06);
    oppositeSideHandExpectedPose.setOrientation(-1.78, 0.0, 0.0);
    assertTrue(oppositeSideHandPose.epsilonEquals(oppositeSideHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));
   }

   private void assertDebrisIsNotOnTheFloorAndOverAZThreshold()
   {
      ContactableSelectableBoxRobot debrisRobotAfterGrasping = testEnvironment.getEnvironmentRobots().get(0);
      Point3d debrisPositionAfterGrasping = new Point3d();
      debrisRobotAfterGrasping.getPosition(debrisPositionAfterGrasping);
      if (DEBUG)
         System.out.println(debrisPositionAfterGrasping);
      double zThreshold = 0.7;
      //debris is over a Z threshold
      assertTrue(debrisPositionAfterGrasping.getZ() > zThreshold);
   }
   
   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testGraspingStandingDebrisWithRightHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      testEnvironment.addStandingDebris(0.77, -0.2, Math.toRadians(-20.0));
      RobotSide testedSide = RobotSide.RIGHT;
      setUpDrcTestHelper();

      putArmsAtCompactHomePositionAndPrepareSimulation();

      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, midFeetZUpFrame, getRobotModel(), yoTime);

      graspPieceOfDebrisBehavior.initialize();

      getDebrisPoseInWorldAndSetGraspPose(graspPieceOfDebrisBehavior, testedSide);

      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());

      double graspTime = 13.0;
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(graspPieceOfDebrisBehavior, graspTime);

      assertTrue(graspPieceOfDebrisBehavior.isDone());

      assertDebrisIsNotOnTheFloorAndOverAZThreshold();

      assertHandsAreInTheGoodSpot(testedSide);
      
      assertChestAndPelvisAreCloseToHomePosition();
      
      BambooTools.reportTestFinishedMessage();
   }
   
   @Ignore
   @EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
   public void testGraspingStandingDebrisWithLeftHand() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      testEnvironment.addStandingDebris(0.75, 0.2, Math.toRadians(20.0));
      RobotSide testedSide = RobotSide.LEFT;
      setUpDrcTestHelper();
      
      putArmsAtCompactHomePositionAndPrepareSimulation();
      
      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, midFeetZUpFrame, getRobotModel(), yoTime);
      
      graspPieceOfDebrisBehavior.initialize();
      
      getDebrisPoseInWorldAndSetGraspPose(graspPieceOfDebrisBehavior, testedSide);
      
      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());
      
      double graspTime = 13.0;
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(graspPieceOfDebrisBehavior, graspTime);
      
      assertTrue(graspPieceOfDebrisBehavior.isDone());
      
      assertDebrisIsNotOnTheFloorAndOverAZThreshold();
      
      assertHandsAreInTheGoodSpot(testedSide);
      
      assertChestAndPelvisAreCloseToHomePosition();
      
      BambooTools.reportTestFinishedMessage();
   }

   @EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
   public void testGraspingHorizontalDebrisWithRightHand() throws SimulationExceededMaximumTimeException
   {
      testEnvironment.addHorizontalDebrisLeaningOnTwoBoxes(new Point3d(0.75, 0.0, 0.6), Math.toRadians(0.0), Math.toRadians(90.0)); // 20 ,-20
      RobotSide testedSide = RobotSide.RIGHT;
      setUpDrcTestHelper();
      
      putArmsAtCompactHomePositionAndPrepareSimulation();

      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, midFeetZUpFrame, getRobotModel(), yoTime);

      graspPieceOfDebrisBehavior.initialize();

      getDebrisPoseInWorldAndSetGraspPose(graspPieceOfDebrisBehavior, testedSide);

      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());

      double graspTime = 13.0;
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(graspPieceOfDebrisBehavior, graspTime);

      assertTrue(graspPieceOfDebrisBehavior.isDone());

      assertDebrisIsNotOnTheFloorAndOverAZThreshold();

      assertHandsAreInTheGoodSpot(testedSide);
      
      assertChestAndPelvisAreCloseToHomePosition();
   }

   @EstimatedDuration(duration = 70.0)
   @Test(timeout = 300000)
   public void testGraspingLeaningAgainstAWallDebrisWithRightHand() throws SimulationExceededMaximumTimeException
   {
      testEnvironment.addVerticalDebrisLeaningAgainstAWall(0.75, -0.35, Math.toRadians(-20.0), Math.toRadians(18.0));
      RobotSide testedSide = RobotSide.RIGHT;
      setUpDrcTestHelper();
      
      putArmsAtCompactHomePositionAndPrepareSimulation();

      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, midFeetZUpFrame, getRobotModel(), yoTime);

      graspPieceOfDebrisBehavior.initialize();

      getDebrisPoseInWorldAndSetGraspPose(graspPieceOfDebrisBehavior, testedSide);

      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());

      double graspTime = 13.0;
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(graspPieceOfDebrisBehavior, graspTime);

      assertTrue(graspPieceOfDebrisBehavior.isDone());

      assertDebrisIsNotOnTheFloorAndOverAZThreshold();

      assertHandsAreInTheGoodSpot(testedSide);
      
      assertChestAndPelvisAreCloseToHomePosition();
   }
}
