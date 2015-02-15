package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.util.NetworkConfigParameters;
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
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.UnfinishedTest;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations.ArmConfigurations;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

@UnfinishedTest
public abstract class DRCRemoveSingleDebrisBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double ROBOT_POSITION_TOLERANCE = 0.05;
   private final double POSITION_ERROR_MARGIN = 0.025;
   private final double ANGLE_ERROR_MARGIN = 0.05;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private void showMemoryUsageBeforeTest()
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
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();

   private DoubleYoVariable yoTime;
   private SDFRobot robot;
   private SDFFullRobotModel fullRobotModel;
   private DRCRobotModel drcRobotModel;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      showMemoryUsageBeforeTest();

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

      testEnvironment.addStandingDebris(1.0, -0.35, 0.0);
      testEnvironment.createDebrisContactController();

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null, startingLocation,
            simulationTestingParameters, getRobotModel(), controllerCommunicator);

      yoTime = drcBehaviorTestHelper.getRobot().getYoTime();

      robot = drcBehaviorTestHelper.getRobot();
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      drcRobotModel = getRobotModel();
   }

   @AverageDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testRemovingDebrisOnRightSide() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

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

      double[] rightArmDefaultConfigurationJointAngles = drcRobotModel.getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, RobotSide.RIGHT);
      double[] leftArmDefaultConfigurationJointAngles = drcRobotModel.getDefaultArmConfigurations().getArmDefaultConfigurationJointAngles(
            ArmConfigurations.COMPACT_HOME, RobotSide.LEFT);

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.2));

      drcBehaviorTestHelper.sendHandPosePacketToListeners(new HandPosePacket(RobotSide.RIGHT, 0.5, rightArmDefaultConfigurationJointAngles));
      drcBehaviorTestHelper.sendHandPosePacketToListeners(new HandPosePacket(RobotSide.LEFT, 0.5, leftArmDefaultConfigurationJointAngles));
      drcBehaviorTestHelper.updateRobotModel();

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      drcBehaviorTestHelper.updateRobotModel();

      final RemoveSingleDebrisBehavior removeSingleDebrisBehavior = new RemoveSingleDebrisBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
            fullRobotModel, drcBehaviorTestHelper.getReferenceFrames(), yoTime, drcRobotModel, drcRobotModel.getWalkingControllerParameters());

      removeSingleDebrisBehavior.initialize();

      // from DebrisTaskBehaviorPanel.storeDebrisDataInList
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform debrisTransformBeforeRemove = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransformBeforeRemove);
      debrisTransformBeforeRemove.applyTranslation(new Vector3d(0.0, 0.0, -testEnvironment.getDebrisLength() / 2.0));

      FrameVector tempGraspVector = new FrameVector(worldFrame);
      tempGraspVector.set(-1.0, 0.0, 0.0);
      tempGraspVector.applyTransform(debrisTransformBeforeRemove);
      Vector3d graspVector = new Vector3d();
      tempGraspVector.get(graspVector);

      FramePoint tempGraspVectorPosition = new FramePoint(worldFrame);
      tempGraspVectorPosition.setZ(0.6);
      tempGraspVectorPosition.applyTransform(debrisTransformBeforeRemove);
      Point3d graspVectorPosition = new Point3d();
      tempGraspVectorPosition.get(graspVectorPosition);

      removeSingleDebrisBehavior.setInputs(debrisTransformBeforeRemove, graspVectorPosition, graspVector);

      assertTrue(removeSingleDebrisBehavior.hasInputBeenSet());

      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(removeSingleDebrisBehavior, 34.0);
      fullRobotModel.updateFrames();

      assertTrue(removeSingleDebrisBehavior.isDone());

      RigidBodyTransform debrisTransformAfterRemove = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransformAfterRemove);

      //check that the robot got closer to the debris before it was removed
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      FramePose debrisPoseBeforeRemove = new FramePose(worldFrame, debrisTransformBeforeRemove);
      debrisPoseBeforeRemove.changeFrame(chestFrame);
      assertTrue(debrisPoseBeforeRemove.getX() <= (0.75 + ROBOT_POSITION_TOLERANCE));

      //check debris Position in world
      FramePose debrisPoseAfterRemove = new FramePose(worldFrame, debrisTransformAfterRemove);
      assertTrue(debrisPoseAfterRemove.getY() < -0.80);

      //Right wrist Position
      ReferenceFrame rightHandReferenceFrame = fullRobotModel.getEndEffector(RobotSide.RIGHT, LimbName.ARM).getBodyFixedFrame();
      FramePose rightHandPose = new FramePose(rightHandReferenceFrame);
      rightHandPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("right hand pose");
         System.out.println(rightHandPose);
      }
      FramePose rightHandExpectedPose = new FramePose(worldFrame);
      rightHandExpectedPose.setPosition(0.73, -0.24, 1.05);
      rightHandExpectedPose.setOrientation(1.80, 0.0, 0.0);
      assertTrue(rightHandPose.epsilonEquals(rightHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      //Left wrist Position
      ReferenceFrame leftHandReferenceFrame = fullRobotModel.getEndEffector(RobotSide.LEFT, LimbName.ARM).getBodyFixedFrame();
      FramePose leftHandPose = new FramePose(leftHandReferenceFrame);
      leftHandPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("left hand pose");
         System.out.println(leftHandPose);
      }
      FramePose leftHandExpectedPose = new FramePose(worldFrame);
      leftHandExpectedPose.setPosition(0.73, 0.24, 1.05);
      leftHandExpectedPose.setOrientation(-1.80, 0.0, 0.0);
      assertTrue(leftHandPose.epsilonEquals(leftHandExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

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
      chestExpectedPose.setPosition(0.22, 0.0, 1.29);
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
      pelvisExpectedPose.setPosition(0.30, 0.0, 0.79);
      pelvisExpectedPose.setOrientation(0.0, 0.0, 0.0);
      assertTrue(pelvisPose.epsilonEquals(pelvisExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      BambooTools.reportTestFinishedMessage();
   }
}
