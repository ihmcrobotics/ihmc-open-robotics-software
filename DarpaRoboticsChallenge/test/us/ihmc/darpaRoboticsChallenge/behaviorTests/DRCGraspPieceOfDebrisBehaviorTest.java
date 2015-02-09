package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDebrisEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspPieceOfDebrisBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCGraspPieceOfDebrisBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private ReferenceFrame midFeetZUpFrame;

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final boolean DEBUG = false;

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.0;

   private final double FINGER1_JOINT_1_EXPECTED_RADIANS = 0.37;
   private final double FINGER1_JOINT_2_EXPECTED_RADIANS = 1.05;
   private final double FINGER1_JOINT_3_EXPECTED_RADIANS = 0.24;

   private final double FINGER2_JOINT_1_EXPECTED_RADIANS = 0.37;
   private final double FINGER2_JOINT_2_EXPECTED_RADIANS = 1.05;
   private final double FINGER2_JOINT_3_EXPECTED_RADIANS = 0.24;

   private final double MIDDLEFINGER_JOINT_1_EXPECTED_RADIANS = 0.48;
   private final double MIDDLEFINGER_JOINT_2_EXPECTED_RADIANS = 1.02;
   private final double MIDDLEFINGER_JOINT_3_EXPECTED_RADIANS = 0.25;

   private final double FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS = 0.05;

   private final double POSITION_ERROR_MARGIN = 0.02;
   private final double ANGLE_ERROR_MARGIN = 0.05;

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();
   private final KryoLocalPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
         "DRCGraspPieceOfDebrisBehaviorTestControllerCommunicator");

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private SDFRobot robot;
   private SDFFullRobotModel fullRobotModel;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

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

      testEnvironment.addStandingDebris(0.65, -0.30, 0.0); //0.65
      testEnvironment.createDebrisContactController();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, controllerCommunicator, getSimpleRobotName(), null, startingLocation,
            simulationTestingParameters, false, getRobotModel(), controllerCommunicator);

      Robot robotToTest = drcBehaviorTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      robot = drcBehaviorTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();

      midFeetZUpFrame = drcBehaviorTestHelper.getReferenceFrames().getMidFeetZUpFrame();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      controllerCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      PacketCommunicator junkyObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), 10,
            "DRCComHeightBehaviorTestJunkyCommunicator");

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, controllerCommunicator, robotToTest.getRobotsYoVariableRegistry());
   }

   @AverageDuration(duration = 300.0)
   @Test(timeout = 900000)
   public void testGraspingDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      //angles in AtlasDefaultArmConfigurations
      robot.getOneDegreeOfFreedomJoint("r_arm_shz").setQ(-0.1);
      robot.getOneDegreeOfFreedomJoint("r_arm_shx").setQ(1.6);
      robot.getOneDegreeOfFreedomJoint("r_arm_ely").setQ(1.9);
      robot.getOneDegreeOfFreedomJoint("r_arm_elx").setQ(-2.1);
      robot.getOneDegreeOfFreedomJoint("r_arm_wry").setQ(0.0);
      robot.getOneDegreeOfFreedomJoint("r_arm_wrx").setQ(0.55);

      double[] jointAngles = new double[6];
      jointAngles[0] = -0.1;
      jointAngles[1] = 1.6;
      jointAngles[2] = 1.9;
      jointAngles[3] = -2.1;
      jointAngles[4] = 0.0;
      jointAngles[5] = 0.55;

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      
      controllerCommunicator.send(new HandPosePacket(RobotSide.RIGHT, jointAngles));
      
      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      final GraspPieceOfDebrisBehavior graspPieceOfDebrisBehavior = new GraspPieceOfDebrisBehavior(communicationBridge, fullRobotModel, midFeetZUpFrame,
            getRobotModel(), yoTime);
      communicationBridge.attachGlobalListenerToController(graspPieceOfDebrisBehavior.getControllerGlobalPacketConsumer());

      graspPieceOfDebrisBehavior.initialize();

      // from DebrisTaskBehaviorPanel.storeDebrisDataInList
      ContactableSelectableBoxRobot debrisRobot = testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform debrisTransform = new RigidBodyTransform();
      debrisRobot.getBodyTransformToWorld(debrisTransform);
      debrisTransform.applyTranslation(new Vector3d(0.0, 0.0, -testEnvironment.getDebrisLength() / 2.0));

      FrameVector tempGraspVector = new FrameVector(worldFrame);
      tempGraspVector.set(-1.0, 0.0, 0.0);
      tempGraspVector.applyTransform(debrisTransform);
      Vector3d graspVector = new Vector3d();
      tempGraspVector.get(graspVector);

      FramePoint tempGraspVectorPosition = new FramePoint(worldFrame);
      tempGraspVectorPosition.setZ(0.5);
      tempGraspVectorPosition.applyTransform(debrisTransform);
      Point3d graspVectorPosition = new Point3d();
      tempGraspVectorPosition.get(graspVectorPosition);

      graspPieceOfDebrisBehavior.setGraspPose(debrisTransform, graspVectorPosition, graspVector, RobotSide.RIGHT);

      assertTrue(graspPieceOfDebrisBehavior.hasInputBeenSet());

      double graspTime = 20.0;
      executeBehavior(graspPieceOfDebrisBehavior, graspTime);
      assertTrue(graspPieceOfDebrisBehavior.isDone());

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      DoubleYoVariable finger1_Joint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_1");
      DoubleYoVariable finger1_Joint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_2");
      DoubleYoVariable finger1_Joint3 = (DoubleYoVariable) robot.getVariable("q_r_finger_1_joint_3");

      DoubleYoVariable finger2_Joint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_1");
      DoubleYoVariable finger2_Joint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_2");
      DoubleYoVariable finger2_Joint3 = (DoubleYoVariable) robot.getVariable("q_r_finger_2_joint_3");

      DoubleYoVariable fingerMiddle_Joint1 = (DoubleYoVariable) robot.getVariable("q_r_finger_middle_joint_1");
      DoubleYoVariable fingerMiddle_Joint2 = (DoubleYoVariable) robot.getVariable("q_r_finger_middle_joint_2");
      DoubleYoVariable fingerMiddle_Joint3 = (DoubleYoVariable) robot.getVariable("q_r_finger_middle_joint_3");

      assertTrue(Math.abs(finger1_Joint1.getDoubleValue() - FINGER1_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger1_Joint2.getDoubleValue() - FINGER1_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger1_Joint3.getDoubleValue() - FINGER1_JOINT_3_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);

      assertTrue(Math.abs(finger2_Joint1.getDoubleValue() - FINGER2_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger2_Joint2.getDoubleValue() - FINGER2_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(finger2_Joint3.getDoubleValue() - FINGER2_JOINT_3_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);

      assertTrue(Math.abs(fingerMiddle_Joint1.getDoubleValue() - MIDDLEFINGER_JOINT_1_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(fingerMiddle_Joint2.getDoubleValue() - MIDDLEFINGER_JOINT_2_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);
      assertTrue(Math.abs(fingerMiddle_Joint3.getDoubleValue() - MIDDLEFINGER_JOINT_3_EXPECTED_RADIANS) < FINGER_JOINT_ANGLE_ERROR_MARGIN_RADIANS);

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
      rightHandExpectedPose.setPosition(0.45, -0.28, 1.21);
      rightHandExpectedPose.setOrientation(1.27, 0.07, 0.06);
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
      leftHandExpectedPose.setPosition(0.33, 0.33, 0.74);
      leftHandExpectedPose.setOrientation(-1.85, 0.25, -0.53);
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
      chestExpectedPose.setPosition(-0.17, -0.02, 1.28);
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
      pelvisExpectedPose.setPosition(-0.09, -0.02, 0.79);
      pelvisExpectedPose.setOrientation(0.0, 0.0, 0.0);
      assertTrue(pelvisPose.epsilonEquals(pelvisExpectedPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      BambooTools.reportTestFinishedMessage();
   }

   private boolean executeBehavior(final BehaviorInterface behavior, double trajectoryTime) throws SimulationExceededMaximumTimeException
   {
      final double simulationRunTime = trajectoryTime + EXTRA_SIM_TIME_FOR_SETTLING;

      SysoutTool.println("\n starting behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue(), DEBUG);

      Thread behaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;

               while (simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }

                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;

                  behavior.doControl();
               }
            }
         }
      };

      behaviorThread.start();

      boolean ret = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      SysoutTool.println("done simulating behavior: " + behavior.getName() + "   t = " + yoTime.getDoubleValue(), DEBUG);

      return ret;
   }

}
