package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.FileNotFoundException;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCValveEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.RotateHandAboutAxisBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableValveRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import us.ihmc.utilities.Axis;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;

import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCRotateHandAboutAxisBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();
      
      

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private static final boolean DEBUG = false;

   @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(createTestEnvironment(), networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel(), controllerCommunicator);
   }

   private static DRCValveEnvironment createTestEnvironment()
   {
      double valveX = 1.0 * TurnValveBehavior.howFarToStandBackFromValve;
      double valveY = TurnValveBehavior.howFarToStandToTheRightOfValve;
      double valveZ = 1.0;
      double valveYaw_degrees = 0.0 * 45.0;

      return new DRCValveEnvironment(valveX, valveY, valveZ, valveYaw_degrees);
   }

   @AverageDuration(duration = 50.0)
   @Test(timeout = 300000)
   public void testSimpleRotate() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      CommonAvatarEnvironmentInterface testEnvironment = drcBehaviorTestHelper.getTestEnviroment();
      ContactableValveRobot valveRobot = (ContactableValveRobot) testEnvironment.getEnvironmentRobots().get(0);

      RigidBodyTransform valveTransformToWorld = new RigidBodyTransform();
      valveRobot.getBodyTransformToWorld(valveTransformToWorld);

      FramePose valvePose = new FramePose(ReferenceFrame.getWorldFrame(), valveTransformToWorld);
      SysoutTool.println("Valve Pose = " + valvePose, DEBUG);
      SysoutTool.println("Robot Pose = " + getRobotPose(drcBehaviorTestHelper.getReferenceFrames()), DEBUG);

      final RotateHandAboutAxisBehavior rotateHandBehavior = createNewRotateHandBehavior();

      RobotSide robotSide = RobotSide.RIGHT;
      Axis pinJointAxisInGraspedObjectFrame = Axis.X;
      FramePose initialHandPoseInWorld = getCurrentHandPose(robotSide);
      double turnAngleRad = Math.toRadians(90.0);
      double trajectoryTime = 2.0;

      rotateHandBehavior.initialize();
      rotateHandBehavior.setInput(robotSide, initialHandPoseInWorld, pinJointAxisInGraspedObjectFrame, valveTransformToWorld, turnAngleRad, trajectoryTime);

      success = drcBehaviorTestHelper.executeBehaviorUntilDone(rotateHandBehavior);

      drcBehaviorTestHelper.createMovie(getSimpleRobotName(), 1);

      success = success & rotateHandBehavior.isDone();
      assertTrue(success);

      BambooTools.reportTestFinishedMessage();
   }

   private RotateHandAboutAxisBehavior createNewRotateHandBehavior()
   {
      BehaviorCommunicationBridge communicationBridge = drcBehaviorTestHelper.getBehaviorCommunicationBridge();
      FullRobotModel fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      ReferenceFrames referenceFrames = drcBehaviorTestHelper.getReferenceFrames();
      DoubleYoVariable yoTime = drcBehaviorTestHelper.getYoTime();

      RotateHandAboutAxisBehavior ret = new RotateHandAboutAxisBehavior(communicationBridge, fullRobotModel, referenceFrames, yoTime);
      return ret;
   }

   private FramePose getCurrentHandPose(RobotSide robotSideToTest)
   {
      FramePose ret = new FramePose();
      drcBehaviorTestHelper.updateRobotModel();
      ret.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSideToTest));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }

   private FramePose getRobotPose(ReferenceFrames referenceFrames)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      FramePose ret = new FramePose();

      drcBehaviorTestHelper.updateRobotModel();
      ReferenceFrame midFeetFrame = referenceFrames.getMidFeetZUpFrame();

      ret.setToZero(midFeetFrame);
      ret.changeFrame(worldFrame);

      return ret;
   }

}
