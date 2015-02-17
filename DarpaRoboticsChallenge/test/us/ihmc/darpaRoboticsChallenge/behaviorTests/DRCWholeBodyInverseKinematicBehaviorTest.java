package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.ExcludedTest;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

@ExcludedTest
public abstract class DRCWholeBodyInverseKinematicBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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

   private final double EXTRA_SIM_TIME_FOR_SETTLING = 1.5;

   private final double POSITION_ERROR_MARGIN = 0.03;
   private final double ANGLE_ERROR_MARGIN = 0.1;

   private final DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();

   private DoubleYoVariable yoTime;

   private SDFFullRobotModel fullRobotModel;
   private WholeBodyControllerParameters wholeBodyControllerParameters;

   private final double DELTA_YAW = 0.2;//0.05;
   private final double DELTA_PITCH = 0.5;//0.08;
   private final double DELTA_ROLL = Math.PI;

      @Before
   public void setUp()
   {
      if (NetworkConfigParameters.USE_BEHAVIORS_MODULE)
      {
         throw new RuntimeException("Must set NetworkConfigParameters.USE_BEHAVIORS_MODULE = false in order to perform this test!");
      }

      showMemoryUsageBeforeTest();

      KryoPacketCommunicator controllerCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.CONTROLLER.ordinal(), "DRCControllerCommunicator");
      KryoPacketCommunicator networkObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.NETWORK_PROCESSOR.ordinal(), "MockNetworkProcessorCommunicator");

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel(), controllerCommunicator);

      yoTime = drcBehaviorTestHelper.getRobot().getYoTime();

      wholeBodyControllerParameters = getRobotModel();
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();

   }

   @AverageDuration(duration = 90.0)
   @Test(timeout = 300000)
   public void testRandomRightHandPose() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(EXTRA_SIM_TIME_FOR_SETTLING));

      drcBehaviorTestHelper.updateRobotModel();

      final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior = new WholeBodyInverseKinematicBehavior(
            drcBehaviorTestHelper.getBehaviorCommunicationBridge(), wholeBodyControllerParameters, fullRobotModel, yoTime);

      wholeBodyIKBehavior.initialize();

      wholeBodyIKBehavior.setPositionAndOrientationErrorTolerance(POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN);

      RobotSide side = RobotSide.RIGHT;

      double trajectoryDuration = 4.0;

      double[] xyzMin = new double[3];
      double[] xyzMax = new double[3];
      double[] yawPitchRollMin = new double[3];
      double[] yawPitchRollMax = new double[3];

      generatePositionAndAngleLimits(side, xyzMin, xyzMax, yawPitchRollMin, yawPitchRollMax);

      Random random = new Random();
      FramePose desiredHandPose = RandomTools.generateRandomFramePose(random, worldFrame, xyzMin, xyzMax, yawPitchRollMin, yawPitchRollMax);

      if (DEBUG)
      {
         System.out.println("desired hand Pose");
         System.out.println(desiredHandPose);
         System.out.println("  ");
      }
      wholeBodyIKBehavior.setInputs(side, desiredHandPose, trajectoryDuration , 5, ControlledDoF.DOF_3P3R);
      assertTrue(wholeBodyIKBehavior.hasInputBeenSet());

      wholeBodyIKBehavior.computeSolution();
      assertTrue(wholeBodyIKBehavior.hasSolutionBeenFound());
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(wholeBodyIKBehavior, trajectoryDuration);

      assertTrue(wholeBodyIKBehavior.isDone());

      //Right wrist Position
      ReferenceFrame rightHandReferenceFrame = fullRobotModel.getHandControlFrame(side);
      FramePose rightHandPose = new FramePose(rightHandReferenceFrame);
      rightHandPose.changeFrame(worldFrame);
      if (DEBUG)
      {
         System.out.println("!!!!!!!!!!!!!!!!!!! MOVEMENT HAPPENED !!!!!!!!!!!!!!!!!!!");
         System.out.println("desired right hand pose");
         System.out.println(desiredHandPose);
         System.out.println("    ");

         System.out.println("right hand pose");
         System.out.println(rightHandPose);
      }
      assertTrue(rightHandPose.epsilonEquals(desiredHandPose, POSITION_ERROR_MARGIN, ANGLE_ERROR_MARGIN));

      BambooTools.reportTestFinishedMessage();
   }

   private void generatePositionAndAngleLimits(RobotSide side, double[] xyzMin, double[] xyzMax, double[] yawPitchRollMin, double[] yawPitchRollMax)
   {
      xyzMin[0] = 0.39;
      xyzMax[0] = 0.39 + 0.1;

      xyzMin[1] = side.negateIfRightSide(0.34);
      xyzMax[1] = side.negateIfRightSide(0.34);

      xyzMin[2] = 0.74 - 0.1;
      xyzMax[2] = 0.74 + 0.1;

      yawPitchRollMin[0] = 0.125 - DELTA_YAW;
      yawPitchRollMax[0] = 0.125 + DELTA_YAW;

      yawPitchRollMin[1] = 0.543 - DELTA_PITCH;
      yawPitchRollMax[1] = 0.543 + DELTA_PITCH;

      yawPitchRollMin[2] = side.negateIfRightSide(0.295);
      yawPitchRollMax[2] = side.negateIfRightSide(0.295 + DELTA_ROLL);
   }
}
