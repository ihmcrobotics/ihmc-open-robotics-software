package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import java.io.IOException;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public abstract class WholeBodyInverseKinematicsBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;

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
      
      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.close();
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface envrionment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 160000)
   public void testSolvingForAHandPose() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      String nameSpace = robotSide.getCamelCaseNameForStartOfExpression() + HandControlModule.class.getSimpleName();
      String varname = nameSpace + "SwitchTime";
      double initialSwitchTime = scs.getVariable(nameSpace, varname).getValueAsDouble();

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                     drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                     getRobotModel().createFullRobotModel());

      ReferenceFrame handControlFrame = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      ReferenceFrame chestControlFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      FrameOrientation initialChestOrientation = new FrameOrientation(chestControlFrame);
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      ReferenceFrame pelvisControlFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getPelvis().getBodyFixedFrame();
      FrameOrientation initialPelvisOrientation = new FrameOrientation(pelvisControlFrame);
      initialPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose desiredHandPose = new FramePose(handControlFrame);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPose.translate(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(robotSide, desiredHandPose);
      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         ThreadTools.sleep(100);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double newSwitchTime = scs.getVariable(nameSpace, varname).getValueAsDouble();

      assertNotEquals(initialSwitchTime, newSwitchTime, 1.0e-3);

      FramePose currentHandPose = new FramePose(handControlFrame);
      currentHandPose.changeFrame(ReferenceFrame.getWorldFrame());

      FrameOrientation finalChestOrientation = new FrameOrientation(chestControlFrame);
      finalChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FrameOrientation finalPelvisOrientation = new FrameOrientation(pelvisControlFrame);
      finalPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      double angleEpsilon = Math.toRadians(10);

      assertTrue(isOrientationEqual(initialChestOrientation.getQuaternion(), finalChestOrientation.getQuaternion(), angleEpsilon));
      assertTrue(isOrientationEqual(initialPelvisOrientation.getQuaternion(), finalPelvisOrientation.getQuaternion(), angleEpsilon));

      double handPosition = desiredHandPose.getPositionDistance(currentHandPose);
      double positionEpsilon = 1.0e-2;
      assertTrue(Math.abs(handPosition) < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 160000)
   public void testSolvingForBothHandPoses() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                     drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                     getRobotModel().createFullRobotModel());

      ReferenceFrame handControlFrameR = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

      FramePose desiredHandPoseR = new FramePose(handControlFrameR);
      desiredHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPoseR.translate(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPoseR);
      FramePose desiredHandPoseL = new FramePose(handControlFrameL);
      desiredHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPoseL.translate(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(RobotSide.LEFT, desiredHandPoseL);
      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         ThreadTools.sleep(100);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FramePose currentHandPoseR = new FramePose(handControlFrameR);
      currentHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose currentHandPoseL = new FramePose(handControlFrameL);
      currentHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());

      AxisAngle4d leftAngle = new AxisAngle4d();
      AxisAngle4d rightAngle = new AxisAngle4d();

      desiredHandPoseL.getAxisAngleRotationToOtherPose(currentHandPoseL, leftAngle);
      desiredHandPoseR.getAxisAngleRotationToOtherPose(currentHandPoseR, rightAngle);

      double angleEpsilon = Math.toRadians(2);

      assertTrue(Math.abs(leftAngle.angle) < angleEpsilon);
      assertTrue(Math.abs(rightAngle.angle) < angleEpsilon);

      double leftPosition = desiredHandPoseL.getPositionDistance(currentHandPoseL);
      double rightPosition = desiredHandPoseR.getPositionDistance(currentHandPoseR);

      double positionEpsilon = 1.0e-2;
      System.out.println(leftPosition);
      assertTrue(Math.abs(leftPosition) < positionEpsilon);
      assertTrue(Math.abs(rightPosition) < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private boolean isOrientationEqual(Quat4d initialQuat, Quat4d finalQuat, double angleEpsilon)
   {
      Quat4d quatDifference = new Quat4d(initialQuat);
      quatDifference.mulInverse(finalQuat);

      AxisAngle4d angleDifference = new AxisAngle4d();
      angleDifference.set(quatDifference);
      AngleTools.trimAngleMinusPiToPi(angleDifference.getAngle());

      return Math.abs(angleDifference.getAngle()) < angleEpsilon;
   }

   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
   }
}
