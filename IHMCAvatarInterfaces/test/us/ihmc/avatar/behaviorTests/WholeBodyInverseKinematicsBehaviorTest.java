package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.*;

import java.io.IOException;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
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
 
   @ContinuousIntegrationTest(estimatedDuration = 27.0)
   @Test(timeout = 160000)
   public void testSolvingForAHandPose() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      CommonAvatarEnvironmentInterface envrionment = new FlatGroundEnvironment();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());
      setupKinematicsToolboxModule();
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
      
      ReferenceFrame chestControlFrame = getRobotModel().createFullRobotModel().getChest().getBodyFixedFrame();
      FrameOrientation initialChestOrientation = new FrameOrientation(chestControlFrame);   
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      
      ReferenceFrame pelvisControlFrame = drcBehaviorTestHelper.getReferenceFrames().getPelvisFrame();
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

      assertFalse(ik.hasSolverFailed());

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
      
      assertTrue(initialChestOrientation.epsilonEquals(finalChestOrientation, 1.0e-2));
      assertTrue(initialPelvisOrientation.epsilonEquals(finalPelvisOrientation, 1.0e-2));

      assertTrue("Expect: " + desiredHandPose + "\nActual: " + currentHandPose, currentHandPose.epsilonEquals(desiredHandPose, 1.0e-2));

   }

   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      new KinematicsToolboxModule(robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), isKinematicsToolboxVisualizerEnabled);
      drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
   }
}
