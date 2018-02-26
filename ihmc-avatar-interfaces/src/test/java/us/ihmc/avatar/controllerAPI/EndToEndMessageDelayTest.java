package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.ClearDelayQueueMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public abstract class EndToEndMessageDelayTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setKeepSCSUp(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 19.1)
   @Test(timeout = 95000)
   public void testClearingDelaysWithMessageOfMessagesWithDelays() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      
      Random random = new Random(564574L);
      
      DRCRobotModel robotModel = getRobotModel();
      DRCRobotJointMap jointMap = robotModel.getJointMap();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      
      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      
      MessageOfMessages messageOfMessages = new MessageOfMessages();
      
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      humanoidReferenceFrames.updateFrames();

      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      RigidBody chest = fullRobotModel.getChest();
      
      double trajectoryTime = 1.0;
      FrameQuaternion lookStraightAhead = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), new Quaternion());
      lookStraightAhead.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookLeftQuat = new Quaternion();
      lookLeftQuat.appendYawRotation(Math.PI / 8.0);
      lookLeftQuat.appendPitchRotation(Math.PI / 16.0);
      lookLeftQuat.appendRollRotation(-Math.PI / 16.0);
      FrameQuaternion lookLeft = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookLeftQuat);
      lookLeft.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion lookRightQuat = new Quaternion();
      lookRightQuat.appendYawRotation(-Math.PI / 8.0);
      lookRightQuat.appendPitchRotation(-Math.PI / 16.0);
      lookRightQuat.appendRollRotation(Math.PI / 16.0);
      FrameQuaternion lookRight = new FrameQuaternion(humanoidReferenceFrames.getPelvisZUpFrame(), lookRightQuat);
      lookRight.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage lookStraightAheadMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, lookStraightAhead, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookStraightAheadMessage.getSO3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookStraightAheadMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(robotModel.getControllerDT()));

      ChestTrajectoryMessage lookLeftMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, lookLeft, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookLeftMessage.getSO3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookLeftMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(robotModel.getControllerDT()));

      ChestTrajectoryMessage lookRightMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, lookRight, ReferenceFrame.getWorldFrame(), pelvisZUpFrame);
      lookRightMessage.getSO3Trajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE, -1);
      drcSimulationTestHelper.send(lookRightMessage);

      SpineJointName[] spineJointNames = jointMap.getSpineJointNames();
      SpineTrajectoryMessage zeroSpineJointMessage = HumanoidMessageTools.createSpineTrajectoryMessage(spineJointNames.length, 1);
      zeroSpineJointMessage.getQueueingProperties().setExecutionDelayTime(5.0);
      for(int i = 0; i < spineJointNames.length; i++)
      {
         zeroSpineJointMessage.getJointspaceTrajectory().setTrajectory1DMessage(i, HumanoidMessageTools.createOneDoFJointTrajectoryMessage(1.0, 0.0));
      }
      drcSimulationTestHelper.send(zeroSpineJointMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(robotModel.getControllerDT() * 3.0));
      assertEquals(RigidBodyControlMode.TASKSPACE, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
      
      ClearDelayQueueMessage clearMessage = new ClearDelayQueueMessage();
      clearMessage.setClassToClear(SpineTrajectoryMessage.class);
      drcSimulationTestHelper.send(clearMessage);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0));
      //send a taskspace command, then delayed a jointspace command and then cleared the delay queue, so we should be in taskspace still
      assertEquals(RigidBodyControlMode.TASKSPACE, EndToEndArmTrajectoryMessageTest.findControllerState(chest.getName(), scs));
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
