package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollectionMessenger;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndWholeBodyTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBody foot = fullRobotModel.getFoot(footSide);
      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);
      drcSimulationTestHelper.send(HumanoidMessageTools.createAutomaticManipulationAbortMessage(false));

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose3D desiredFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      if (footSide == RobotSide.LEFT)
         wholeBodyTrajectoryMessage.getLeftFootTrajectoryMessage().set(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));
      else
         wholeBodyTrajectoryMessage.getRightFootTrajectoryMessage().set(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose3D> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] arm = ScrewTools.createOneDoFJointPath(chest, hand);
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJoint joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         if (robotSide == RobotSide.LEFT)
            wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage().set(HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame));
         else
            wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage().set(HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame));
      }


      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RigidBody pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
      desiredPelvisPose.setZ(desiredPelvisPose.getZ() - 0.1);
      desiredPosition = new Point3D();
      desiredOrientation = new Quaternion();
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.get(desiredPosition, desiredOrientation);
      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation));

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(worldFrame, RandomGeometry.nextQuaternion(random, 0.5));
      desiredChestOrientation.changeFrame(worldFrame);
      desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame);
      chestTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(chestTrajectoryMessage);

      drcSimulationTestHelper.send(wholeBodyTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RigidBody chest = fullRobotModel.getChest();
      String footName = fullRobotModel.getFoot(footSide).getName();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, scs, chest);
//      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, scs);
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(footName, desiredFootPose.getPosition(), desiredFootPose.getOrientation(), scs);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = drcSimulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3DReadOnly desiredHandPosition = desiredHandPoses.get(robotSide).getPosition();
         Quaternion desiredHandOrientation = new Quaternion(desiredHandPoses.get(robotSide).getOrientation());

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, scs);
      }
   }

   public void testSingleWaypointUsingMessageOfMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      MessageCollectionMessenger messageCollectionMessenger = new MessageCollectionMessenger();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBody foot = fullRobotModel.getFoot(footSide);
      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);
      //drcSimulationTestHelper.send(new AutomaticManipulationAbortMessage(false));

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose3D desiredFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      messageCollectionMessenger.addMessage(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose3D> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] arm = ScrewTools.createOneDoFJointPath(chest, hand);
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJoint joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         messageCollectionMessenger.addMessage(HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame));
      }


      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RigidBody pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
      desiredPelvisPose.setZ(desiredPelvisPose.getZ() - 0.1);
      desiredPosition = new Point3D();
      desiredOrientation = new Quaternion();
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.get(desiredPosition, desiredOrientation);
      messageCollectionMessenger.addMessage(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation));

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(worldFrame, RandomGeometry.nextQuaternion(random, 0.5));
      desiredChestOrientation.changeFrame(worldFrame);
      desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame);
      chestTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      messageCollectionMessenger.addMessage(chestTrajectoryMessage);

      messageCollectionMessenger.sendMessageCollectionSafe(drcSimulationTestHelper.getControllerCommunicator(), false);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RigidBody chest = fullRobotModel.getChest();
      String footName = fullRobotModel.getFoot(footSide).getName();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, scs, chest);
//      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, scs);
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(footName, desiredFootPose.getPosition(), desiredFootPose.getOrientation(), scs);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = drcSimulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3DReadOnly desiredHandPosition = desiredHandPoses.get(robotSide).getPosition();
         Quaternion desiredHandOrientation = new Quaternion(desiredHandPoses.get(robotSide).getOrientation());

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, scs);
      }
   }

   public void testSingleWaypointUsingMessageOfMessagesWithDelays() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      MessageCollectionMessenger messageCollectionMessenger = new MessageCollectionMessenger();

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBody foot = fullRobotModel.getFoot(footSide);
      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.setPosition(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      drcSimulationTestHelper.send(footTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose3D desiredFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredFootPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredFootPose.setPosition(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      messageCollectionMessenger.addMessage(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose3D> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = fullRobotModel.getChest();
         RigidBody hand = fullRobotModel.getHand(robotSide);
         OneDoFJoint[] arm = ScrewTools.createOneDoFJointPath(chest, hand);
         OneDoFJoint[] armClone = ScrewTools.cloneOneDoFJointPath(chest, hand);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJoint joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBody handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, worldFrame);
         handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionDelayTime(5.0);
         messageCollectionMessenger.addMessage(handTrajectoryMessage);
      }


      HumanoidReferenceFrames humanoidReferenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      RigidBody pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredPelvisPose.setOrientation(RandomGeometry.nextQuaternion(random, 1.0));
      desiredPelvisPose.setPosition(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
      desiredPelvisPose.setZ(desiredPelvisPose.getZ() - 0.1);
      desiredPosition = new Point3D();
      desiredOrientation = new Quaternion();
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.get(desiredPosition, desiredOrientation);
      messageCollectionMessenger.addMessage(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation));

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(worldFrame, RandomGeometry.nextQuaternion(random, 0.5));
      desiredChestOrientation.changeFrame(worldFrame);
      desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame);
      chestTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      messageCollectionMessenger.addMessage(chestTrajectoryMessage);

      messageCollectionMessenger.sendMessageCollectionSafe(drcSimulationTestHelper.getControllerCommunicator(), false);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      RigidBody chest = fullRobotModel.getChest();
      String footName = fullRobotModel.getFoot(footSide).getName();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, scs, chest);
//      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, scs);
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(footName, desiredFootPose.getPosition(), desiredFootPose.getOrientation(), scs);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0 + trajectoryTime);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = drcSimulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3DReadOnly desiredHandPosition = desiredHandPoses.get(robotSide).getPosition();
         Quaternion desiredHandOrientation = new Quaternion(desiredHandPoses.get(robotSide).getOrientation());

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, scs);
      }
   }

   public void testIssue47BadChestTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
      SO3TrajectoryMessage so3Trajectory = chestTrajectoryMessage.getSo3Trajectory();
      so3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(pelvisZUpFrame));
      so3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      so3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(0.00, new Quaternion(), new Vector3D()));
      so3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(0.10, new Quaternion(), new Vector3D()));
      so3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(0.20, new Quaternion(), new Vector3D()));
      so3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(0.10, new Quaternion(), new Vector3D()));
      so3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSO3TrajectoryPointMessage(0.00, new Quaternion(), new Vector3D()));




      wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(chestTrajectoryMessage);
      drcSimulationTestHelper.send(wholeBodyTrajectoryMessage);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
   }

   public void testIssue47BadPelvisTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.00, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.10, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.20, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.10, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.00, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(pelvisTrajectoryMessage);
      drcSimulationTestHelper.send(wholeBodyTrajectoryMessage);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
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
