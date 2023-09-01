package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import ihmc_common_msgs.msg.dds.MessageCollectionNotification;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollectionMessenger;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndWholeBodyTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSingleWaypoint() throws Exception
   {
      simulationTestingParameters.setKeepSCSUp(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBodyBasics foot = fullRobotModel.getFoot(footSide);
      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.getPosition().set(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      simulationTestHelper.publishToController(footTrajectoryMessage);
      simulationTestHelper.publishToController(HumanoidMessageTools.createAutomaticManipulationAbortMessage(false));

      success = simulationTestHelper.simulateNow(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose3D desiredFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredFootPose.getOrientation().set(EuclidCoreRandomTools.nextYawPitchRoll(random, 0.17, 0.4, 0.4));
      desiredFootPose.getPosition().set(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.2, 0.3));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      if (footSide == RobotSide.LEFT)
         wholeBodyTrajectoryMessage.getLeftFootTrajectoryMessage()
                                   .set(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));
      else
         wholeBodyTrajectoryMessage.getRightFootTrajectoryMessage()
                                   .set(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose3D> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] arm = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         OneDoFJointBasics[] armClone = MultiBodySystemTools.filterJoints(MultiBodySystemFactories.cloneKinematicChain(arm, robotSide.getLowerCaseName() + "ArmCopy"), OneDoFJointBasics.class);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJointBasics joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBodyBasics handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         if (robotSide == RobotSide.LEFT)
            wholeBodyTrajectoryMessage.getLeftHandTrajectoryMessage()
                                      .set(HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                            trajectoryTime,
                                                                                            desiredPosition,
                                                                                            desiredOrientation,
                                                                                            worldFrame));
         else
            wholeBodyTrajectoryMessage.getRightHandTrajectoryMessage()
                                      .set(HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                            trajectoryTime,
                                                                                            desiredPosition,
                                                                                            desiredOrientation,
                                                                                            worldFrame));
      }

      CommonHumanoidReferenceFrames humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();
      humanoidReferenceFrames.updateFrames();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredPelvisPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 1.0));
      desiredPelvisPose.getPosition().set(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
      desiredPelvisPose.setZ(desiredPelvisPose.getZ() - 0.1);
      desiredPosition = new Point3D();
      desiredOrientation = new Quaternion();
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.get(desiredPosition, desiredOrientation);
      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage()
                                .set(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation));

      FrameQuaternion desiredChestOrientation = new FrameQuaternion(worldFrame, RandomGeometry.nextQuaternion(random, 0.5));
      desiredChestOrientation.changeFrame(worldFrame);
      desiredOrientation = new Quaternion(desiredChestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredOrientation, pelvisZUpFrame);
      chestTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      wholeBodyTrajectoryMessage.getChestTrajectoryMessage().set(chestTrajectoryMessage);

      simulationTestHelper.publishToController(wholeBodyTrajectoryMessage);

      success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      RigidBodyBasics chest = fullRobotModel.getChest();
      String footName = fullRobotModel.getFoot(footSide).getName();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, simulationTestHelper, chest, "Orientation");
      //      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, simulationTestHelper);
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(footName, desiredFootPose.getPosition(), desiredFootPose.getOrientation(), simulationTestHelper);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = simulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3DReadOnly desiredHandPosition = desiredHandPoses.get(robotSide).getPosition();
         Quaternion desiredHandOrientation = new Quaternion(desiredHandPoses.get(robotSide).getOrientation());

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, simulationTestHelper);
      }
   }

   @Test
   public void testSingleWaypointUsingMessageOfMessages() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      MessageCollectionMessenger messageCollectionMessenger = new MessageCollectionMessenger();

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBodyBasics foot = fullRobotModel.getFoot(footSide);
      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.getPosition().set(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      simulationTestHelper.publishToController(footTrajectoryMessage);
      //drcSimulationTestHelper.send(new AutomaticManipulationAbortMessage(false));

      success = simulationTestHelper.simulateNow(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose3D desiredFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredFootPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 0.5));
      desiredFootPose.getPosition().set(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.05, 0.15));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      messageCollectionMessenger.addMessage(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose3D> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] arm = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         OneDoFJointBasics[] armClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(chest, hand);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJointBasics joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBodyBasics handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         messageCollectionMessenger.addMessage(HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                trajectoryTime,
                                                                                                desiredPosition,
                                                                                                desiredOrientation,
                                                                                                worldFrame));
      }

      CommonHumanoidReferenceFrames humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();
      humanoidReferenceFrames.updateFrames();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredPelvisPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 0.7));
      desiredPelvisPose.getPosition().set(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
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

      simulationTestHelper.createSubscriberFromController(MessageCollectionNotification.class, messageCollectionMessenger::receivedNotification);
      messageCollectionMessenger.sendMessageCollectionSafe(simulationTestHelper::publishToController, false);

      success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      RigidBodyBasics chest = fullRobotModel.getChest();
      String footName = fullRobotModel.getFoot(footSide).getName();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, simulationTestHelper, chest, "Orientation");
      //      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, simulationTestHelper);
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(footName, desiredFootPose.getPosition(), desiredFootPose.getOrientation(), simulationTestHelper);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = simulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3DReadOnly desiredHandPosition = desiredHandPoses.get(robotSide).getPosition();
         Quaternion desiredHandOrientation = new Quaternion(desiredHandPoses.get(robotSide).getOrientation());

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, simulationTestHelper);
      }
   }

   @Test
   public void testSingleWaypointUsingMessageOfMessagesWithDelays() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      MessageCollectionMessenger messageCollectionMessenger = new MessageCollectionMessenger();

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      RobotSide footSide = RobotSide.LEFT;
      // First need to pick up the foot:
      RigidBodyBasics foot = fullRobotModel.getFoot(footSide);
      FramePose3D footPoseCloseToActual = new FramePose3D(foot.getBodyFixedFrame());
      footPoseCloseToActual.getPosition().set(0.0, 0.0, 0.10);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      footPoseCloseToActual.changeFrame(worldFrame);
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      footPoseCloseToActual.get(desiredPosition, desiredOrientation);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.0, desiredPosition, desiredOrientation);
      simulationTestHelper.publishToController(footTrajectoryMessage);

      success = simulationTestHelper.simulateNow(1.0 + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime());
      assertTrue(success);

      // Now we can do the usual test.
      double trajectoryTime = 1.0;
      FramePose3D desiredFootPose = new FramePose3D(foot.getBodyFixedFrame());
      desiredFootPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 0.5));
      desiredFootPose.getPosition().set(RandomGeometry.nextPoint3D(random, -0.1, -0.1, 0.05, 0.1, 0.05, 0.15));
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      messageCollectionMessenger.addMessage(HumanoidMessageTools.createFootTrajectoryMessage(footSide, trajectoryTime, desiredPosition, desiredOrientation));

      SideDependentList<FramePose3D> desiredHandPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] arm = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         OneDoFJointBasics[] armClone = MultiBodySystemFactories.cloneOneDoFJointKinematicChain(chest, hand);
         for (int i = 0; i < armClone.length; i++)
         {
            OneDoFJointBasics joint = armClone[i];
            joint.setQ(arm[i].getQ() + RandomNumbers.nextDouble(random, -0.2, 0.2));
         }
         RigidBodyBasics handClone = armClone[armClone.length - 1].getSuccessor();
         FramePose3D desiredRandomHandPose = new FramePose3D(handClone.getBodyFixedFrame());
         desiredRandomHandPose.changeFrame(worldFrame);
         desiredHandPoses.put(robotSide, desiredRandomHandPose);
         desiredPosition = new Point3D();
         desiredOrientation = new Quaternion();
         desiredRandomHandPose.get(desiredPosition, desiredOrientation);
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                        trajectoryTime,
                                                                                                        desiredPosition,
                                                                                                        desiredOrientation,
                                                                                                        worldFrame);
         handTrajectoryMessage.getSe3Trajectory().getQueueingProperties().setExecutionDelayTime(5.0);
         messageCollectionMessenger.addMessage(handTrajectoryMessage);
      }

      CommonHumanoidReferenceFrames humanoidReferenceFrames = simulationTestHelper.getControllerReferenceFrames();
      humanoidReferenceFrames.updateFrames();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      ReferenceFrame pelvisZUpFrame = humanoidReferenceFrames.getPelvisZUpFrame();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      desiredPelvisPose.getOrientation().set(RandomGeometry.nextQuaternion(random, 0.7));
      desiredPelvisPose.getPosition().set(RandomGeometry.nextPoint3D(random, 0.05, 0.03, 0.05));
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

      simulationTestHelper.createSubscriberFromController(MessageCollectionNotification.class, messageCollectionMessenger::receivedNotification);
      messageCollectionMessenger.sendMessageCollectionSafe(simulationTestHelper::publishToController, false);

      success = simulationTestHelper.simulateNow(getRobotModel().getControllerDT()); // Trick to get frames synchronized with the controller.
      assertTrue(success);

      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(pelvisZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         desiredHandPoses.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      RigidBodyBasics chest = fullRobotModel.getChest();
      String footName = fullRobotModel.getFoot(footSide).getName();
      humanoidReferenceFrames.updateFrames();
      desiredChestOrientation.changeFrame(worldFrame);
      EndToEndChestTrajectoryMessageTest.assertSingleWaypointExecuted(desiredChestOrientation, simulationTestHelper, chest, "Orientation");
      //      EndToEndPelvisTrajectoryMessageTest.assertSingleWaypointExecuted(desiredPosition, desiredOrientation, simulationTestHelper);
      EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(footName, desiredFootPose.getPosition(), desiredFootPose.getOrientation(), simulationTestHelper);

      success = simulationTestHelper.simulateNow(5.0 + trajectoryTime);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = simulationTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
         desiredHandPoses.get(robotSide).changeFrame(worldFrame);
         Point3DReadOnly desiredHandPosition = desiredHandPoses.get(robotSide).getPosition();
         Quaternion desiredHandOrientation = new Quaternion(desiredHandPoses.get(robotSide).getOrientation());

         EndToEndHandTrajectoryMessageTest.assertSingleWaypointExecuted(handName, desiredHandPosition, desiredHandOrientation, simulationTestHelper);
      }
   }

   @Test
   public void testIssue47BadChestTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();
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
      simulationTestHelper.publishToController(wholeBodyTrajectoryMessage);

      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);
   }

   public void testIssue47BadPelvisTrajectoryMessage() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);

      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                             .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.00, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                             .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.10, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                             .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.20, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                             .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.10, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                             .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(0.00, new Point3D(), new Quaternion(), new Vector3D(), new Vector3D()));
      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().set(pelvisTrajectoryMessage);
      simulationTestHelper.publishToController(wholeBodyTrajectoryMessage);

      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
