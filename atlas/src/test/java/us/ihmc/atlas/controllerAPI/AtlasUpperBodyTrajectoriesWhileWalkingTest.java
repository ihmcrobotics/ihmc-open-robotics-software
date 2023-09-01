package us.ihmc.atlas.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.trajectories.generators.OneDoFTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Tag("controller-api-2")
public class AtlasUpperBodyTrajectoriesWhileWalkingTest
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      //TODO: remove this once state estimator parameters are tuned
      simulationTestingParameters.setUsePefectSensors(true);
      simulationTestingParameters.setKeepSCSUp(false);
   }

   protected SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testWalkingWithRandomArmTrajectoryMovements() throws Exception
   {
      Random random = new Random(564654L);
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, simulationTestingParameters);
      simulationTestHelper.start();
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();

      double timeToCompleteWalking = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames);
      sendArmTrajectoryMessageWithRandomPoints(random, robotModel, fullRobotModel);

      success = simulationTestHelper.simulateNow(timeToCompleteWalking);
      assertTrue(success);
   }

   @Test
   public void testWalkingWithArmsHoldingInFeetFrame() throws Exception
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, simulationTestingParameters);
      simulationTestHelper.start();
      boolean success = simulationTestHelper.simulateNow(2.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      fullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = (HumanoidReferenceFrames) simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();

      double timeToCompleteWalking = sendWalkingPacket(robotModel, fullRobotModel, referenceFrames);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame handFrame = referenceFrames.getHandFrame(robotSide);
         FramePose3D handPosition = new FramePose3D();
         handPosition.setToZero(handFrame);
         handPosition.changeFrame(worldFrame);
         Point3D position = new Point3D(handPosition.getPosition());
         Quaternion orientation = new Quaternion(handPosition.getOrientation());

         HandTrajectoryMessage handHoldMessage = new HandTrajectoryMessage();
         handHoldMessage.setRobotSide(robotSide.toByte());
         handHoldMessage.getSe3Trajectory().getFrameInformation()
                        .setTrajectoryReferenceFrameId(MessageTools.toFrameId(referenceFrames.getAnkleZUpFrame(robotSide.getOppositeSide())));
         handHoldMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         Vector3D zeroVelocity = new Vector3D();
         handHoldMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add()
                        .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(11.0, position, orientation, zeroVelocity, zeroVelocity));
         simulationTestHelper.publishToController(handHoldMessage);
      }
      success = simulationTestHelper.simulateNow(timeToCompleteWalking);
      assertTrue(success);
   }

   private void sendArmTrajectoryMessageWithRandomPoints(Random random, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel)
   {
      boolean success;
      long id = 1264L;
      double timePerWaypoint = 0.5;
      int numberOfMessages = 5;
      int numberOfTrajectoryPoints = 5;

      SideDependentList<OneDoFJointBasics[]> armsJoints = new SideDependentList<>();
      SideDependentList<List<ArmTrajectoryMessage>> armTrajectoryMessages = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics chest = fullRobotModel.getChest();
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         armsJoints.put(robotSide, armJoints);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(armJoints);

         List<ArmTrajectoryMessage> messageList = new ArrayList<>();

         for (int messageIndex = 0; messageIndex < numberOfMessages; messageIndex++)
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide);
            armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setMessageId(id);

            if (messageIndex > 0)
            {
               armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
               armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setPreviousMessageId(id - 1);
            }
            id++;

            OneDoFTrajectoryPointCalculator trajectoryPoint1DCalculator = new OneDoFTrajectoryPointCalculator();

            for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
            {
               OneDoFJointBasics joint = armJoints[jointIndex];
               OneDoFJointTrajectoryMessage jointTrajectoryMessage = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().add();

               trajectoryPoint1DCalculator.clear();

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  double desiredJointPosition = RandomNumbers.nextDouble(random, joint.getJointLimitLower(), joint.getJointLimitUpper());
                  trajectoryPoint1DCalculator.appendTrajectoryPoint(desiredJointPosition);
               }

               trajectoryPoint1DCalculator.compute(timePerWaypoint * (numberOfTrajectoryPoints - 1));
               OneDoFTrajectoryPointList trajectoryData = trajectoryPoint1DCalculator.getTrajectoryData();
               trajectoryData.addTimeOffset(timePerWaypoint);

               for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
               {
                  OneDoFTrajectoryPoint trajectoryPoint = trajectoryData.getTrajectoryPoint(trajectoryPointIndex);
                  jointTrajectoryMessage.getTrajectoryPoints().add()
                                        .set(HumanoidMessageTools.createTrajectoryPoint1DMessage(trajectoryPoint.getTime(),
                                                                                                 trajectoryPoint.getPosition(),
                                                                                                 trajectoryPoint.getVelocity()));
               }
            }
            messageList.add(armTrajectoryMessage);
            simulationTestHelper.publishToController(armTrajectoryMessage);

            success = simulationTestHelper.simulateNow(robotModel.getControllerDT());
            assertTrue(success);
         }
         armTrajectoryMessages.put(robotSide, messageList);
      }
   }

   private double sendWalkingPacket(DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, CommonHumanoidReferenceFrames referenceFrames)
   {
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double stepTime = swingTime + transferTime;

      Vector2D desiredVelocity = new Vector2D(0.15, 0.0);
      int numberOfSteps = 30;
      FootstepDataListMessage footsteps = EndToEndPelvisTrajectoryMessageTest.computeNextFootsteps(numberOfSteps,
                                                                                                   RobotSide.LEFT,
                                                                                                   referenceFrames.getSoleFrames(),
                                                                                                   walkingControllerParameters,
                                                                                                   desiredVelocity);
      footsteps.setDefaultSwingDuration(swingTime);
      footsteps.setDefaultTransferDuration(transferTime);

      simulationTestHelper.publishToController(footsteps);

      int timeWalking = numberOfSteps;
      double timeToCompleteWalking = stepTime * timeWalking;
      return timeToCompleteWalking;
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
