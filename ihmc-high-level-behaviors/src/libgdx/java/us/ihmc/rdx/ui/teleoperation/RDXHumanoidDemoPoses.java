package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import imgui.ImGui;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

//TODO move parameters of poses in Nadia
public class RDXHumanoidDemoPoses extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RDXTeleoperationParameters teleoperationParameters;

   private SideDependentList<double[]> armsConfiguration = new SideDependentList<>();
   private final YawPitchRoll chestOrientation;
   private final YawPitchRoll pelvisOrientation;
   private final Point3D pelvisPosition;
   private boolean usedFirstMode = false;

   private final SideDependentList<FootLoadBearingMessage> footLoadBearingMessagesToPublish = new SideDependentList<>();
   private final AtomicReference<ChestTrajectoryMessage> chestTrajectoryMessagesToPublish = new AtomicReference<>();
   private final ArrayList<ArmTrajectoryMessage> armTrajectoryMessagesToPublish = new ArrayList<>();
   private final ArrayList<PelvisTrajectoryMessage> pelvisTrajectoryMessagesToPublish = new ArrayList<>();
   private final SideDependentList<FootTrajectoryMessage> footTrajectoryMessagesToPublish = new SideDependentList<>();

   private static final double[] rightArmGreeting1 = {0.27, -1.00, -0.682, -2.24, -0.592, -0.61, -0.79};
   private static final double[] rightArmGreeting2 = {0.253, -0.872, -1.22, -2.035, 0.073, -0.61, -0.839};
   private static final double[] leftArmSquat = new double[] {-0.7, 0.8, 0.53, -1.9};
   private static final double[] rightArmSquat = new double[] {-0.7, -0.8, -0.53, -1.9};

   private static final double[] leftArmFlex1 = new double[] {0.71, 1.4, 1.12, -2.32};
   private static final double[] rightArmFlex1 = new double[] {0.71, -1.4, -1.12, -2.32};
   private static final double[] leftArmFlex2 = new double[] {0.02, 1.1, -0.96, -2.32};
   private static final double[] rightArmFlex2 = new double[] {0.02, -1.1, 0.96, -2.32};

   private static final double[] leftArmBallet1 = new double[] {-0.39, 2.21, 0.48, -2.32};
   private static final double[] rightArmBallet1 = new double[] {0.02, -0.7, 0.44, -1.7};
   private static final double[] leftArmBallet2 = new double[] {0.6, 0.8, -0.91, -1.84};
   private static final double[] rightArmBallet2 = new double[] {-0.56, -0.31, -0.53, -2.24};

   public RDXHumanoidDemoPoses(DRCRobotModel robotModel,
                               ROS2SyncedRobotModel syncedRobot,
                               ROS2ControllerHelper ros2ControllerHelper,
                               RDXTeleoperationParameters teleoperationParameters)
   {
      super("Demo Poses");

      setRenderMethod(this::renderImGuiWidgets);
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.teleoperationParameters = teleoperationParameters;

      armsConfiguration.put(RobotSide.LEFT, robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
      armsConfiguration.put(RobotSide.RIGHT, robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.HOME));
      chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
      pelvisOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
      pelvisPosition = new Point3D(0.0, 0.0, 1.08);
   }

   private static double[] quickCopy(double[] incoming)
   {
      double[] outgoing = new double[incoming.length];
      System.arraycopy(incoming, 0, outgoing, 0, incoming.length);
      return outgoing;
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Demo Poses: ");
      ImGui.sameLine();
      // zero our poses
      pelvisOrientation.setToZero();
      pelvisPosition.setToZero();
      pelvisPosition.setZ(1.08);
      chestOrientation.setToZero();

      double[] leftArm = quickCopy(robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
      double[] rightArm = quickCopy(robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.HOME));
      armsConfiguration.put(RobotSide.LEFT, leftArm);
      armsConfiguration.put(RobotSide.RIGHT, rightArm);

      chestTrajectoryMessagesToPublish.set(null);
      armTrajectoryMessagesToPublish.clear();
      pelvisTrajectoryMessagesToPublish.clear();
      footTrajectoryMessagesToPublish.clear();

      if (ImGui.button(labels.get("Home Pose")))
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime());
            if (isFootOffGround(robotSide))
               setFootDown(robotSide, 1.0);
         }
         appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime());
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime());

         publishPoses();
      }
      if (ImGui.button(labels.get("Greeting")))
      {
         if (usedFirstMode)
         {
            System.arraycopy(rightArmGreeting1, 0, rightArm, 0, Math.min(rightArm.length, rightArmGreeting1.length));
         }
         else
         {
            System.arraycopy(rightArmGreeting2, 0, rightArm, 0, Math.min(rightArm.length, rightArmGreeting2.length));
         }

         armsConfiguration.put(RobotSide.RIGHT, rightArm);
         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), 1.0);
         appendChestOrientationToPublish(chestOrientation, 1.0);
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, 1.0);

         publishPoses();

         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Squat")))
      {
         if (!usedFirstMode)
         {
            System.arraycopy(leftArmSquat, 0, leftArm, 0, Math.min(leftArm.length, leftArmSquat.length));
            System.arraycopy(rightArmSquat, 0, rightArm, 0, Math.min(rightArm.length, rightArmSquat.length));
            armsConfiguration.put(RobotSide.LEFT, leftArm);
            armsConfiguration.put(RobotSide.RIGHT, rightArm);

            pelvisOrientation.setToPitchOrientation(Math.toRadians(10.0));
            pelvisPosition.setZ(0.78);
         }
         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime());
         appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime());
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime());

         publishPoses();
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Flex")))
      {
         if (usedFirstMode)
         {
            System.arraycopy(leftArmFlex1, 0, leftArm, 0, Math.min(leftArm.length, leftArmFlex1.length));
            System.arraycopy(rightArmFlex1, 0, rightArm, 0, Math.min(rightArm.length, rightArmFlex1.length));
         }
         else
         {
            System.arraycopy(leftArmFlex2, 0, leftArm, 0, Math.min(leftArm.length, leftArmFlex2.length));
            System.arraycopy(rightArmFlex2, 0, rightArm, 0, Math.min(rightArm.length, rightArmFlex2.length));

            pelvisOrientation.setToPitchOrientation(Math.toRadians(35));
            pelvisPosition.setZ(1.0);
         }
         armsConfiguration.put(RobotSide.LEFT, leftArm);
         armsConfiguration.put(RobotSide.RIGHT, rightArm);

         for (RobotSide robotSide : RobotSide.values)
            appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime());
         appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime());
         appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime());
         publishPoses();
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Ballet")))
      {
         if (usedFirstMode)
         {
            System.arraycopy(leftArmBallet1, 0, leftArm, 0, Math.min(leftArm.length, leftArmBallet1.length));
            System.arraycopy(rightArmBallet1, 0, rightArm, 0, Math.min(rightArm.length, rightArmBallet1.length));

            chestOrientation.setToYawOrientation(Math.toRadians(30.0));
            pelvisOrientation.setYaw(Math.toRadians(15));
            pelvisOrientation.setPitch(Math.toRadians(10));

            FramePose3D rightLiftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleZUpFrame(RobotSide.LEFT));
            rightLiftFootPose.getPosition().addZ(0.025);
            appendFootPoseToPublish(RobotSide.RIGHT, 0.15, rightLiftFootPose);

            FramePose3D rightFinalFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleZUpFrame(RobotSide.LEFT));
            rightFinalFootPose.setZ(0.2);
            rightFinalFootPose.setY(-0.5);
            rightFinalFootPose.appendYawRotation(Math.toRadians(30.0));
            rightFinalFootPose.appendPitchRotation(Math.toRadians(45.0));
            rightFinalFootPose.appendRollRotation(Math.toRadians(-25.0));

            appendFootPoseToPublish(RobotSide.RIGHT, 0.75, rightFinalFootPose);

            for (RobotSide robotSide : RobotSide.values)
               appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime(), 0.25);
            appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime(), 0.25);
            appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime(), 0.25);
         }
         else
         {
            System.arraycopy(leftArmBallet2, 0, leftArm, 0, Math.min(leftArm.length, leftArmBallet2.length));
            System.arraycopy(rightArmBallet2, 0, rightArm, 0, Math.min(rightArm.length, rightArmBallet2.length));

            chestOrientation.setToZero();
            pelvisOrientation.setToPitchOrientation(Math.toRadians(30));
            pelvisPosition.setZ(0.90);

            if (isFootOffGround(RobotSide.RIGHT))
               setFootDown(RobotSide.RIGHT, 1.0);

            for (RobotSide robotSide : RobotSide.values)
               appendArmTrajectoryMessageToPublish(robotSide, armsConfiguration.get(robotSide), teleoperationParameters.getTrajectoryTime(), 0.25);
            appendChestOrientationToPublish(chestOrientation, teleoperationParameters.getTrajectoryTime(), 0.25);
            appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, teleoperationParameters.getTrajectoryTime(), 0.25);
         }

         armsConfiguration.put(RobotSide.LEFT, leftArm);
         armsConfiguration.put(RobotSide.RIGHT, rightArm);



         publishPoses();
         usedFirstMode = !usedFirstMode;
      }
   }

   private boolean isFootOffGround(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return syncedRobot.getLatestCapturabilityBasedStatus().getLeftFootSupportPolygon3d().isEmpty();
      else
         return syncedRobot.getLatestCapturabilityBasedStatus().getRightFootSupportPolygon3d().isEmpty();
   }

   private void setFootDown(RobotSide robotSide, double totalDuration)
   {
      FramePose3D finalFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide.getOppositeSide()));
      finalFootPose.setY(robotSide.negateIfRightSide(0.225)); // TODO extract this parameter

      FramePose3D intermediatePose = new FramePose3D(finalFootPose);
      intermediatePose.getPosition().addZ(0.075);
      finalFootPose.getPosition().subZ(0.02);

      intermediatePose.changeFrame(ReferenceFrame.getWorldFrame());
      finalFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      appendFootPoseToPublish(robotSide, 0.75 * totalDuration, intermediatePose);
      appendFootPoseToPublish(robotSide, 0.25 * totalDuration, finalFootPose);

      FootLoadBearingMessage loadBearingMessage = HumanoidMessageTools.createFootLoadBearingMessage(robotSide, LoadBearingRequest.LOAD);
      loadBearingMessage.setExecutionDelayTime(totalDuration);

      footLoadBearingMessagesToPublish.put(robotSide, loadBearingMessage);
   }

   private void appendFootPoseToPublish(RobotSide robotSide, double moveDuration, FramePose3DReadOnly footPose)
   {
      if (footTrajectoryMessagesToPublish.get(robotSide) == null)
      {
         FootTrajectoryMessage footTrajectoryMessage = new FootTrajectoryMessage();
         footTrajectoryMessage.setRobotSide(RobotSide.RIGHT.toByte());

         footTrajectoryMessagesToPublish.put(robotSide, footTrajectoryMessage);
      }

      FootTrajectoryMessage footTrajectoryMessage = footTrajectoryMessagesToPublish.get(robotSide);

      FramePose3D poseToPublish = new FramePose3D(footPose);
      poseToPublish.changeFrame(ReferenceFrame.getWorldFrame());

      double waypointTime;
      if (!footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
         waypointTime = footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().get(footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().size() - 1).getTime();
      else
         waypointTime = 0.0;
      waypointTime += moveDuration;

      SE3TrajectoryPointMessage trajectoryPoint = footTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPoint.getPosition().set(footPose.getPosition());
      trajectoryPoint.getOrientation().set(footPose.getOrientation());
      trajectoryPoint.setTime(waypointTime);
   }

   private void appendPelvisOrientationToPublish(Orientation3DReadOnly pelvisOrientation, Point3DReadOnly pelvisPosition, double trajectoryDuration)
   {
      appendPelvisOrientationToPublish(pelvisOrientation, pelvisPosition, trajectoryDuration, 0.0);
   }

   private void appendPelvisOrientationToPublish(Orientation3DReadOnly pelvisOrientation,
                                                 Point3DReadOnly pelvisPosition,
                                                 double trajectoryDuration,
                                                 double executionDelay)
   {
      FramePose3D pelvisPose = new FramePose3D(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      pelvisPose.changeFrame(syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame());
      pelvisPose.getTranslation().setZ(pelvisPosition.getZ());
      pelvisPose.getRotation().set(pelvisOrientation);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

      PelvisTrajectoryMessage pelvisMessage = new PelvisTrajectoryMessage();
      pelvisMessage.getSe3Trajectory()
                   .set(HumanoidMessageTools.createSE3TrajectoryMessage(trajectoryDuration,
                                                                        pelvisPose.getPosition(),
                                                                        pelvisPose.getOrientation(),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      pelvisMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      pelvisMessage.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(false);
      pelvisMessage.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(false);
      pelvisMessage.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(true);

      pelvisMessage.getSe3Trajectory().getQueueingProperties().setExecutionDelayTime(executionDelay);

      pelvisTrajectoryMessagesToPublish.add(pelvisMessage);
   }

   private void appendChestOrientationToPublish(Orientation3DReadOnly chestOrientation, double trajectoryDuration)
   {
      appendChestOrientationToPublish(chestOrientation, trajectoryDuration, 0.0);
   }

   private void appendChestOrientationToPublish(Orientation3DReadOnly chestOrientation, double trajectoryDuration, double executionDelay)
   {
      ChestTrajectoryMessage chestTrajectoryMessage = chestTrajectoryMessagesToPublish.get();
      if (chestTrajectoryMessage == null)
      {
         chestTrajectoryMessage = new ChestTrajectoryMessage();
         chestTrajectoryMessagesToPublish.set(chestTrajectoryMessage);
      }
      FrameQuaternion desiredChestOrientation = new FrameQuaternion(syncedRobot.getReferenceFrames().getPelvisFrame(), chestOrientation);
      // FIXME append to the queue
      chestTrajectoryMessage.getSo3Trajectory()
                            .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryDuration,
                                                                                 desiredChestOrientation,
                                                                                 EuclidCoreTools.zeroVector3D,
                                                                                 desiredChestOrientation.getReferenceFrame()));
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setXSelected(true);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setYSelected(true);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setZSelected(true);

      chestTrajectoryMessage.getSo3Trajectory().getQueueingProperties().setExecutionDelayTime(executionDelay);
   }

   private void appendArmTrajectoryMessageToPublish(RobotSide side, double[] armConfiguration, double trajectoryDuration)
   {
      appendArmTrajectoryMessageToPublish(side, armConfiguration, trajectoryDuration, 0.0);
   }

   private void appendArmTrajectoryMessageToPublish(RobotSide side, double[] armConfiguration, double trajectoryDuration, double executionDelay)
   {
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                  trajectoryDuration,
                                                                                                  armConfiguration);
      armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionDelayTime(executionDelay);

      armTrajectoryMessagesToPublish.add(armTrajectoryMessage);
   }

   public void publishPoses()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footTrajectoryMessagesToPublish.get(robotSide) != null)
         {
            ros2ControllerHelper.publishToController(footTrajectoryMessagesToPublish.get(robotSide));
            footTrajectoryMessagesToPublish.put(robotSide, null);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootLoadBearingMessage footLoadBearingMessage = footLoadBearingMessagesToPublish.get(robotSide);
         if (footLoadBearingMessage != null)
            ros2ControllerHelper.publishToController(footLoadBearingMessage);
         footLoadBearingMessagesToPublish.put(robotSide, null);
      }

      while (!armTrajectoryMessagesToPublish.isEmpty())
         ros2ControllerHelper.publishToController(armTrajectoryMessagesToPublish.remove(0));

      if (chestTrajectoryMessagesToPublish.get() != null)
         ros2ControllerHelper.publishToController(chestTrajectoryMessagesToPublish.getAndSet(null));

      while (!pelvisTrajectoryMessagesToPublish.isEmpty())
         ros2ControllerHelper.publishToController(pelvisTrajectoryMessagesToPublish.remove(0));
   }
}
