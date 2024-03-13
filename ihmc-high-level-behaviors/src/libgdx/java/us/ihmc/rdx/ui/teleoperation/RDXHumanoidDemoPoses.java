package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import imgui.ImGui;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXHumanoidDemoPoses extends RDXPanel
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RDXTeleoperationParameters teleoperationParameters;

   private SideDependentList<double[]> armsConfiguration = new SideDependentList<>();
   private YawPitchRoll chestOrientation;
   private Point3D pelvisPosition;
   private boolean usedFirstMode = false;

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
      pelvisPosition = new Point3D(0.0, 0.0, 0.99);
   }

   private void renderImGuiWidgets()
   {
      ImGui.text("Demo Poses: ");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Home Pose")))
      {
         armsConfiguration.replace(RobotSide.LEFT, robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
         armsConfiguration.replace(RobotSide.RIGHT, robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.HOME));
         chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
         pelvisPosition = new Point3D(0.0, 0.0, 1.0);
         executePose(teleoperationParameters.getTrajectoryTime());
      }
      if (ImGui.button(labels.get("Greeting")))
      {
         if (usedFirstMode)
         {
            armsConfiguration.put(RobotSide.LEFT, robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {0.23, -0.95, -1.22, -1.76});
            chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 1.0);
            executePose(1.0);
         }
         else
         {
            armsConfiguration.put(RobotSide.LEFT, robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {0.23, -0.63, -1.22, -1.52});
            chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 1.0);
            executePose(1.0);
         }
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Squat")))
      {
         if (usedFirstMode)
         {
            armsConfiguration.replace(RobotSide.LEFT, robotModel.getPresetArmConfiguration(RobotSide.LEFT, PresetArmConfiguration.HOME));
            armsConfiguration.replace(RobotSide.RIGHT, robotModel.getPresetArmConfiguration(RobotSide.RIGHT, PresetArmConfiguration.HOME));
            chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 0.95);
         }
         else
         {
            armsConfiguration.put(RobotSide.LEFT, new double[] {-0.7, 0.8, 0.53, -1.9});
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {-0.7, -0.8, -0.53, -1.9});
            chestOrientation = new YawPitchRoll(0.0, Math.toRadians(10), 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 0.75);
         }
         executePose(teleoperationParameters.getTrajectoryTime());
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Flex")))
      {
         if (usedFirstMode)
         {
            armsConfiguration.put(RobotSide.LEFT, new double[] {0.71, 1.4, 1.12, -2.32});
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {0.71, -1.4, -1.12, -2.32});
            chestOrientation = new YawPitchRoll(0.0, 0.0, 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 1.0);
         }
         else
         {
            armsConfiguration.put(RobotSide.LEFT, new double[] {0.02, 1.1, -0.96, -2.32});
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {0.02, -1.1, 0.96, -2.32});
            chestOrientation = new YawPitchRoll(0.0, Math.toRadians(35), 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 0.98);
         }
         executePose(teleoperationParameters.getTrajectoryTime());
         usedFirstMode = !usedFirstMode;
      }

      if (ImGui.button(labels.get("Ballet")))
      {
         if (usedFirstMode)
         {
            armsConfiguration.put(RobotSide.LEFT, new double[] {-0.39, 2.21, 0.48, -2.32});
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {0.02, -0.7, 0.44, -1.7});
            chestOrientation = new YawPitchRoll(Math.toRadians(30), Math.toRadians(10), 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 1.0);
         }
         else
         {
            armsConfiguration.put(RobotSide.LEFT, new double[] {0.6, 0.8, -0.91, -1.84});
            armsConfiguration.replace(RobotSide.RIGHT, new double[] {-0.56, -0.31, -0.53, -2.24});
            chestOrientation = new YawPitchRoll(0.0, Math.toRadians(30), 0.0);
            pelvisPosition = new Point3D(0.0, 0.0, 0.90);
         }
         executePose(teleoperationParameters.getTrajectoryTime());
         usedFirstMode = !usedFirstMode;
      }
   }

   public void executePose(double trajectoryTime)
   {
      for (RobotSide side : RobotSide.values())
      {
         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                     trajectoryTime,
                                                                                                     armsConfiguration.get(side));
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
      }

      FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
      frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      frameChestYawPitchRoll.set(chestOrientation);
      ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
      chestTrajectoryMessage.getSo3Trajectory()
                            .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime,
                                                                                 frameChestYawPitchRoll,
                                                                                 EuclidCoreTools.zeroVector3D,
                                                                                 syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setXSelected(true);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setYSelected(true);
      chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setZSelected(true);
      ros2ControllerHelper.publishToController(chestTrajectoryMessage);

      FramePose3D syncedPose = new FramePose3D(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      syncedPose.changeFrame(syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame());
      syncedPose.getTranslation().setZ(pelvisPosition.getZ());

      PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
      message.getSe3Trajectory()
             .set(HumanoidMessageTools.createSE3TrajectoryMessage(trajectoryTime,
                                                                  syncedPose.getPosition(),
                                                                  syncedPose.getOrientation(),
                                                                  syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      message.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(false);
      message.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(false);
      message.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(true);
      ros2ControllerHelper.publishToController(message);
   }
}
