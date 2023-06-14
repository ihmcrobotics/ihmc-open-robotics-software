package us.ihmc.rdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXFightingAvatarManager
{
   private RDXBaseUI baseUI = null;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final RDXTeleoperationParameters teleoperationParameters;

   private final SideDependentList<double[]> armHomes = new SideDependentList<>();
   private final YawPitchRoll chestHome = new YawPitchRoll(0.6, -0.046, 0.0);

   public RDXFightingAvatarManager(DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        ROS2ControllerHelper ros2Helper,
                        RDXTeleoperationParameters teleoperationParameters)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2Helper = ros2Helper;
      this.teleoperationParameters = teleoperationParameters;

      for (RobotSide side : RobotSide.values)
      {
         armHomes.put(side,
                      new double[] {0.5,
                                    side.negateIfRightSide(0.0),
                                    side.negateIfRightSide(-0.5),
                                    -1.0,
                                    side.negateIfRightSide(0.0),
                                    0.000,
                                    side.negateIfLeftSide(0.0)});
      }
      armHomes.put(RobotSide.LEFT, new double[] {-0.221, -0.124, -0.4, -2.613, -0.0, -0.0, 0.0});
      armHomes.put(RobotSide.RIGHT, new double[] {0.4, -0.124, 0.7, -2.6, 0.0, 0.0, -0.0});
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Fighting avatar: ");
      ImGui.sameLine();
      if(ImGui.button(labels.get("Guard Pose")))
      {
         for (RobotSide side : RobotSide.values)
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        armHomes.get(side));
            ros2Helper.publishToController(armTrajectoryMessage);
         }

         FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
         frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
         frameChestYawPitchRoll.set(chestHome);
         frameChestYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
         chestTrajectoryMessage.getSo3Trajectory()
                .set(HumanoidMessageTools.createSO3TrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                     frameChestYawPitchRoll,
                                                                     EuclidCoreTools.zeroVector3D,
                                                                     syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
         chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setXSelected(true);
         chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setYSelected(true);
         chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setZSelected(true);
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         chestTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
         ros2Helper.publishToController(chestTrajectoryMessage);
      }
      ImGui.sameLine();
      if(ImGui.button(labels.get("Guard Stance")))
      {
         for (RobotSide side : RobotSide.values)
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        armHomes.get(side));
            ros2Helper.publishToController(armTrajectoryMessage);
         }

         FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
         frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
         frameChestYawPitchRoll.set(chestHome);
         frameChestYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());
         ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
         chestTrajectoryMessage.getSo3Trajectory()
                               .set(HumanoidMessageTools.createSO3TrajectoryMessage(teleoperationParameters.getTrajectoryTime(),
                                                                                    frameChestYawPitchRoll,
                                                                                    EuclidCoreTools.zeroVector3D,
                                                                                    syncedRobot.getReferenceFrames().getPelvisZUpFrame()));
         chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setXSelected(true);
         chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setYSelected(true);
         chestTrajectoryMessage.getSo3Trajectory().getSelectionMatrix().setZSelected(true);
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         chestTrajectoryMessage.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
         ros2Helper.publishToController(chestTrajectoryMessage);
      }
   }
}
