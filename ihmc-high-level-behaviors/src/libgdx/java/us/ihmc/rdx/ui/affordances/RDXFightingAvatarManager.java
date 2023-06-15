package us.ihmc.rdx.ui.affordances;

import controller_msgs.msg.dds.*;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.UUID;

public class RDXFightingAvatarManager
{
   private RDXBaseUI baseUI = null;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final RDXTeleoperationParameters teleoperationParameters;

   private final SideDependentList<double[]> armsHome = new SideDependentList<>();
   private final YawPitchRoll chestHome = new YawPitchRoll(0.475, -0.046, 0.0);
   private final SideDependentList<Pose3D> feetHome = new SideDependentList<>();
   private static final double pelvisHeightHome = 1.077;
   private long sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;

   public RDXFightingAvatarManager(DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        ROS2ControllerHelper ros2Helper,
                        RDXTeleoperationParameters teleoperationParameters)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.ros2Helper = ros2Helper;
      this.teleoperationParameters = teleoperationParameters;

      armsHome.put(RobotSide.LEFT, new double[] {-0.221, -0.124, -0.4, -2.613, -0.0, -0.0, 0.0});
      armsHome.put(RobotSide.RIGHT, new double[] {0.4, -0.124, 0.7, -2.6, 0.0, 0.0, -0.0});

      feetHome.put(RobotSide.LEFT, new Pose3D(0.10, 0.0, 0.0, 0.0, 0.0, 0.0));
      feetHome.put(RobotSide.RIGHT, new Pose3D(-0.20, 0.0, 0.0, -Math.toRadians(45), 0.0, 0.0));
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
                                                                                                        armsHome.get(side));
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

         double trajectoryTime = teleoperationParameters.getPelvisHeightChangeVelocity();
         PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
         message.getEuclideanTrajectory()
                .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(trajectoryTime,
                                                                           new Point3D(0.0, 0.0, pelvisHeightHome),
                                                                           ReferenceFrame.getWorldFrame()));
         frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
         message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
         message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
         message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
         ros2Helper.publishToController(message);
      }
      ImGui.sameLine();
      if(ImGui.button(labels.get("Guard Stance")))
      {
         sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
         FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
         footstepDataListMessage.setDefaultSwingDuration(robotModel.getWalkingControllerParameters().getDefaultSwingTime());
         footstepDataListMessage.setDefaultTransferDuration(robotModel.getWalkingControllerParameters().getDefaultTransferTime());
         footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);

         SideDependentList<MovingReferenceFrame> feetFrames = syncedRobot.getFullRobotModel().getSoleFrames();
         SideDependentList<RigidBodyTransform> currentFeetTransformToWorld = new SideDependentList<>();
         SideDependentList<FramePose3D> feetFramePosesToSend = new SideDependentList<>();

         for (RobotSide side : RobotSide.values)
            currentFeetTransformToWorld.put(side,feetFrames.get(side).getTransformToRoot());

         // check feet are squared up
         if (currentFeetTransformToWorld.get(RobotSide.LEFT).getRotation().geometricallyEquals(currentFeetTransformToWorld.get(RobotSide.RIGHT).getRotation(), 0.01))
         {
            for (RobotSide side : RobotSide.values)
            {
               feetFramePosesToSend.put(side, new FramePose3D(feetFrames.get(side), feetHome.get(side)));
               feetFramePosesToSend.get(side).changeFrame(ReferenceFrame.getWorldFrame());
            }
         } // if not align right foot with left one
         else
         {
            feetFramePosesToSend.put(RobotSide.LEFT, new FramePose3D(feetFrames.get(RobotSide.LEFT)));
            feetFramePosesToSend.put(RobotSide.RIGHT, new FramePose3D(feetFrames.get(RobotSide.LEFT)));
            feetFramePosesToSend.get(RobotSide.RIGHT).appendTranslation(0.0, -0.260, 0.0);
            feetFramePosesToSend.get(RobotSide.LEFT).changeFrame(ReferenceFrame.getWorldFrame());
            feetFramePosesToSend.get(RobotSide.RIGHT).changeFrame(ReferenceFrame.getWorldFrame());
         }

         for (RobotSide side : RobotSide.values)
         {
            FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();
            footstepDataMessage.setSequenceId(sequenceId++);
            footstepDataMessage.setRobotSide(side.toByte());
            footstepDataMessage.getLocation().set(feetFramePosesToSend.get(side).getPosition());
            footstepDataMessage.getOrientation().set(feetFramePosesToSend.get(side).getOrientation());
            footstepDataMessage.setTrajectoryType(TrajectoryType.DEFAULT.toByte());
         }
         ros2Helper.publishToController(footstepDataListMessage);
      }
   }
}
