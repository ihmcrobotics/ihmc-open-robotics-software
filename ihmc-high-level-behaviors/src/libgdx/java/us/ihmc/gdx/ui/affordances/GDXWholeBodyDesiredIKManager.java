package us.ihmc.gdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;

public class GDXWholeBodyDesiredIKManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FullHumanoidRobotModel desiredRobot;
   private FullHumanoidRobotModel workingRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final GDXTeleoperationParameters teleoperationParameters;

   private final ArmJointName[] armJointNames;
   private HandDataType handPoseDataTypeToSend = HandDataType.JOINT_ANGLES;

   private final SideDependentList<GDXArmDesiredIKManager> armManagers = new SideDependentList<>(GDXArmDesiredIKManager::new);

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private enum HandDataType
   {
      JOINT_ANGLES, POSE_WORLD, POSE_CHEST
   }

   public GDXWholeBodyDesiredIKManager(DRCRobotModel robotModel,
                                       ROS2SyncedRobotModel syncedRobot,
                                       FullHumanoidRobotModel desiredRobot,
                                       ROS2ControllerHelper ros2Helper,
                                       GDXTeleoperationParameters teleoperationParameters)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.ros2Helper = ros2Helper;
      this.teleoperationParameters = teleoperationParameters;
      armJointNames = robotModel.getJointMap().getArmJointNames();
   }

   public void create()
   {
      workingRobot = robotModel.createFullRobotModel();

      for (RobotSide side : RobotSide.values)
      {
         armManagers.get(side).create(robotModel, syncedRobot.getFullRobotModel(), desiredRobot, workingRobot);
      }
   }

   // TODO this update should be moved into the control ring, and should use the control ring pose.
   public void update(SideDependentList<GDXHandInteractable> handInteractables)
   {
      boolean desiredHandsChanged = false;
      for (RobotSide side : handInteractables.sides())
      {
         armManagers.get(side).update(handInteractables.get(side), desiredRobot);

         // We only want to evaluate this when we are going to take action on it
         // Otherwise, we will not notice the desired changed while the solver was still solving
         if (readyToSolve)
         {
            desiredHandsChanged |= armManagers.get(side).getArmDesiredChanged();
         }
      }

      // The following puts the solver on a thread as to not slow down the UI
      if (readyToSolve && desiredHandsChanged)
      {
         readyToSolve = false;
         for (RobotSide side : handInteractables.sides())
         {
            armManagers.get(side).copyActualToWork();
         }

         MissingThreadTools.startAThread("IKSolver", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
         {
            try
            {
               for (RobotSide side : handInteractables.sides())
               {
                  armManagers.get(side).solve();
               }
            }
            finally
            {
               readyToCopySolution = true;
            }
         });
      }
      if (readyToCopySolution)
      {
         readyToCopySolution = false;
         for (RobotSide side : handInteractables.sides())
         {
            armManagers.get(side).copyWorkToDesired();
         }

         readyToSolve = true;
      }

      desiredRobot.getRootJoint().setJointConfiguration(syncedRobot.getFullRobotModel().getRootJoint().getJointPose());

      // TODO Update the spine joints
      desiredRobot.getRootJoint().updateFramesRecursively();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Arm setpoints:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Joint angles"), handPoseDataTypeToSend == HandDataType.JOINT_ANGLES))
      {
         handPoseDataTypeToSend = HandDataType.JOINT_ANGLES;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Pose World"), handPoseDataTypeToSend == HandDataType.POSE_WORLD))
      {
         handPoseDataTypeToSend = HandDataType.POSE_WORLD;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Pose Chest"), handPoseDataTypeToSend == HandDataType.POSE_CHEST))
      {
         handPoseDataTypeToSend = HandDataType.POSE_CHEST;
      }
   }

   public Runnable getSubmitDesiredArmSetpointsCallback(RobotSide robotSide)
   {
      Runnable runnable = () ->
      {
         if (handPoseDataTypeToSend == HandDataType.JOINT_ANGLES)
         {
            double[] jointAngles = new double[armJointNames.length];
            int i = -1;
            for (ArmJointName armJoint : armJointNames)
            {
               jointAngles[++i] = desiredRobot.getArmJoint(robotSide, armJoint).getQ();
            }

            LogTools.info("Sending ArmTrajectoryMessage");
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        jointAngles);
            ros2Helper.publishToController(armTrajectoryMessage);
         }
         else if (handPoseDataTypeToSend == HandDataType.POSE_WORLD || handPoseDataTypeToSend == HandDataType.POSE_CHEST)
         {
            FramePose3D desiredControlFramePose = armManagers.get(robotSide).getDesiredControlFramePose();

            ReferenceFrame frame;
            if (handPoseDataTypeToSend == HandDataType.POSE_WORLD)
            {
               frame = ReferenceFrame.getWorldFrame();
            }
            else
            {
               frame = syncedRobot.getReferenceFrames().getChestFrame();
            }

            desiredControlFramePose.changeFrame(frame);

            LogTools.info("Sending HandTrajectoryMessage");
            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                           teleoperationParameters.getTrajectoryTime(),
                                                                                                           desiredControlFramePose,
                                                                                                           frame);
            long dataFrameId = MessageTools.toFrameId(frame);
            handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(dataFrameId);
            ros2Helper.publishToController(handTrajectoryMessage);

            desiredControlFramePose.changeFrame(desiredRobot.getChest().getBodyFixedFrame());
         }
      };
      return runnable;
   }

   public void setDesiredToCurrent()
   {
      for (RobotSide side : RobotSide.values)
      {
         armManagers.get(side).setDesiredToCurrent();
      }
   }
}