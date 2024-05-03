package us.ihmc.rdx.ui.affordances;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXDesiredRobot;
import us.ihmc.rdx.ui.teleoperation.RDXHandConfigurationManager;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;

import java.util.function.BooleanSupplier;

/**
 * This class manages the UI for operating the arms of a humanoid robot.
 * This includes sending the arms to predefined joint angles poses and
 * operating a IK solver to achieve desired hand and elbow poses.
 */
public class RDXArmManager
{
   /* Sake hand's fingers do not extend much outward from the hand at 15 degrees apart, i.e. they are nearly parallel
      kinda look like this:
      15 degrees apart  |  0 degrees apart

        /\    /\                 /||\
       | |    | | <- fingers -> / /\ \
       | |    | |              / /  \ \
       |  ----  |             |  ----  |
       |        | <-  palm -> |        |
       |________|             |________|
    */
   private final static double SAKE_HAND_SAFEE_FINGER_ANGLE = 15.0;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXDesiredRobot desiredRobot;

   private final RDXTeleoperationParameters teleoperationParameters;
   private final SideDependentList<RDXInteractableHand> interactableHands;
   private final BooleanSupplier enableWholeBodyIK;

   private final SideDependentList<ArmJointName[]> armJointNames = new SideDependentList<>();
   private RDXArmControlMode armControlMode = RDXArmControlMode.JOINTSPACE;
   private ReferenceFrame taskspaceTrajectoryFrame = ReferenceFrame.getWorldFrame();
   private final RDXHandConfigurationManager handManager;

   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> desiredRobotArmJoints = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private final ImBoolean indicateWrenchOnScreen = new ImBoolean(false);
   private RDX3DPanelHandWrenchIndicator panelHandWrenchIndicator;

   private final ImInt selectedArmConfiguration = new ImInt(PresetArmConfiguration.TUCKED_UP_ARMS.ordinal());
   private final String[] armConfigurationNames = new String[PresetArmConfiguration.values.length];

   private final TypedNotification<RobotSide> showWarningNotification = new TypedNotification<>();

   public RDXArmManager(CommunicationHelper communicationHelper,
                        DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        RDXDesiredRobot desiredRobot,
                        RDXTeleoperationParameters teleoperationParameters,
                        SideDependentList<RDXInteractableHand> interactableHands,
                        BooleanSupplier enableWholeBodyIK)
   {
      this.communicationHelper = communicationHelper;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.teleoperationParameters = teleoperationParameters;
      this.interactableHands = interactableHands;
      this.enableWholeBodyIK = enableWholeBodyIK;

      for (RobotSide side : RobotSide.values)
      {
         if (robotModel.getRobotVersion().hasArm(side))
         {
            armIKSolvers.put(side, new ArmIKSolver(side, robotModel.getJointMap(), syncedRobot.getFullRobotModel()));
            ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames(side);
            desiredRobotArmJoints.put(side, FullRobotModelUtils.getArmJoints(desiredRobot.getDesiredFullRobotModel(), side, armJointNames));
            this.armJointNames.put(side, armJointNames);
         }
      }

      for (int i = 0; i < PresetArmConfiguration.values.length; i++)
      {
         armConfigurationNames[i] = PresetArmConfiguration.values[i].name();
      }

      handManager = new RDXHandConfigurationManager();
   }

   public void create(RDXBaseUI baseUI)
   {
      panelHandWrenchIndicator = new RDX3DPanelHandWrenchIndicator(baseUI.getPrimary3DPanel());
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(() ->
      {
         if (indicateWrenchOnScreen.get())
            panelHandWrenchIndicator.renderImGuiOverlay();
      });

      handManager.create(baseUI, communicationHelper, syncedRobot);
   }

   public void update(boolean interactablesEnabled)
   {
      handManager.update();

      boolean showWrench = indicateWrenchOnScreen.get();

      for (RobotSide side : interactableHands.sides())
      {
         if (showWrench)
         {
            panelHandWrenchIndicator.update(side,
                                            syncedRobot.getHandWrenchCalculators().get(side).getLinearWrenchMagnitude(true),
                                            syncedRobot.getHandWrenchCalculators().get(side).getAngularWrenchMagnitude(true));
         }
      }

      if (!enableWholeBodyIK.getAsBoolean() && interactablesEnabled)
      {
         boolean desiredHandPoseChanged = false;
         for (RobotSide side : interactableHands.sides())
         {
            // wrench expressed in wrist pitch body fixed-frame
            if (interactableHands.get(side).getEstimatedHandWrenchArrows().getShow() != showWrench)
               interactableHands.get(side).getEstimatedHandWrenchArrows().setShow(showWrench);
            if (robotModel.getHandModel(side) != null)
               interactableHands.get(side).updateEstimatedWrench(syncedRobot.getHandWrenchCalculators().get(side).getFilteredWrench());

            if (!interactableHands.get(side).isDeleted())
            {
               armIKSolvers.get(side).update(syncedRobot.getReferenceFrames().getChestFrame(), interactableHands.get(side).getControlReferenceFrame());

               // Check if the desired hand pose changed and we need to run the solver again.
               // We only want to evaluate this when we are going to take action on it
               // Otherwise, we will not notice the desired changed while the solver was still solving
               if (readyToSolve)
               {
                  desiredHandPoseChanged |= armIKSolvers.get(side).getDesiredHandControlPoseChanged();
               }
            }
         }

         // The following puts the solver on a thread as to not slow down the UI
         if (readyToSolve && desiredHandPoseChanged)
         {
            readyToSolve = false;
            for (RobotSide side : interactableHands.sides())
            {
               armIKSolvers.get(side).copySourceToWork();
            }

            MissingThreadTools.startAThread("IKSolver", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
            {
               try
               {
                  for (RobotSide side : interactableHands.sides())
                  {
                     armIKSolvers.get(side).solve();
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
            for (RobotSide side : interactableHands.sides())
            {
               MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(armIKSolvers.get(side).getSolutionOneDoFJoints(), desiredRobotArmJoints.get(side));
            }

            readyToSolve = true;
         }

         desiredRobot.getDesiredFullRobotModel().getRootJoint().setJointConfiguration(syncedRobot.getFullRobotModel().getRootJoint().getJointPose());
         desiredRobot.getDesiredFullRobotModel().updateFrames();
      }
   }

   public void renderImGuiWidgets()
   {
      handManager.renderImGuiWidgets();

      ImGui.text("Arm Presets:");
      ImGui.pushItemWidth(140.0f);
      ImGui.combo(labels.getHidden("Arm Configuration Combo"), selectedArmConfiguration, armConfigurationNames);
      ImGui.popItemWidth();
      ImGui.sameLine();
      ImGui.text("Command");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(side.getPascalCaseName())))
         {
            executeArmAngles(side, PresetArmConfiguration.values[selectedArmConfiguration.get()], teleoperationParameters.getTrajectoryTime());
         }
      }

      ImGui.text("Hand control mode:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Jointspace"), armControlMode == RDXArmControlMode.JOINTSPACE))
      {
         armControlMode = RDXArmControlMode.JOINTSPACE;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Taskspace"), armControlMode == RDXArmControlMode.TASKSPACE))
      {
         armControlMode = RDXArmControlMode.TASKSPACE;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Hybrid"), armControlMode == RDXArmControlMode.HYBRID))
      {
         armControlMode = RDXArmControlMode.HYBRID;
      }

      ImGui.text("Taskspace trajectory frame:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World"), taskspaceTrajectoryFrame == ReferenceFrame.getWorldFrame()))
      {
         taskspaceTrajectoryFrame = ReferenceFrame.getWorldFrame();
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Chest"), taskspaceTrajectoryFrame == syncedRobot.getReferenceFrames().getChestFrame()))
      {
         taskspaceTrajectoryFrame = syncedRobot.getReferenceFrames().getChestFrame();
      }

      ImGui.checkbox(labels.get("Hand wrench magnitudes on 3D View"), indicateWrenchOnScreen);

      // Pop up warning if notification is set
      if (showWarningNotification.peekHasValue() && showWarningNotification.poll())
      {
         ImGui.openPopup(labels.get("Warning"));
      }

      if (ImGui.beginPopupModal(labels.get("Warning")))
      {
         ImGui.text("""
                          The hand is currently open.
                                                    
                          Continuing to door avoidance
                          may cause the hand to collide
                          with the body of the robot.""");

         ImGui.separator();
         if (ImGui.button("Continue"))
         {
            executeArmAngles(showWarningNotification.read(), PresetArmConfiguration.DOOR_AVOIDANCE, teleoperationParameters.getTrajectoryTime());
            ImGui.closeCurrentPopup();
         }
         ImGui.sameLine();
         if (ImGui.button("Cancel"))
         {
            ImGui.closeCurrentPopup();
         }
         ImGui.endPopup();
      }
   }

   public void executeArmHome(RobotSide side)
   {
      GoHomeMessage armHomeMessage = new GoHomeMessage();
      armHomeMessage.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);

      if (side == RobotSide.LEFT)
         armHomeMessage.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      else
         armHomeMessage.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);

      armHomeMessage.setTrajectoryTime(teleoperationParameters.getTrajectoryTime());
      communicationHelper.publishToController(armHomeMessage);
   }

   public void executeDoorAvoidanceArmAngles(RobotSide side)
   {
      // Warning pops up if fingers are more than 15 degrees from "zero" (zero = when fingertips are parallel)
      // i.e. when the fingers are more than 30 degrees apart from each other
      // This is an arbitrary value
      if (syncedRobot.getRobotModel().getHandModels().toString().contains("SakeHand") &&
           syncedRobot.getLatestHandJointAnglePacket(side).getJointAngles().get(0) > Math.toRadians(SAKE_HAND_SAFEE_FINGER_ANGLE))
      {
         showWarningNotification.set(side);
      }
      else
      {
         executeArmAngles(side, PresetArmConfiguration.DOOR_AVOIDANCE, teleoperationParameters.getTrajectoryTime());
      }
   }

   public void executeArmAngles(RobotSide side, PresetArmConfiguration presetArmConfiguration, double trajectoryTime)
   {
      RDXBaseUI.pushNotification("Commanding arm trajectory...");
      double[] jointAngles = robotModel.getPresetArmConfiguration(side, presetArmConfiguration);
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                  trajectoryTime,
                                                                                                  jointAngles);
      communicationHelper.publishToController(armTrajectoryMessage);
   }

   public void executeDesiredArmCommand(RobotSide robotSide)
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      for (ArmJointName armJoint : armJointNames.get(robotSide))
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(teleoperationParameters.getTrajectoryTime());
         trajectoryPoint1DMessage.setPosition(desiredRobot.getDesiredFullRobotModel().getArmJoint(robotSide, armJoint).getQ());
         trajectoryPoint1DMessage.setVelocity(0.0);
      }

      long trajectoryReferenceFrameID = MessageTools.toFrameId(taskspaceTrajectoryFrame);
      FramePose3D desiredControlFramePose = new FramePose3D(interactableHands.get(robotSide).getControlReferenceFrame());
      desiredControlFramePose.changeFrame(taskspaceTrajectoryFrame);

      SE3TrajectoryMessage se3TrajectoryMessage = new SE3TrajectoryMessage();
      se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      // Select all axes and use default weights
      SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
      se3TrajectoryPointMessage.setTime(teleoperationParameters.getTrajectoryTime());
      se3TrajectoryPointMessage.getPosition().set(desiredControlFramePose.getPosition());
      se3TrajectoryPointMessage.getOrientation().set(desiredControlFramePose.getOrientation());
      se3TrajectoryPointMessage.getLinearVelocity().setToZero();
      se3TrajectoryPointMessage.getAngularVelocity().setToZero();
      se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameID);

      switch (armControlMode)
      {
         case JOINTSPACE ->
         {
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
            armTrajectoryMessage.setRobotSide(robotSide.toByte());
            armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
            RDXBaseUI.pushNotification("Commanding arm jointspace trajectory...");
            communicationHelper.publishToController(armTrajectoryMessage);
         }
         case TASKSPACE ->
         {
            HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
            handTrajectoryMessage.setRobotSide(robotSide.toByte());
            handTrajectoryMessage.getSe3Trajectory().set(se3TrajectoryMessage);
            RDXBaseUI.pushNotification("Commanding taskspace %s frame trajectory...".formatted(taskspaceTrajectoryFrame.getName()));
            communicationHelper.publishToController(handTrajectoryMessage);
         }
         case HYBRID ->
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridJointspaceTaskspaceTrajectoryMessage
                  = new HandHybridJointspaceTaskspaceTrajectoryMessage();
            handHybridJointspaceTaskspaceTrajectoryMessage.setRobotSide(robotSide.toByte());
            handHybridJointspaceTaskspaceTrajectoryMessage.getTaskspaceTrajectoryMessage().set(se3TrajectoryMessage);
            handHybridJointspaceTaskspaceTrajectoryMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
            RDXBaseUI.pushNotification("Commanding arm hybrid jointspace taskpace trajectory...");
            communicationHelper.publishToController(handHybridJointspaceTaskspaceTrajectoryMessage);
         }
      }
   }

   public RDXHandConfigurationManager getHandManager()
   {
      return handManager;
   }

   public RDXArmControlMode getArmControlMode()
   {
      return armControlMode;
   }

   public FramePose3D getDesiredHandFramePose(RobotSide side)
   {
      if (interactableHands.containsKey(side))
         return new FramePose3D(interactableHands.get(side).getControlReferenceFrame());
      else
         return null;
   }

   public void setDesiredHandFramePose(RobotSide side, FramePose3D desiredPose)
   {
      if (interactableHands.containsKey(side))
      {
         interactableHands.get(side).selectInteractable();
         ReferenceFrame interactableHandFrame = interactableHands.get(side).getPoseGizmo().getGizmoFrame().getParent();
         desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
         interactableHands.get(side).getSyncedControlFrame().getTransformToWorldFrame().set(desiredPose);
         if (!interactableHandFrame.isWorldFrame())
            desiredPose.changeFrame(interactableHandFrame);
         interactableHands.get(side).getPoseGizmo().getTransformToParent().set(desiredPose);
      }
   }

   public double[] getDesiredJointAngles(RobotSide side)
   {
      double[] jointAngles = new double[armJointNames.get(side).length];
      int i = -1;
      for (ArmJointName armJoint : armJointNames.get(side))
      {
         jointAngles[++i] = desiredRobot.getDesiredFullRobotModel().getArmJoint(side, armJoint).getQ();
      }
      return jointAngles;
   }

   public SideDependentList<ArmIKSolver> getArmIKSolvers()
   {
      return armIKSolvers;
   }

   public void setIndicateWrenchOnScreen(boolean indicateWrenchOnScreen)
   {
      this.indicateWrenchOnScreen.set(indicateWrenchOnScreen);
   }

   public RDX3DPanelHandWrenchIndicator getPanelHandWrenchIndicator()
   {
      return panelHandWrenchIndicator;
   }
}