package us.ihmc.rdx.ui.affordances;

import controller_msgs.ArmTrajectoryMessage;
import controller_msgs.GoHomeMessage;
import controller_msgs.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.HandTrajectoryMessage;
import controller_msgs.JointspaceTrajectoryMessage;
import controller_msgs.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.QueueableMessage;
import ihmc_common_msgs.SE3TrajectoryMessage;
import ihmc_common_msgs.SE3TrajectoryPointMessage;
import ihmc_common_msgs.TrajectoryPoint1DMessage;
import imgui.ImGui;
import imgui.flag.ImGuiTreeNodeFlags;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXDesiredRobot;
import us.ihmc.rdx.ui.hands.RDXHandManager;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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
   private final ROS2ControllerHelper controllerHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXDesiredRobot desiredRobot;

   private final RDXTeleoperationParameters teleoperationParameters;
   private final SideDependentList<RDXInteractableHand> interactableHands;
   private final BooleanSupplier enableWholeBodyIK;

   private final SideDependentList<ArmJointName[]> armJointNames = new SideDependentList<>();
   private RDXArmControlMode armControlMode = RDXArmControlMode.JOINTSPACE;
   private ReferenceFrame taskspaceTrajectoryFrame = ReferenceFrame.getWorldFrame();
   private final RDXHandManager handManager;

   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> desiredRobotArmJoints = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private final ImBoolean indicateWrenchOnScreen = new ImBoolean(false);
   private RDX3DPanelHandWrenchIndicator panelHandWrenchIndicator;

   private final ImInt selectedArmConfiguration = new ImInt(PresetArmConfiguration.HOME.ordinal());
   private final String[] armConfigurationNames = new String[PresetArmConfiguration.values.length];

   private final TypedNotification<RobotSide> showWarningNotification = new TypedNotification<>();

   public RDXArmManager(ROS2ControllerHelper controllerHelper,
                        DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        RDXDesiredRobot desiredRobot,
                        RDXTeleoperationParameters teleoperationParameters,
                        SideDependentList<RDXInteractableHand> interactableHands,
                        BooleanSupplier enableWholeBodyIK)
   {
      this.controllerHelper = controllerHelper;
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

      handManager = new RDXHandManager(robotModel, controllerHelper.getROS2Node());
   }

   public void create(RDXBaseUI baseUI)
   {
      panelHandWrenchIndicator = new RDX3DPanelHandWrenchIndicator(baseUI.getPrimary3DPanel());

      handManager.create(baseUI);
   }

   public void update(boolean interactablesEnabled)
   {
      handManager.update();

      boolean showWrench = indicateWrenchOnScreen.get();

      for (RobotSide side : interactableHands.sides())
      {
         if (showWrench && syncedRobot.getHandWrenchCalculators().get(side) != null)
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
            
            if (syncedRobot.getHandWrenchCalculators().get(side) == null)
               continue;

            // wrench expressed in wrist pitch body fixed-frame
            if (interactableHands.get(side).getEstimatedHandWrenchArrows().getShow() != showWrench)
               interactableHands.get(side).getEstimatedHandWrenchArrows().setShow(showWrench);
            interactableHands.get(side).updateEstimatedWrench(syncedRobot.getHandWrenchCalculators().get(side).getFilteredWrench());
         }

         // The following puts the solver on a thread as to not slow down the UI
         if (readyToSolve && desiredHandPoseChanged)
         {
            readyToSolve = false;
            for (RobotSide side : interactableHands.sides())
            {
               armIKSolvers.get(side).copySourceToWork();
            }

            ThreadTools.startAThread(() ->
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
            }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, "IKSolver");
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
      if (ImGui.collapsingHeader(labels.get("Arms")))
      {
         float widgetStartX = 112.0f;

         ImGui.text("Arm Presets:");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
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

         ImGui.text("Arm Control Mode:");
         ImGui.sameLine();
         ImGui.setCursorPosX(widgetStartX);
         if (ImGui.radioButton(labels.get("Jointspace"), armControlMode == RDXArmControlMode.JOINTSPACE))
         {
            armControlMode = RDXArmControlMode.JOINTSPACE;
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Taskspace"), armControlMode == RDXArmControlMode.TASKSPACE))
         {
            armControlMode = RDXArmControlMode.TASKSPACE;
         }

         ImGui.text("Reference Frame:");
         ImGui.sameLine();
         if (armControlMode == RDXArmControlMode.JOINTSPACE)
            ImGui.beginDisabled();
         ImGui.setCursorPosX(widgetStartX);
         if (ImGui.radioButton(labels.get("World"), taskspaceTrajectoryFrame == ReferenceFrame.getWorldFrame()))
         {
            taskspaceTrajectoryFrame = ReferenceFrame.getWorldFrame();
         }
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("Chest"), taskspaceTrajectoryFrame == syncedRobot.getReferenceFrames().getChestFrame()))
         {
            taskspaceTrajectoryFrame = syncedRobot.getReferenceFrames().getChestFrame();
         }
         if (armControlMode == RDXArmControlMode.JOINTSPACE)
            ImGui.endDisabled();

         ImGui.separator();
         if (ImGui.checkbox(labels.get("Display EE Wrenches"), indicateWrenchOnScreen))
         {
            if (indicateWrenchOnScreen.get())
               RDXBaseUI.getInstance().getPrimary3DPanel().addOverlayPanel("Hand wrenches", () -> panelHandWrenchIndicator.renderImGuiOverlay());
            else
               RDXBaseUI.getInstance().getPrimary3DPanel().removeOverlayPanel("Hand wrenches");
         }
      }

      boolean hasHand = false;
      for (RobotSide side : RobotSide.values)
      {
         hasHand |= handManager.getHand(side) != null;
      }
      if (hasHand)
      {
         handManager.renderImGuiWidgets();
      }
   }

   public void destroy()
   {
      handManager.destroy();
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
      controllerHelper.publishToController(armHomeMessage);
   }

   public void executeArmAngles(RobotSide side, PresetArmConfiguration presetArmConfiguration, double trajectoryTime)
   {
      RDXBaseUI.pushNotification("Commanding arm trajectory...");
      double[] jointAngles = robotModel.getPresetArmConfiguration(side, presetArmConfiguration);
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                  trajectoryTime,
                                                                                                  jointAngles);
      controllerHelper.publishToController(armTrajectoryMessage);
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
      se3TrajectoryPointMessage.getLinearVelocity().getVector().setToZero();
      se3TrajectoryPointMessage.getAngularVelocity().getVector().setToZero();
      se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameID);

      switch (armControlMode)
      {
         case JOINTSPACE ->
         {
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
            armTrajectoryMessage.setRobotSide(robotSide.toByte());
            armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
            RDXBaseUI.pushNotification("Commanding arm jointspace trajectory...");
            controllerHelper.publishToController(armTrajectoryMessage);
         }
         case TASKSPACE ->
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridJointspaceTaskspaceTrajectoryMessage
                  = new HandHybridJointspaceTaskspaceTrajectoryMessage();
            handHybridJointspaceTaskspaceTrajectoryMessage.setRobotSide(robotSide.toByte());
            handHybridJointspaceTaskspaceTrajectoryMessage.getTaskspaceTrajectoryMessage().set(se3TrajectoryMessage);
            handHybridJointspaceTaskspaceTrajectoryMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
            RDXBaseUI.pushNotification("Commanding arm hybrid jointspace taskpace trajectory...");
            controllerHelper.publishToController(handHybridJointspaceTaskspaceTrajectoryMessage);
         }
      }
   }

   public RDXHandManager getHandManager()
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