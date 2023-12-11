package us.ihmc.rdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
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
   private final static double SAKE_HAND_SAFE_FINGER_ANGLE = 15.0;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXDesiredRobot desiredRobot;

   private final RDXTeleoperationParameters teleoperationParameters;
   private final SideDependentList<RDXInteractableHand> interactableHands;

   private final SideDependentList<ArmJointName[]> armJointNames = new SideDependentList<>();
   private RDXArmControlMode armControlMode = RDXArmControlMode.JOINT_ANGLES;
   private final RDXHandConfigurationManager handManager;

   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> desiredRobotArmJoints = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators = new SideDependentList<>();
   private final ImBoolean indicateWrenchOnScreen = new ImBoolean(false);
   private RDX3DPanelHandWrenchIndicator panelHandWrenchIndicator;

   private final ImInt selectedArmConfiguration = new ImInt();
   private final String[] armConfigurationNames = new String[PresetArmConfiguration.values.length];

   private final TypedNotification<RobotSide> showWarningNotification = new TypedNotification<>();

   public RDXArmManager(CommunicationHelper communicationHelper,
                        DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        RDXDesiredRobot desiredRobot,
                        RDXTeleoperationParameters teleoperationParameters,
                        SideDependentList<RDXInteractableHand> interactableHands)
   {
      this.communicationHelper = communicationHelper;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.teleoperationParameters = teleoperationParameters;
      this.interactableHands = interactableHands;

      for (RobotSide side : RobotSide.values)
      {
         handWrenchCalculators.put(side, new ROS2HandWrenchCalculator(side, syncedRobot));
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
         ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames(side);
         desiredRobotArmJoints.put(side, FullRobotModelUtils.getArmJoints(desiredRobot.getDesiredFullRobotModel(), side, armJointNames));
         this.armJointNames.put(side, armJointNames);
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

   public void update()
   {
      handManager.update();

      boolean desiredHandPoseChanged = false;
      for (RobotSide side : interactableHands.sides())
      {
         handWrenchCalculators.get(side).compute();
         armIKSolvers.get(side).update(syncedRobot.getReferenceFrames().getChestFrame(),
                                       interactableHands.get(side).getControlReferenceFrame());

         // wrench expressed in wrist pitch body fixed-frame
         boolean showWrench = indicateWrenchOnScreen.get();
         if (interactableHands.get(side).getEstimatedHandWrenchArrows().getShow() != showWrench)
            interactableHands.get(side).getEstimatedHandWrenchArrows().setShow(showWrench);
         interactableHands.get(side).updateEstimatedWrench(handWrenchCalculators.get(side).getFilteredWrench());

         // Check if the desired hand pose changed and we need to run the solver again.
         // We only want to evaluate this when we are going to take action on it
         // Otherwise, we will not notice the desired changed while the solver was still solving
         if (readyToSolve)
         {
            desiredHandPoseChanged |= armIKSolvers.get(side).getDesiredHandControlPoseChanged();
         }

         if (showWrench)
         {
            panelHandWrenchIndicator.update(side,
                                            handWrenchCalculators.get(side).getLinearWrenchMagnitude(true),
                                            handWrenchCalculators.get(side).getAngularWrenchMagnitude(true));
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

      ImGui.text("Arm & hand control mode:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Joint angles (DDogleg)"), armControlMode == RDXArmControlMode.JOINT_ANGLES))
      {
         armControlMode = RDXArmControlMode.JOINT_ANGLES;
      }
      ImGui.text("Hand pose only:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("World"), armControlMode == RDXArmControlMode.POSE_WORLD))
      {
         armControlMode = RDXArmControlMode.POSE_WORLD;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Chest"), armControlMode == RDXArmControlMode.POSE_CHEST))
      {
         armControlMode = RDXArmControlMode.POSE_CHEST;
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
           syncedRobot.getLatestHandJointAnglePacket(side).getJointAngles().get(0) > Math.toRadians(SAKE_HAND_SAFE_FINGER_ANGLE))
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
      double[] jointAngles = robotModel.getPresetArmConfiguration(side, presetArmConfiguration);
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                  trajectoryTime,
                                                                                                  jointAngles);
      communicationHelper.publishToController(armTrajectoryMessage);
   }

   public Runnable getSubmitDesiredArmSetpointsCallback(RobotSide robotSide)
   {
      Runnable runnable = () ->
      {
         if (armControlMode == RDXArmControlMode.JOINT_ANGLES)
         {
            double[] jointAngles = new double[armJointNames.get(robotSide).length];
            int i = -1;
            for (ArmJointName armJoint : armJointNames.get(robotSide))
            {
               jointAngles[++i] = desiredRobot.getDesiredFullRobotModel().getArmJoint(robotSide, armJoint).getQ();
            }

            LogTools.info("Sending ArmTrajectoryMessage");
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        jointAngles);
            communicationHelper.publishToController(armTrajectoryMessage);
         }
         else if (armControlMode == RDXArmControlMode.POSE_WORLD || armControlMode == RDXArmControlMode.POSE_CHEST)
         {
            FramePose3D desiredControlFramePose = new FramePose3D(interactableHands.get(robotSide).getControlReferenceFrame());

            ReferenceFrame frame;
            if (armControlMode == RDXArmControlMode.POSE_WORLD)
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
            communicationHelper.publishToController(handTrajectoryMessage);
         }
      };
      return runnable;
   }

   public RDXHandConfigurationManager getHandManager()
   {
      return handManager;
   }

   public RDXArmControlMode getArmControlMode()
   {
      return armControlMode;
   }
}