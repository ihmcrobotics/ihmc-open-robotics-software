package us.ihmc.rdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
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
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final CommunicationHelper communicationHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXDesiredRobot desiredRobot;

   private final RDXTeleoperationParameters teleoperationParameters;
   private final SideDependentList<RDXInteractableHand> interactableHands;

   private final ArmJointName[] armJointNames;
   private RDXArmControlMode armControlMode = RDXArmControlMode.JOINT_ANGLES;
   private final SideDependentList<double[]> armsWide = new SideDependentList<>();
   private final SideDependentList<double[]> doorAvoidanceArms = new SideDependentList<>();
   private final RDXHandConfigurationManager handManager;

   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> desiredRobotArmJoints = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private final HandWrenchCalculator handWrenchCalculator;
   private final ImBoolean indicateWrenchOnScreen = new ImBoolean(false);
   private RDX3DPanelHandWrenchIndicator panelHandWrenchIndicator;

   public RDXArmManager(CommunicationHelper communicationHelper,
                        DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        RDXDesiredRobot desiredRobot,
                        RDXTeleoperationParameters teleoperationParameters,
                        SideDependentList<RDXInteractableHand> interactableHands)
   {
      this.communicationHelper = communicationHelper;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.teleoperationParameters = teleoperationParameters;
      this.interactableHands = interactableHands;
      armJointNames = robotModel.getJointMap().getArmJointNames();

      for (RobotSide side : RobotSide.values)
      {
         armsWide.put(side,
                      new double[] {0.6,
                                    side.negateIfRightSide(0.3),
                                    side.negateIfRightSide(-0.5),
                                    -1.0,
                                    side.negateIfRightSide(-0.6),
                                    0.000,
                                    side.negateIfLeftSide(0.0)});
      }
      doorAvoidanceArms.put(RobotSide.LEFT, new double[] {-0.121, -0.124, -0.971, -1.713, -0.935, -0.873, 0.277});
      doorAvoidanceArms.put(RobotSide.RIGHT, new double[] {-0.523, -0.328, 0.586, -2.192, 0.828, 1.009, -0.281});

      handWrenchCalculator = new HandWrenchCalculator(syncedRobot);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
         desiredRobotArmJoints.put(side, FullRobotModelUtils.getArmJoints(desiredRobot.getDesiredFullRobotModel(), side, robotModel.getJointMap().getArmJointNames()));
      }

      handManager = new RDXHandConfigurationManager();
   }

   public void create(RDXBaseUI baseUI)
   {
      baseUI.getImGuiPanelManager().addPanel("Arms & Hands", this::renderImGuiWidgets);

      panelHandWrenchIndicator = new RDX3DPanelHandWrenchIndicator(baseUI.getPrimary3DPanel());
      RDX3DPanelToolbarButton wrenchToolbarButton = baseUI.getPrimary3DPanel().addToolbarButton();
      wrenchToolbarButton.loadAndSetIcon("icons/handWrench.png");
      wrenchToolbarButton.setTooltipText("Show / hide estimated hand wrench");
      wrenchToolbarButton.setOnPressed(() -> indicateWrenchOnScreen.set(!indicateWrenchOnScreen.get()));
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(() ->
      {
         if (indicateWrenchOnScreen.get())
            panelHandWrenchIndicator.renderImGuiOverlay();
      });

      handManager.create(baseUI, communicationHelper, syncedRobot);
   }

   public void update()
   {
      boolean desiredHandPoseChanged = false;

      handWrenchCalculator.compute();

      for (RobotSide side : interactableHands.sides())
      {
         armIKSolvers.get(side).update(interactableHands.get(side).getControlReferenceFrame());

         // wrench expressed in wrist pitch body fixed-frame
         boolean showWrench = indicateWrenchOnScreen.get();
         if (interactableHands.get(side).getEstimatedHandWrenchArrows().getShow() != showWrench)
            interactableHands.get(side).getEstimatedHandWrenchArrows().setShow(showWrench);
         interactableHands.get(side).updateEstimatedWrench(handWrenchCalculator.getFilteredWrench().get(side));

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
                                            handWrenchCalculator.getLinearWrenchMagnitude(side, true),
                                            handWrenchCalculator.getAngularWrenchMagnitude(side, true));
         }
      }

      // The following puts the solver on a thread as to not slow down the UI
      if (readyToSolve && desiredHandPoseChanged)
      {
         readyToSolve = false;
         for (RobotSide side : interactableHands.sides())
         {
            armIKSolvers.get(side).copyActualToWork();
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

      ImGui.text("Arms:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Home " + side.getPascalCaseName())))
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
      }

      ImGui.text("Wide Arms:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Wide " + side.getPascalCaseName())))
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        armsWide.get(side));
            communicationHelper.publishToController(armTrajectoryMessage);
         }
      }

      ImGui.text("Door avoidance arms:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(side.getPascalCaseName())))
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        doorAvoidanceArms.get(side));
            communicationHelper.publishToController(armTrajectoryMessage);
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
   }

   public Runnable getSubmitDesiredArmSetpointsCallback(RobotSide robotSide)
   {
      Runnable runnable = () ->
      {
         if (armControlMode == RDXArmControlMode.JOINT_ANGLES)
         {
            double[] jointAngles = new double[armJointNames.length];
            int i = -1;
            for (ArmJointName armJoint : armJointNames)
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

   public RDXArmControlMode getArmControlMode()
   {
      return armControlMode;
   }
}