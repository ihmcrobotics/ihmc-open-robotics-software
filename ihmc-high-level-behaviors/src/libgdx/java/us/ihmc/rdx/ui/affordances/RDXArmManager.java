package us.ihmc.rdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanelToolbarButton;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;

/**
 * This class manages the UI for operating the arms of a humanoid robot.
 * This includes sending the arms to predefined joint angles poses and
 * operating a IK solver to acheive desired hand and elbow poses.
 */
public class RDXArmManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final FullHumanoidRobotModel desiredRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final SideDependentList<RDXInteractableHand> interactableHands;

   private final ArmJointName[] armJointNames;
   private RDXArmControlMode armControlMode = RDXArmControlMode.JOINT_ANGLES;
   private final SideDependentList<double[]> armHomes = new SideDependentList<>();
   private final SideDependentList<double[]> doorAvoidanceArms = new SideDependentList<>();

   private final SideDependentList<RDXDDogLegArmIKSolver> armIKSolvers = new SideDependentList<>();

   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private final HandWrenchCalculator handWrenchCalculator;
   private final ImBoolean indicateWrenchOnScreen = new ImBoolean(true);
   private RDX3DPanelToolbarButton wrenchToolbarButton;
   private RDX3DPanelHandWrenchIndicator panelHandWrenchIndicator;

   public RDXArmManager(DRCRobotModel robotModel,
                        ROS2SyncedRobotModel syncedRobot,
                        FullHumanoidRobotModel desiredRobot,
                        ROS2ControllerHelper ros2Helper,
                        RDXTeleoperationParameters teleoperationParameters,
                        SideDependentList<RDXInteractableHand> interactableHands)
   {
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.ros2Helper = ros2Helper;
      this.teleoperationParameters = teleoperationParameters;
      this.interactableHands = interactableHands;
      armJointNames = robotModel.getJointMap().getArmJointNames();

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
      doorAvoidanceArms.put(RobotSide.LEFT, new double[] {-0.121, -0.124, -0.971, -1.713, -0.935, -0.873, 0.277});
      doorAvoidanceArms.put(RobotSide.RIGHT, new double[] {-0.523, -0.328, 0.586, -2.192, 0.828, 1.009, -0.281});

      handWrenchCalculator = new HandWrenchCalculator(syncedRobot);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side,
                          new RDXDDogLegArmIKSolver(side,
                                                   robotModel,
                                                   syncedRobot.getFullRobotModel(),
                                                   desiredRobot,
                                                   interactableHands.get(side).getControlReferenceFrame()));
      }
   }

   public void create(RDXBaseUI baseUI)
   {
      panelHandWrenchIndicator = new RDX3DPanelHandWrenchIndicator(baseUI.getPrimary3DPanel());
      wrenchToolbarButton = baseUI.getPrimary3DPanel().addToolbarButton();
      wrenchToolbarButton.loadAndSetIcon("icons/handWrench.png");
      wrenchToolbarButton.setTooltipText("Show / hide estimated hand wrench");
      wrenchToolbarButton.setOnPressed(()->
                                       {
                                          boolean showWrench = !indicateWrenchOnScreen.get();
                                          indicateWrenchOnScreen.set(showWrench);
                                          panelHandWrenchIndicator.setShowAndUpdate(showWrench);
                                       });
      baseUI.getPrimary3DPanel().addImGuiOverlayAddition(panelHandWrenchIndicator::renderImGuiOverlay);
   }

   public void update()
   {
      boolean desiredHandPoseChanged = false;

      handWrenchCalculator.compute();

      for (RobotSide side : interactableHands.sides())
      {
         armIKSolvers.get(side).update();

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

         panelHandWrenchIndicator.update(side,
                                         handWrenchCalculator.getLinearWrenchMagnitude(side, true),
                                         handWrenchCalculator.getAngularWrenchMagnitude(side, true));
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
            armIKSolvers.get(side).copyWorkToDesired();
         }

         readyToSolve = true;
      }

      desiredRobot.getRootJoint().setJointConfiguration(syncedRobot.getFullRobotModel().getRootJoint().getJointPose());

      // TODO Update the spine joints
      desiredRobot.getRootJoint().updateFramesRecursively();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Arms:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Home " + side.getPascalCaseName())))
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(side,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        armHomes.get(side));
            ros2Helper.publishToController(armTrajectoryMessage);
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
            ros2Helper.publishToController(armTrajectoryMessage);
         }
      }

      ImGui.text("Arm & hand control mode:");
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

      if (ImGui.checkbox(labels.get("Hand wrench magnitudes on 3D View"), indicateWrenchOnScreen))
      {
         panelHandWrenchIndicator.setShowAndUpdate(indicateWrenchOnScreen.get());
      }
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
               jointAngles[++i] = desiredRobot.getArmJoint(robotSide, armJoint).getQ();
            }

            LogTools.info("Sending ArmTrajectoryMessage");
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        jointAngles);
            ros2Helper.publishToController(armTrajectoryMessage);
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
            ros2Helper.publishToController(handTrajectoryMessage);
         }
      };
      return runnable;
   }

   public void setDesiredToCurrent()
   {
      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.get(side).setDesiredToCurrent();
      }
   }

   public RDXArmControlMode getArmControlMode()
   {
      return armControlMode;
   }
}