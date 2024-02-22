package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.LegTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.affordances.RDXInteractableFoot;
import us.ihmc.rdx.ui.affordances.RDXInteractableHand;
import us.ihmc.rdx.ui.affordances.RDXInteractableRobotLink;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.geometry.FramePose3DChangedTracker;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RDXWholeBodyIKManager
{
   private final DRCRobotModel robotModel;
   private final RDXTeleoperationParameters teleoperationParameters;
   private final RDXDesiredRobot desiredRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private SideDependentList<RDXInteractableHand> interactableHands;
   private SideDependentList<RDXInteractableFoot> interactableFeet;
   private RDXInteractableRobotLink interactableChest;
   private RDXInteractableRobotLink interactablePelvis;
   private final SideDependentList<FramePose3DChangedTracker> desiredHandPoseChangedTrackers = new SideDependentList<>();
   private final SideDependentList<FramePose3DChangedTracker> desiredFootPoseChangedTrackers = new SideDependentList<>();
   private FramePose3DChangedTracker desiredChestPoseChangedTracker;
   private FramePose3DChangedTracker desiredPelvisPoseChangedTracker;
   private final HumanoidKinematicsSolver wholeBodyIKSolver;
   private final SideDependentList<KinematicsToolboxRigidBodyCommand> handRigidBodyCommands = new SideDependentList<>();
   private final KinematicsToolboxRigidBodyCommand chestRigidBodyCommand = new KinematicsToolboxRigidBodyCommand();
   private final KinematicsToolboxRigidBodyCommand pelvisRigidBodyCommand = new KinematicsToolboxRigidBodyCommand();
   private final SideDependentList<KinematicsToolboxRigidBodyCommand> feetRigidBodyCommands = new SideDependentList<>(KinematicsToolboxRigidBodyCommand::new);
   private final WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private volatile boolean isSolutionGood = true;

   public RDXWholeBodyIKManager(DRCRobotModel robotModel,
                                RDXTeleoperationParameters teleoperationParameters,
                                RDXDesiredRobot desiredRobot,
                                ROS2ControllerHelper ros2Helper,
                                ROS2SyncedRobotModel syncedRobot,
                                ControllerStatusTracker controllerStatusTracker)
   {
      this.robotModel = robotModel;
      this.teleoperationParameters = teleoperationParameters;
      this.desiredRobot = desiredRobot;
      this.ros2Helper = ros2Helper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      wholeBodyIKSolver = new HumanoidKinematicsSolver(robotModel, yoGraphicsListRegistry, new YoRegistry(getClass().getSimpleName()));

      for (RobotSide side : RobotSide.values)
      {
         if (robotModel.getJointMap().getHandName(side) != null) // Handle one handed configurations
         {
            KinematicsToolboxRigidBodyCommand rigidBodyCommand = new KinematicsToolboxRigidBodyCommand();
            rigidBodyCommand.setEndEffector(wholeBodyIKSolver.getDesiredFullRobotModel().getHand(side));
            rigidBodyCommand.getControlFramePose().setToZero(wholeBodyIKSolver.getDesiredFullRobotModel().getHandControlFrame(side));
            rigidBodyCommand.getControlFramePose().changeFrame(wholeBodyIKSolver.getDesiredFullRobotModel().getHand(side).getBodyFixedFrame());
            // TODO: Use default values from somewhere else
            rigidBodyCommand.getWeightMatrix().setLinearWeights(20.0, 20.0, 20.0);
            rigidBodyCommand.getWeightMatrix().setAngularWeights(1.0, 1.0, 1.0);
            handRigidBodyCommands.put(side, rigidBodyCommand);
         }

         KinematicsToolboxRigidBodyCommand rigidBodyCommand = feetRigidBodyCommands.get(side);
         rigidBodyCommand.setEndEffector(wholeBodyIKSolver.getDesiredFullRobotModel().getFoot(side));
         rigidBodyCommand.getControlFramePose().setToZero(wholeBodyIKSolver.getDesiredFullRobotModel().getSoleFrame(side));
         rigidBodyCommand.getControlFramePose().changeFrame(wholeBodyIKSolver.getDesiredFullRobotModel().getFoot(side).getBodyFixedFrame());
         rigidBodyCommand.getWeightMatrix().setLinearWeights(20.0, 20.0, 20.0);
         rigidBodyCommand.getWeightMatrix().setAngularWeights(1.0, 1.0, 1.0);
      }
      chestRigidBodyCommand.setEndEffector(wholeBodyIKSolver.getDesiredFullRobotModel().getChest());
      chestRigidBodyCommand.getControlFramePose().setToZero(wholeBodyIKSolver.getDesiredFullRobotModel().getChest().getParentJoint().getFrameAfterJoint());
      chestRigidBodyCommand.getControlFramePose().changeFrame(wholeBodyIKSolver.getDesiredFullRobotModel().getChest().getBodyFixedFrame());
      chestRigidBodyCommand.getWeightMatrix().setLinearWeights(20.0, 20.0, 20.0);
      chestRigidBodyCommand.getWeightMatrix().setAngularWeights(1.0, 1.0, 1.0);
      pelvisRigidBodyCommand.setEndEffector(wholeBodyIKSolver.getDesiredFullRobotModel().getPelvis());
      pelvisRigidBodyCommand.getControlFramePose().setToZero(wholeBodyIKSolver.getDesiredFullRobotModel().getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisRigidBodyCommand.getControlFramePose().changeFrame(wholeBodyIKSolver.getDesiredFullRobotModel().getPelvis().getBodyFixedFrame());
      pelvisRigidBodyCommand.getWeightMatrix().setLinearWeights(20.0, 20.0, 20.0);
      pelvisRigidBodyCommand.getWeightMatrix().setAngularWeights(1.0, 1.0, 1.0);
   }

   public void setInteractables(SideDependentList<RDXInteractableHand> interactableHands,
                                SideDependentList<RDXInteractableFoot> interactableFeet,
                                RDXInteractableRobotLink interactableChest,
                                RDXInteractableRobotLink interactablePelvis)
   {
      this.interactableHands = interactableHands;
      this.interactableFeet = interactableFeet;
      this.interactableChest = interactableChest;
      this.interactablePelvis = interactablePelvis;

      for (RobotSide side : interactableHands.sides())
         desiredHandPoseChangedTrackers.put(side, new FramePose3DChangedTracker(interactableHands.get(side).getPose()));
      for (RobotSide side : RobotSide.values)
         desiredFootPoseChangedTrackers.put(side, new FramePose3DChangedTracker(interactableFeet.get(side).getPose()));
      desiredChestPoseChangedTracker = new FramePose3DChangedTracker(interactableChest.getPose());
      desiredPelvisPoseChangedTracker = new FramePose3DChangedTracker(interactablePelvis.getPose());
   }

   public void update()
   {
      for (RobotSide side : interactableHands.sides())
         desiredRobot.setArmShowing(side, true);
      for (RobotSide side : interactableFeet.sides())
         desiredRobot.setLegShowing(side, true);
      desiredRobot.setChestShowing(true);
      desiredRobot.setPelvisShowing(true);

      if (readyToSolve)
      {
         boolean desiredsChanged = false;
         for (RobotSide side : interactableHands.sides())
            if (!interactableHands.get(side).isDeleted())
               desiredsChanged |= desiredHandPoseChangedTrackers.get(side).hasChanged();
         for (RobotSide side : RobotSide.values)
            if (!interactableFeet.get(side).isDeleted())
               desiredsChanged |= desiredFootPoseChangedTrackers.get(side).hasChanged();
         if (!interactableChest.isDeleted())
            desiredsChanged |= desiredChestPoseChangedTracker.hasChanged();
         if (!interactablePelvis.isDeleted())
            desiredsChanged |= desiredPelvisPoseChangedTracker.hasChanged();

         if (desiredsChanged)
         {
            readyToSolve = false;

            wholeBodyIKSolver.setInitialConfiguration(syncedRobot.getLatestRobotConfigurationData());
            wholeBodyIKSolver.setCapturabilityBasedStatus(controllerStatusTracker.getLatestCapturabilityBasedStatus());
            wholeBodyIKSolver.initialize();

            for (RobotSide side : handRigidBodyCommands.sides())
            {
               if (!interactableHands.get(side).isDeleted())
               {
                  KinematicsToolboxRigidBodyCommand rigidBodyCommand = handRigidBodyCommands.get(side);
                  rigidBodyCommand.getDesiredPose().setFromReferenceFrame(interactableHands.get(side).getControlReferenceFrame());
                  wholeBodyIKSolver.submit(rigidBodyCommand);
               }
            }
            for (RobotSide side : RobotSide.values)
            {
               if (!interactableFeet.get(side).isDeleted())
               {
                  KinematicsToolboxRigidBodyCommand rigidBodyCommand = feetRigidBodyCommands.get(side);
                  rigidBodyCommand.getDesiredPose().setFromReferenceFrame(interactableFeet.get(side).getControlReferenceFrame());
                  wholeBodyIKSolver.submit(rigidBodyCommand);
               }
            }
            if (!interactableChest.isDeleted())
            {
               chestRigidBodyCommand.getDesiredPose().setFromReferenceFrame(interactableChest.getControlReferenceFrame());
               wholeBodyIKSolver.submit(chestRigidBodyCommand);
            }
            if (!interactablePelvis.isDeleted())
            {
               pelvisRigidBodyCommand.getDesiredPose().setFromReferenceFrame(interactablePelvis.getControlReferenceFrame());
               wholeBodyIKSolver.submit(pelvisRigidBodyCommand);
            }

            MissingThreadTools.startAThread(getClass().getSimpleName(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
            {
               try
               {
                  isSolutionGood = wholeBodyIKSolver.solve();
               }
               finally
               {
                  readyToCopySolution = true;
               }
            });
         }
      }

      if (readyToCopySolution)
      {
         readyToCopySolution = false;

         desiredRobot.getDesiredFullRobotModel()
                     .getRootJoint()
                     .setJointConfiguration(wholeBodyIKSolver.getSolution().getDesiredRootOrientation(),
                                            wholeBodyIKSolver.getSolution().getDesiredRootPosition());
         MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(wholeBodyIKSolver.getDesiredOneDoFJoints(),
                                                                   desiredRobot.getDesiredFullRobotModel().getOneDoFJoints());
         desiredRobot.setWholeBodyColor(RDXIKSolverColors.getColor(isSolutionGood));
         desiredRobot.getDesiredFullRobotModel().updateFrames();

         readyToSolve = true;
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.checkbox(labels.get("Enable Whole Body IK"), enabled);

      if (enabled.get() && desiredRobot.isActive() && ImGui.isKeyReleased(ImGuiTools.getSpaceKey()))
      {
         wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().clear();
         wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().clear();
         wholeBodyTrajectoryMessage.getLeftLegTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().clear();
         wholeBodyTrajectoryMessage.getRightLegTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().clear();
         wholeBodyTrajectoryMessage.getSpineTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().clear();
         wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().clear();

         populateArmTrajectoryMessage(wholeBodyTrajectoryMessage.getLeftArmTrajectoryMessage(), RobotSide.LEFT);
         populateArmTrajectoryMessage(wholeBodyTrajectoryMessage.getRightArmTrajectoryMessage(), RobotSide.RIGHT);
         if (!interactableFeet.get(RobotSide.LEFT).isDeleted())
            populateLegTrajectoryMessage(wholeBodyTrajectoryMessage.getLeftLegTrajectoryMessage(), RobotSide.LEFT);
         if (!interactableFeet.get(RobotSide.RIGHT).isDeleted())
            populateLegTrajectoryMessage(wholeBodyTrajectoryMessage.getRightLegTrajectoryMessage(), RobotSide.RIGHT);

         SpineTrajectoryMessage spineTrajectoryMessage = wholeBodyTrajectoryMessage.getSpineTrajectoryMessage();
         spineTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = spineTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
         for (SpineJointName spineJointName : robotModel.getJointMap().getSpineJointNames())
         {
            OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.add();
            oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
            oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

            TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
            trajectoryPoint1DMessage.setTime(teleoperationParameters.getTrajectoryTime());
            trajectoryPoint1DMessage.setPosition(desiredRobot.getDesiredFullRobotModel().getSpineJoint(spineJointName).getQ());
            trajectoryPoint1DMessage.setVelocity(0.0);
         }

         long trajectoryReferenceFrameID = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         FramePose3D desiredPelvisPose = new FramePose3D(desiredRobot.getDesiredFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());

         PelvisTrajectoryMessage pelvisTrajectoryMessage = wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage();
         SE3TrajectoryMessage se3TrajectoryMessage = pelvisTrajectoryMessage.getSe3Trajectory();
         se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         // Select all axes and use default weights
         SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
         se3TrajectoryPointMessage.setTime(teleoperationParameters.getTrajectoryTime());
         se3TrajectoryPointMessage.getPosition().set(desiredPelvisPose.getPosition());
         se3TrajectoryPointMessage.getOrientation().set(desiredPelvisPose.getOrientation());
         se3TrajectoryPointMessage.getLinearVelocity().setToZero();
         se3TrajectoryPointMessage.getAngularVelocity().setToZero();
         se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameID);

         LogTools.info("Commanding whole body trajectory...");
         ros2Helper.publishToController(wholeBodyTrajectoryMessage);
      }
   }

   private void populateArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage, RobotSide side)
   {
      armTrajectoryMessage.setRobotSide(side.toByte());
      armTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = armTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      for (ArmJointName armJointName : robotModel.getJointMap().getArmJointNames(side))
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.add();
         oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
         oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(teleoperationParameters.getTrajectoryTime());
         trajectoryPoint1DMessage.setPosition(desiredRobot.getDesiredFullRobotModel().getArmJoint(side, armJointName).getQ());
         trajectoryPoint1DMessage.setVelocity(0.0);
      }
   }

   private void populateLegTrajectoryMessage(LegTrajectoryMessage legTrajectoryMessage, RobotSide side)
   {
      legTrajectoryMessage.setRobotSide(side.toByte());
      legTrajectoryMessage.getJointspaceTrajectory().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      Object<OneDoFJointTrajectoryMessage> jointTrajectoryMessages = legTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages();
      for (LegJointName legJointName : robotModel.getJointMap().getLegJointNames())
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointTrajectoryMessages.add();
         oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
         oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(teleoperationParameters.getTrajectoryTime());
         trajectoryPoint1DMessage.setPosition(desiredRobot.getDesiredFullRobotModel().getLegJoint(side, legJointName).getQ());
         trajectoryPoint1DMessage.setVelocity(0.0);
      }
   }

   public boolean getEnabled()
   {
      return enabled.get();
   }
}
