package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.BodyPartPoseStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class HandPoseAction extends HandPoseActionData implements BehaviorAction
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private int actionIndex;
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final HandWrenchCalculator handWrenchCalculator;
   private final HandPoseJointAnglesStatusMessage handPoseJointAnglesStatus = new HandPoseJointAnglesStatusMessage();
   private final IHMCROS2Input<BodyPartPoseStatusMessage> chestOrientationStatusSubscription;
   private final IHMCROS2Input<BodyPartPoseStatusMessage> pelvisPositionStatusSubscription;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public HandPoseAction(ROS2ControllerHelper ros2ControllerHelper,
                         ReferenceFrameLibrary referenceFrameLibrary,
                         DRCRobotModel robotModel,
                         ROS2SyncedRobotModel syncedRobot,
                         HandWrenchCalculator handWrenchCalculator)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculator = handWrenchCalculator;
      setReferenceFrameLibrary(referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }

      chestOrientationStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.CHEST_ORIENTATION_STATUS);
      pelvisPositionStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.PELVIS_POSITION_STATUS);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrencyWithPreviousIndex, int indexShiftConcurrentAction)
   {
      update();

      this.actionIndex = actionIndex;

   // if this action has to be executed with the previous or next one, it means it belongs to a group of concurrent actions
      if (concurrencyWithPreviousIndex || getExecuteWithNextAction())
      {
         // if chest is in the same group of concurrent actions, then update the IK according to that chest pose
         if (chestOrientationStatusSubscription.getMessageNotification().poll())
         {
            BodyPartPoseStatusMessage chestPoseStatusMessage = chestOrientationStatusSubscription.getLatest();
            ModifiableReferenceFrame chestReferenceFrame = new ModifiableReferenceFrame(getReferenceFrameLibrary()
                                                                                              .findFrameByName(chestPoseStatusMessage.getParentFrame()
                                                                                                                                     .getString(0)).get());
            chestReferenceFrame.update(transformToParent -> MessageTools.toEuclid(chestPoseStatusMessage.getTransformToParent(), transformToParent));

            ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
            armIKSolver.copyActualToWork();
            armIKSolver.setChestExternally(chestReferenceFrame.getReferenceFrame());
            computeAndPublishIKSolution(armIKSolver);
         }
         // compute the IK with the synced chest
         else
         {
            ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
            armIKSolver.copyActualToWork();
            computeAndPublishIKSolution(armIKSolver);
         }
//         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
//         armIKSolver.copyActualToWork();
//         armIKSolver.setChestExternally(getChestFrameAtTheEndOfAction(syncedRobot.getFullRobotModel().getChest(), getReferenceFrameLibrary(),
//                                                                      chestOrientationStatusSubscription, pelvisPositionStatusSubscription));
//         computeAndPublishIKSolution(armIKSolver);
      }
      else if (actionIndex == nextExecutionIndex)
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
         armIKSolver.setChestExternally(null);
         armIKSolver.copyActualToWork();
         computeAndPublishIKSolution(armIKSolver);

      }
   }

   public static ReferenceFrame getChestFrameAtTheEndOfAction(RigidBodyBasics syncedChest,
                                                              ReferenceFrameLibrary frameLibrary,
                                                              IHMCROS2Input<BodyPartPoseStatusMessage> chestOrientationStatusSubscription,
                                                              IHMCROS2Input<BodyPartPoseStatusMessage> pelvisPositionStatusSubscription)
   {
      ModifiableReferenceFrame chestInteractableReferenceFrame;
      if (chestOrientationStatusSubscription.getMessageNotification().poll())
      {
         LogTools.info("HAND COMPUTED WITH CHEST");
         BodyPartPoseStatusMessage chestPoseStatusMessage = chestOrientationStatusSubscription.getLatest();
         chestInteractableReferenceFrame = new ModifiableReferenceFrame(frameLibrary.findFrameByName(chestPoseStatusMessage.getParentFrame()
                                                                                                                                  .getString(0)).get());
         chestInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(chestPoseStatusMessage.getTransformToParent(), transformToParent));
      }
      else
         chestInteractableReferenceFrame = null;

      ModifiableReferenceFrame pelvisInteractableReferenceFrame;
      if (pelvisPositionStatusSubscription.getMessageNotification().poll())
      {
         LogTools.info("HAND COMPUTED WITH PELVIS");
         BodyPartPoseStatusMessage pelvisPoseStatusMessage = pelvisPositionStatusSubscription.getLatest();
         pelvisInteractableReferenceFrame = new ModifiableReferenceFrame(frameLibrary.findFrameByName(pelvisPoseStatusMessage.getParentFrame()
                                                                                                                                    .getString(0)).get());
         pelvisInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(pelvisPoseStatusMessage.getTransformToParent(), transformToParent));
      }
      else
         pelvisInteractableReferenceFrame = null;

      ReferenceFrame chestFrame = null;
      if (pelvisInteractableReferenceFrame != null && chestInteractableReferenceFrame != null)
      {
         chestInteractableReferenceFrame.update(transformToParent -> updateChestPoseFromPelvisPosition(pelvisInteractableReferenceFrame,
                                                                                                       chestInteractableReferenceFrame.getReferenceFrame(),
                                                                                           transformToParent));
         chestFrame = chestInteractableReferenceFrame.getReferenceFrame();
      }
      else if (pelvisInteractableReferenceFrame != null)
      {
         chestFrame = syncedChest.getParentJoint().getFrameAfterJoint();
         chestFrame.getTransformToWorldFrame()
                   .getTranslation()
                   .set(pelvisInteractableReferenceFrame.getReferenceFrame().getTransformToWorldFrame().getTranslation());
         chestFrame.update();
      }
      else if (chestInteractableReferenceFrame != null)
         chestFrame = chestInteractableReferenceFrame.getReferenceFrame();

      return chestFrame;
   }

   private static void updateChestPoseFromPelvisPosition(ModifiableReferenceFrame pelvisFrame,
                                                         ReferenceFrame parentFrame,
                                                         RigidBodyTransform chestTransformToParent)
   {
      pelvisFrame.changeParentFrame(parentFrame);
      RigidBodyTransform pelvisTranformToParent = pelvisFrame.getTransformToParent();
      chestTransformToParent.getTranslation().set(pelvisTranformToParent.getTranslation());
   }

   private void computeAndPublishIKSolution(ArmIKSolver armIKSolver)
   {
      armIKSolver.update(getPalmFrame());
      armIKSolver.solve();

      // Send the solution back to the UI so the user knows what's gonna happen with the arm.
      handPoseJointAnglesStatus.getActionInformation().setActionIndex(actionIndex);
      handPoseJointAnglesStatus.setRobotSide(getSide().toByte());
      handPoseJointAnglesStatus.setSolutionQuality(armIKSolver.getQuality());
      for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
      {
         handPoseJointAnglesStatus.getJointAngles()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
      }
      if (getSide() == RobotSide.LEFT)
         ros2ControllerHelper.publish(BehaviorActionSequence.LEFT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
      else
         ros2ControllerHelper.publish(BehaviorActionSequence.RIGHT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
   }

   @Override
   public void triggerActionExecution()
   {
      if (getJointSpaceControl())
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());

         OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
         double[] jointAngles = new double[solutionOneDoFJoints.length];
         for (int i = 0; i < jointAngles.length; i++)
         {
            jointAngles[i] = solutionOneDoFJoints[i].getQ();
         }

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getSide(), getTrajectoryDuration(), jointAngles);
         armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
      }
      else
      {
         FramePose3D frameHand = new FramePose3D(getPalmFrame());
         frameHand.changeFrame(ReferenceFrame.getWorldFrame());
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(getSide(),
                                                                                                        getTrajectoryDuration(),
                                                                                                        frameHand.getPosition(),
                                                                                                        frameHand.getOrientation(),
                                                                                                        ReferenceFrame.getWorldFrame());
         handTrajectoryMessage.setForceExecution(true);
         ros2ControllerHelper.publishToController(handTrajectoryMessage);
      }

      executionTimer.reset();
      desiredHandControlPose.setFromReferenceFrame(getPalmFrame());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getSide()));
      startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
      startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredHandControlPose.setFromReferenceFrame(getPalmFrame());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getSide()));

      boolean wasExecuting = isExecuting;
      // Left hand broke on Nadia and not in the robot model?
      isExecuting = !completionCalculator.isComplete(desiredHandControlPose,
                                                     syncedHandControlPose,
                                                     POSITION_TOLERANCE, ORIENTATION_TOLERANCE,
                                                     getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.TRANSLATION,
                                                     BehaviorActionCompletionComponent.ORIENTATION);

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      executionStatusMessage.setHandWrenchMagnitudeLinear(handWrenchCalculator.getLinearWrenchMagnitude(getSide(), true));
      if (!isExecuting && wasExecuting && !getJointSpaceControl() && !getHoldPoseInWorldLater())
      {
         disengageHoldPoseInWorld();
      }
   }

   private void disengageHoldPoseInWorld()
   {
      FramePose3D frameHand = new FramePose3D(getPalmFrame());
      frameHand.changeFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      HandTrajectoryMessage message = HumanoidMessageTools.createHandTrajectoryMessage(getSide(),
                                                                                       getTrajectoryDuration(),
                                                                                       frameHand.getPosition(),
                                                                                       frameHand.getOrientation(),
                                                                                       syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      message.setForceExecution(true);
      ros2ControllerHelper.publishToController(message);
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }
}
