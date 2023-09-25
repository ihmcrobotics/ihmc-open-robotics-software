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
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
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
   private FramePose3D previousPelvisPose;

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

      // while the first action is being executed and the corresponding IK solution is computed, also do that for the following concurrent actions
      if (concurrencyWithPreviousIndex && actionIndex == (nextExecutionIndex + indexShiftConcurrentAction) ||
          (getExecuteWithNextAction() && actionIndex == nextExecutionIndex))
      {
                  ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
                  armIKSolver.copyActualToWork();
                  armIKSolver.setChestExternally(getHandReferenceFrameAtTheEndOfAction(syncedRobot.getFullRobotModel().getChest(), getReferenceFrameLibrary(),
                                                                                       chestOrientationStatusSubscription, pelvisPositionStatusSubscription,
                                                                                       previousPelvisPose));
                  computeAndPublishIKSolution(armIKSolver);
      }
      else if (actionIndex == nextExecutionIndex)
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
         armIKSolver.setChestExternally(null);
         armIKSolver.copyActualToWork();
         computeAndPublishIKSolution(armIKSolver);

      }
   }

   public static ReferenceFrame getHandReferenceFrameAtTheEndOfAction(RigidBodyBasics syncedChest,
                                                                      ReferenceFrameLibrary frameLibrary,
                                                                      IHMCROS2Input<BodyPartPoseStatusMessage> chestOrientationStatusSubscription,
                                                                      IHMCROS2Input<BodyPartPoseStatusMessage> pelvisPositionStatusSubscription,
                                                                      FramePose3D previousPelvisPose)
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

//      ModifiableReferenceFrame pelvisInteractableReferenceFrame;
//      if (pelvisPositionStatusSubscription.getMessageNotification().poll())
//      {
//         LogTools.info("HAND COMPUTED WITH PELVIS");
//         BodyPartPoseStatusMessage pelvisPoseStatusMessage = pelvisPositionStatusSubscription.getLatest();
//         pelvisInteractableReferenceFrame = new ModifiableReferenceFrame(frameLibrary.findFrameByName(pelvisPoseStatusMessage.getParentFrame()
//                                                                                                                                    .getString(0)).get());
//         pelvisInteractableReferenceFrame.update(transformToParent -> MessageTools.toEuclid(pelvisPoseStatusMessage.getTransformToParent(), transformToParent));
//         previousPelvisPose = new FramePose3D(pelvisInteractableReferenceFrame.getReferenceFrame(),
//                                              pelvisInteractableReferenceFrame.getReferenceFrame().getTransformToParent());
//      }
//      else
//      {
//         pelvisInteractableReferenceFrame = null;
//         previousPelvisPose = null;
//      }

      ReferenceFrame chestFrame = null;
//      if (pelvisInteractableReferenceFrame != null && chestInteractableReferenceFrame != null)
      ////      {
      ////         FramePose3D finalPreviousPelvisPose = previousPelvisPose;
      ////         chestInteractableReferenceFrame.changeParentFrameWithoutMoving(ReferenceFrame.getWorldFrame());
      ////         chestInteractableReferenceFrame.update(transformToParent -> updateChestPoseFromPelvisPosition(pelvisInteractableReferenceFrame,
      ////                                                                                                       finalPreviousPelvisPose,
      ////                                                                                                       chestInteractableReferenceFrame.getReferenceFrame(),
      ////                                                                                                       transformToParent));
      ////         chestFrame = chestInteractableReferenceFrame.getReferenceFrame();
      ////      }
      ////      else if (pelvisInteractableReferenceFrame != null)
      ////      {
      ////         chestFrame = syncedChest.getParentJoint().getFrameAfterJoint();
      ////         previousPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      ////         chestFrame.getTransformToWorldFrame()
      ////                   .getTranslation()
      ////                   .setZ(pelvisInteractableReferenceFrame.getReferenceFrame().getTransformToWorldFrame().getTranslationZ() - previousPelvisPose.getTranslationZ());
      ////         chestFrame.update();
      ////      }
      ////      else
       if (chestInteractableReferenceFrame != null)
         chestFrame = chestInteractableReferenceFrame.getReferenceFrame();

      return chestFrame;
   }

   private static void updateChestPoseFromPelvisPosition(ModifiableReferenceFrame pelvisInteractableFrame,
                                                         FramePose3D previousPelvisPose,
                                                         ReferenceFrame parentFrame,
                                                         RigidBodyTransform chestTransformToParent)
   {
      pelvisInteractableFrame.changeParentFrameWithoutMoving(ReferenceFrame.getWorldFrame());
      previousPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      RigidBodyTransform newChestTransformToParent = pelvisInteractableFrame.getTransformToParent();
      // add chest origin and remove abs
      newChestTransformToParent.getTranslation().setZ(Math.abs(pelvisInteractableFrame.getTransformToParent().getTranslationZ() - previousPelvisPose.getTranslationZ()));

      chestTransformToParent.getTranslation().set(newChestTransformToParent.getTranslation());
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
