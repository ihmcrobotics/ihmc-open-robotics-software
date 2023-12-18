package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.NonWallTimer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class HandPoseActionExecutor extends ActionNodeExecutor<HandPoseActionState, HandPoseActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final HandPoseActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final NonWallTimer executionTimer = new NonWallTimer();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();
   private final RigidBodyTransform chestToPelvisZeroAngles = new RigidBodyTransform();
   private final FramePose3D chestInPelvis = new FramePose3D();
   private final FramePose3D goalChestFrame = new FramePose3D();
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();

   public HandPoseActionExecutor(long id,
                                 CRDTInfo crdtInfo,
                                 WorkspaceResourceDirectory saveFileDirectory,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot)
   {
      super(new HandPoseActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }

      FramePose3D chestAfterJointToPelvis = new FramePose3D();
      chestAfterJointToPelvis.setToZero(syncedRobot.getReferenceFrames().getChestFrame());
      chestAfterJointToPelvis.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
      chestAfterJointToPelvis.get(chestToPelvisZeroAngles);
   }

   @Override
   public void update()
   {
      super.update();

      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(state.getPalmFrame().isChildOfWorld());

      if (state.getPalmFrame().isChildOfWorld() && state.getIsNextForExecution())
      {
         ChestOrientationActionExecutor concurrentChestOrientationAction = null;
         PelvisHeightPitchActionExecutor concurrentPelvisHeightPitchAction = null;

         if (getParent() instanceof ActionSequenceExecutor parentSequence)
         {
            if (state.getIsToBeExecutedConcurrently())
            {
               int concurrentSetIndex = parentSequence.getState().getExecutionNextIndex();

               while (concurrentSetIndex <= parentSequence.getLastIndexOfConcurrentSetToExecute())
               {
                  if (parentSequence.getExecutorChildren().get(concurrentSetIndex) instanceof ChestOrientationActionExecutor chestOrientationAction)
                  {
                     concurrentChestOrientationAction = chestOrientationAction;
                  }
                  if (parentSequence.getExecutorChildren().get(concurrentSetIndex) instanceof PelvisHeightPitchActionExecutor pelvisHeightPitchAction)
                  {
                     concurrentPelvisHeightPitchAction = pelvisHeightPitchAction;
                  }
                  ++concurrentSetIndex;
               }
            }

            for (ActionNodeExecutor<?, ?> currentlyExecutingAction : parentSequence.getCurrentlyExecutingActions())
            {
               if (currentlyExecutingAction instanceof ChestOrientationActionExecutor chestOrientationAction)
               {
                  concurrentChestOrientationAction = chestOrientationAction;
               }
               if (currentlyExecutingAction instanceof PelvisHeightPitchActionExecutor pelvisHeightPitchAction)
               {
                  concurrentPelvisHeightPitchAction = pelvisHeightPitchAction;
               }
            }
         }

         if (concurrentChestOrientationAction == null && concurrentPelvisHeightPitchAction == null)
         {
            state.getGoalChestToWorldTransform().getValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
         }
         else if (concurrentPelvisHeightPitchAction == null)
         {
            concurrentChestOrientationAction.getState().update(); // Ensure state's frames are initialized
            state.getGoalChestToWorldTransform().getValue()
                 .set(concurrentChestOrientationAction.getState().getChestFrame().getReferenceFrame().getTransformToRoot());
         }
         else if (concurrentChestOrientationAction == null)
         {
            // FIXME We are ignoring this case for now, just add a pelvis pose to get the desired result
            //   We need to switch to a proper whole body action node
            state.getGoalChestToWorldTransform().getValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
         }
         else // Combined case
         {
            concurrentChestOrientationAction.getState().update(); // Ensure state's frames are initialized
            concurrentPelvisHeightPitchAction.getState().update(); // Ensure state's frames are initialized

            ReferenceFrame chestActionFrame = concurrentChestOrientationAction.getState().getChestFrame().getReferenceFrame();
            ReferenceFrame pelvisActionFrame = concurrentPelvisHeightPitchAction.getState().getPelvisFrame().getReferenceFrame();

            chestInPelvis.setToZero(chestActionFrame);
            chestInPelvis.changeFrame(pelvisActionFrame);

            goalChestFrame.setToZero(pelvisActionFrame);
            goalChestFrame.getRotation().append(chestInPelvis.getRotation()); // Append chest rotation
            goalChestFrame.prependTranslation(chestToPelvisZeroAngles.getTranslation());
            goalChestFrame.changeFrame(ReferenceFrame.getWorldFrame());
            goalChestFrame.get(state.getGoalChestToWorldTransform().getValue());
         }
         state.getGoalChestFrame().update();

         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());
         armIKSolver.copySourceToWork();
         armIKSolver.update(state.getGoalChestFrame(), state.getPalmFrame().getReferenceFrame());
         armIKSolver.solve();

         // Send the solution back to the UI so the user knows what's gonna happen with the arm.
         state.setSolutionQuality(armIKSolver.getQuality());
         for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         {
            state.getJointAngles().getValue()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
         }
      }
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (state.getPalmFrame().isChildOfWorld())
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());

         OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
         double[] jointAngles = new double[solutionOneDoFJoints.length];
         for (int i = 0; i < jointAngles.length; i++)
         {
            jointAngles[i] = solutionOneDoFJoints[i].getQ();
         }

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                     getDefinition().getTrajectoryDuration(),
                                                                                                     jointAngles);
         armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
         if (getDefinition().getJointSpaceControl())
         {
            LogTools.info("Publishing arm jointspace trajectory");
            ros2ControllerHelper.publishToController(armTrajectoryMessage);
         }
         else
         {
            FramePose3D frameHand = new FramePose3D(state.getPalmFrame().getReferenceFrame());
            frameHand.changeFrame(ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(getDefinition().getSide(),
                                                                                                           getDefinition().getTrajectoryDuration(),
                                                                                                           frameHand.getPosition(),
                                                                                                           frameHand.getOrientation(),
                                                                                                           ReferenceFrame.getWorldFrame());
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setXWeight(1.0);
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setYWeight(1.0);
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setZWeight(1.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setXWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setYWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setZWeight(50.0);
            handTrajectoryMessage.setForceExecution(true);

            HandHybridJointspaceTaskspaceTrajectoryMessage hybridHandMessage = HumanoidMessageTools.createHandHybridJointspaceTaskspaceTrajectoryMessage(
                  getDefinition().getSide(),
                  handTrajectoryMessage.getSe3Trajectory(),
                  armTrajectoryMessage.getJointspaceTrajectory());
            LogTools.info("Publishing arm hybrid jointspace taskpace");
            ros2ControllerHelper.publishToController(hybridHandMessage);
         }

         executionTimer.reset();

         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
         startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (executionTimer.isExpired(getState().getNominalExecutionDuration() * 1.5))
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
         return;
      }

      if (state.getPalmFrame().isChildOfWorld())
      {
         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));

         boolean wasExecuting = state.getIsExecuting();
         // Left hand broke on Nadia and not in the robot model?
         state.setIsExecuting(!completionCalculator.isComplete(desiredHandControlPose,
                                                               syncedHandControlPose,
                                                               POSITION_TOLERANCE,
                                                               ORIENTATION_TOLERANCE,
                                                               getDefinition().getTrajectoryDuration(),
                                                               executionTimer,
                                                               getState(),
                                                               BehaviorActionCompletionComponent.TRANSLATION,
                                                               BehaviorActionCompletionComponent.ORIENTATION));

         state.setNominalExecutionDuration(getDefinition().getTrajectoryDuration());
         state.setElapsedExecutionTime(executionTimer.getElapsedTime());
         state.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         state.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         state.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         state.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
         state.setHandWrenchMagnitudeLinear(syncedRobot.getHandWrenchCalculators().get(getDefinition().getSide()).getLinearWrenchMagnitude(true));
         if (!state.getIsExecuting() && wasExecuting && !getDefinition().getJointSpaceControl() && !getDefinition().getHoldPoseInWorldLater())
         {
            disengageHoldPoseInWorld();
         }
      }
   }

   private void disengageHoldPoseInWorld()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                  getDefinition().getTrajectoryDuration(),
                                                                                                  jointAngles);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }
}
