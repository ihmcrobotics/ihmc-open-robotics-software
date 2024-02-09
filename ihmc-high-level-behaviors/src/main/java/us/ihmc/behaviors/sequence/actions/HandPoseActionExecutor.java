package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();
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

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

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
         JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildJointspaceTrajectoryMessage();

         ReferenceFrame taskspaceTrajectoryFrame = ReferenceFrame.getWorldFrame();
         long trajectoryReferenceFrameID = MessageTools.toFrameId(taskspaceTrajectoryFrame);
         FramePose3D desiredControlFramePose = new FramePose3D(state.getPalmFrame().getReferenceFrame());
         desiredControlFramePose.changeFrame(taskspaceTrajectoryFrame);

         SE3TrajectoryMessage se3TrajectoryMessage = new SE3TrajectoryMessage();
         se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         se3TrajectoryMessage.getLinearWeightMatrix().setXWeight(getDefinition().getLinearPositionWeight());
         se3TrajectoryMessage.getLinearWeightMatrix().setYWeight(getDefinition().getLinearPositionWeight());
         se3TrajectoryMessage.getLinearWeightMatrix().setZWeight(getDefinition().getLinearPositionWeight());
         se3TrajectoryMessage.getAngularWeightMatrix().setXWeight(getDefinition().getAngularPositionWeight());
         se3TrajectoryMessage.getAngularWeightMatrix().setYWeight(getDefinition().getAngularPositionWeight());
         se3TrajectoryMessage.getAngularWeightMatrix().setZWeight(getDefinition().getAngularPositionWeight());
         se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameID);
         SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
         se3TrajectoryPointMessage.setTime(getDefinition().getTrajectoryDuration());
         se3TrajectoryPointMessage.getPosition().set(desiredControlFramePose.getPosition());
         se3TrajectoryPointMessage.getOrientation().set(desiredControlFramePose.getOrientation());
         se3TrajectoryPointMessage.getLinearVelocity().setToZero();
         se3TrajectoryPointMessage.getAngularVelocity().setToZero();

         if (getDefinition().getJointspaceOnly())
         {
            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
            armTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
            armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
            armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
            LogTools.info("Publishing arm jointspace trajectory");
            ros2ControllerHelper.publishToController(armTrajectoryMessage);
         }
         else // Publishing taskspace only doesn't work well, so we use hybrid - @dcalvert
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridJointspaceTaskspaceTrajectoryMessage
                  = new HandHybridJointspaceTaskspaceTrajectoryMessage();
            handHybridJointspaceTaskspaceTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
            handHybridJointspaceTaskspaceTrajectoryMessage.getTaskspaceTrajectoryMessage().set(se3TrajectoryMessage);
            handHybridJointspaceTaskspaceTrajectoryMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
            LogTools.info("Publishing arm hybrid jointspace taskpace");
            ros2ControllerHelper.publishToController(handHybridJointspaceTaskspaceTrajectoryMessage);
         }

         trackingCalculator.reset();

         state.setNominalExecutionDuration(getDefinition().getTrajectoryDuration());
         
         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         state.getDesiredTrajectory().setSingleSegmentTrajectory(syncedHandControlPose, desiredHandControlPose, getDefinition().getTrajectoryDuration());
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());
      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         LogTools.error("Task execution timed out. Publishing stop all trajectories message.");
         ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
         return;
      }

      if (state.getPalmFrame().isChildOfWorld())
      {
         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));

         trackingCalculator.computePoseTrackingData(desiredHandControlPose, syncedHandControlPose);
         trackingCalculator.factorInR3Errors(POSITION_TOLERANCE);
         trackingCalculator.factoryInSO3Errors(ORIENTATION_TOLERANCE);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.getCurrentPose().getValue().set(syncedHandControlPose);
         state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
         state.getForce().getValue().set(syncedRobot.getHandWrenchCalculators().get(getDefinition().getSide()).getFilteredWrench().getLinearPart());
         state.getTorque().getValue().set(syncedRobot.getHandWrenchCalculators().get(getDefinition().getSide()).getFilteredWrench().getAngularPart());

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);

            if (!getDefinition().getJointspaceOnly() && !getDefinition().getHoldPoseInWorldLater())
            {
               disengageHoldPoseInWorld();
            }
         }
      }
   }

   private void disengageHoldPoseInWorld()
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildJointspaceTrajectoryMessage();

      ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
      armTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
      armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      LogTools.info("Publishing arm jointspace trajectory to disengage holding hand in taskspace");
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }

   private JointspaceTrajectoryMessage buildJointspaceTrajectoryMessage()
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      for (OneDoFJointBasics solutionOneDoFJoint : armIKSolvers.get(getDefinition().getSide()).getSolutionOneDoFJoints())
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         oneDoFJointTrajectoryMessage.setWeight(getDefinition().getJointspaceWeight());

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(getDefinition().getTrajectoryDuration());
         trajectoryPoint1DMessage.setPosition(solutionOneDoFJoint.getQ());
         trajectoryPoint1DMessage.setVelocity(0.0);
      }
      return jointspaceTrajectoryMessage;
   }
}
