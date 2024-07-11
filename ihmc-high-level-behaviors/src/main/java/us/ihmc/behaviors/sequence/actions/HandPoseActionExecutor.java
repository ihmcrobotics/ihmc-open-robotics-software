package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.List;

public class HandPoseActionExecutor extends ActionNodeExecutor<HandPoseActionState, HandPoseActionDefinition>
{
   private final HandPoseActionState state;
   private final HandPoseActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final TaskspaceTrajectoryTrackingErrorCalculator trackingCalculator = new TaskspaceTrajectoryTrackingErrorCalculator();
   private final RigidBodyTransform chestToPelvisZeroAngles = new RigidBodyTransform();
   private final FramePose3D chestInPelvis = new FramePose3D();
   private final FramePose3D goalChestFrame = new FramePose3D();

   public HandPoseActionExecutor(long id,
                                 CRDTInfo crdtInfo,
                                 WorkspaceResourceDirectory saveFileDirectory,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot)
   {
      super(new HandPoseActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary, robotModel));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel.getJointMap(), syncedRobot.getFullRobotModel()));
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

      if (state.getIsNextForExecution())
      {
         ChestOrientationActionState concurrentChestOrientationAction = null;
         PelvisHeightOrientationActionState concurrentPelvisHeightPitchAction = null;

         BehaviorTreeRootNodeExecutor actionSequenceExecutor = BehaviorTreeTools.findRootNode(this);
         if (actionSequenceExecutor != null)
         {
            if (state.getIsToBeExecutedConcurrently())
            {
               List<ActionNodeState<?>> actionChildren = actionSequenceExecutor.getState().getActionChildren();

               for (int i = state.getActionIndex() - 1; i >= 0 && actionChildren.get(i + 1).getIsToBeExecutedConcurrently(); i--)
               {
                  if (actionChildren.get(i) instanceof ChestOrientationActionState chestOrientationAction)
                  {
                     concurrentChestOrientationAction = chestOrientationAction;
                  }
                  if (actionChildren.get(i) instanceof PelvisHeightOrientationActionState pelvisHeightPitchAction)
                  {
                     concurrentPelvisHeightPitchAction = pelvisHeightPitchAction;
                  }
               }
            }

            for (ActionNodeExecutor<?, ?> currentlyExecutingAction : actionSequenceExecutor.getCurrentlyExecutingActions())
            {
               if (currentlyExecutingAction.getState() instanceof ChestOrientationActionState chestOrientationAction)
               {
                  concurrentChestOrientationAction = chestOrientationAction;
               }
               if (currentlyExecutingAction.getState() instanceof PelvisHeightOrientationActionState pelvisHeightPitchAction)
               {
                  concurrentPelvisHeightPitchAction = pelvisHeightPitchAction;
               }
            }
         }

         if (concurrentChestOrientationAction == null && concurrentPelvisHeightPitchAction == null)
         {
            state.getGoalChestToWorldTransform().accessValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
         }
         else if (concurrentPelvisHeightPitchAction == null)
         {
            concurrentChestOrientationAction.update(); // Ensure state's frames are initialized
            state.getGoalChestToWorldTransform().accessValue().set(concurrentChestOrientationAction.getChestFrame().getReferenceFrame().getTransformToRoot());
         }
         else if (concurrentChestOrientationAction == null)
         {
            // FIXME We are ignoring this case for now, just add a pelvis pose to get the desired result
            //   We need to switch to a proper whole body action node
            state.getGoalChestToWorldTransform().accessValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
         }
         else // Combined case
         {
            concurrentChestOrientationAction.update(); // Ensure state's frames are initialized
            concurrentPelvisHeightPitchAction.update(); // Ensure state's frames are initialized

            ReferenceFrame chestActionFrame = concurrentChestOrientationAction.getChestFrame().getReferenceFrame();
            ReferenceFrame pelvisActionFrame = concurrentPelvisHeightPitchAction.getPelvisFrame().getReferenceFrame();

            chestInPelvis.setToZero(chestActionFrame);
            chestInPelvis.changeFrame(pelvisActionFrame);

            goalChestFrame.setToZero(pelvisActionFrame);
            goalChestFrame.getRotation().append(chestInPelvis.getRotation()); // Append chest rotation
            goalChestFrame.prependTranslation(chestToPelvisZeroAngles.getTranslation());
            goalChestFrame.changeFrame(ReferenceFrame.getWorldFrame());
            goalChestFrame.get(state.getGoalChestToWorldTransform().accessValue());
         }
         state.getGoalChestFrame().update();
      }

      if (definition.getUsePredefinedJointAngles())
      {
         state.setCanExecute(true);

         state.setSolutionQuality(0.0);
         for (int i = 0; i < definition.getJointAngles().getLength(); i++)
         {
            state.getJointAngles().accessValue()[i] = definition.getJointAngles().getValueReadOnly(i);
         }
      }
      else
      {
         state.setCanExecute(state.getPalmFrame().isChildOfWorld());

         if (state.getPalmFrame().isChildOfWorld() && state.getIsNextForExecution())
         {
            ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
            armIKSolver.copySourceToWork();
            armIKSolver.update(state.getGoalChestFrame(), state.getPalmFrame().getReferenceFrame());
            armIKSolver.solve();

            // Send the solution back to the UI so the user knows what's gonna happen with the arm.
            state.setSolutionQuality(armIKSolver.getQuality());
            for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
            {
               state.getJointAngles().accessValue()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
            }
         }
      }
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      state.setNominalExecutionDuration(definition.getTrajectoryDuration());
      trackingCalculator.reset();

      if (definition.getUsePredefinedJointAngles())
      {
         JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildJointspaceTrajectoryMessage();
         publishJointspaceCommand(jointspaceTrajectoryMessage);
      }
      else if (state.getPalmFrame().isChildOfWorld())
      {
         JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildJointspaceTrajectoryMessage();

         if (definition.getJointspaceOnly())
         {
            publishJointspaceCommand(jointspaceTrajectoryMessage);
         }
         else // Publishing taskspace only doesn't work well, so we use hybrid - @dcalvert
         {
            ReferenceFrame taskspaceTrajectoryFrame = ReferenceFrame.getWorldFrame();
            long trajectoryReferenceFrameID = MessageTools.toFrameId(taskspaceTrajectoryFrame);
            FramePose3D desiredControlFramePose = new FramePose3D(state.getPalmFrame().getReferenceFrame());
            desiredControlFramePose.changeFrame(taskspaceTrajectoryFrame);

            SE3TrajectoryMessage se3TrajectoryMessage = new SE3TrajectoryMessage();
            se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
            se3TrajectoryMessage.getLinearWeightMatrix().setXWeight(definition.getLinearPositionWeight());
            se3TrajectoryMessage.getLinearWeightMatrix().setYWeight(definition.getLinearPositionWeight());
            se3TrajectoryMessage.getLinearWeightMatrix().setZWeight(definition.getLinearPositionWeight());
            se3TrajectoryMessage.getAngularWeightMatrix().setXWeight(definition.getAngularPositionWeight());
            se3TrajectoryMessage.getAngularWeightMatrix().setYWeight(definition.getAngularPositionWeight());
            se3TrajectoryMessage.getAngularWeightMatrix().setZWeight(definition.getAngularPositionWeight());
            se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(trajectoryReferenceFrameID);
            SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
            se3TrajectoryPointMessage.setTime(definition.getTrajectoryDuration());
            se3TrajectoryPointMessage.getPosition().set(desiredControlFramePose.getPosition());
            se3TrajectoryPointMessage.getOrientation().set(desiredControlFramePose.getOrientation());
            se3TrajectoryPointMessage.getLinearVelocity().setToZero();
            se3TrajectoryPointMessage.getAngularVelocity().setToZero();

            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridJointspaceTaskspaceTrajectoryMessage
                  = new HandHybridJointspaceTaskspaceTrajectoryMessage();
            handHybridJointspaceTaskspaceTrajectoryMessage.setRobotSide(definition.getSide().toByte());
            handHybridJointspaceTaskspaceTrajectoryMessage.getTaskspaceTrajectoryMessage().set(se3TrajectoryMessage);
            handHybridJointspaceTaskspaceTrajectoryMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
            state.getLogger().info("Publishing arm hybrid jointspace taskpace");
            ros2ControllerHelper.publishToController(handHybridJointspaceTaskspaceTrajectoryMessage);
         }

         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));
         state.getCommandedTrajectory().setSingleSegmentTrajectory(syncedHandControlPose, desiredHandControlPose, definition.getTrajectoryDuration());
      }
      else
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   private void publishJointspaceCommand(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
      armTrajectoryMessage.setRobotSide(definition.getSide().toByte());
      armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      state.getLogger().info("Publishing arm jointspace trajectory");
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());
      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());
      if (syncedRobot.getHandWrenchCalculators().get(definition.getSide()) != null)
      {
         state.getForce().accessValue().set(syncedRobot.getHandWrenchCalculators().get(definition.getSide()).getFilteredWrench().getLinearPart());
         state.getTorque().accessValue().set(syncedRobot.getHandWrenchCalculators().get(definition.getSide()).getFilteredWrench().getAngularPart());
      }

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         state.getLogger().error("Task execution timed out.");
         return;
      }

      if (definition.getUsePredefinedJointAngles())
      {
         state.setIsExecuting(!trackingCalculator.getTimeIsUp());
      }
      else if (state.getPalmFrame().isChildOfWorld())
      {
         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));

         trackingCalculator.computePoseTrackingData(desiredHandControlPose, syncedHandControlPose);
         trackingCalculator.factorInR3Errors(definition.getPositionErrorTolerance());
         trackingCalculator.factoryInSO3Errors(definition.getOrientationErrorTolerance());

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.getCurrentPose().accessValue().set(syncedHandControlPose);
         state.setPositionDistanceToGoalTolerance(definition.getPositionErrorTolerance());
         state.setOrientationDistanceToGoalTolerance(definition.getOrientationErrorTolerance());

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);

            if (!definition.getJointspaceOnly() && !definition.getHoldPoseInWorldLater())
            {
               disengageHoldPoseInWorld();
            }
         }
      }
   }

   private void disengageHoldPoseInWorld()
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildJointspaceTrajectoryMessage();

      state.getLogger().info("Disengaging holding hand in taskspace");
      publishJointspaceCommand(jointspaceTrajectoryMessage);
   }

   private JointspaceTrajectoryMessage buildJointspaceTrajectoryMessage()
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      double[] jointAngles = new double[state.getNumberOfJoints()];

      if (definition.getUsePredefinedJointAngles())
         for (int i = 0; i < jointAngles.length; i++)
            jointAngles[i] = definition.getJointAngles().getValueReadOnly(i);
      else
         for (int i = 0; i < jointAngles.length; i++)
            jointAngles[i] = armIKSolvers.get(definition.getSide()).getSolutionOneDoFJoints()[i].getQ();

      for (double q : jointAngles)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         oneDoFJointTrajectoryMessage.setWeight(definition.getJointspaceWeight());

         TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
         trajectoryPoint1DMessage.setTime(definition.getTrajectoryDuration());
         trajectoryPoint1DMessage.setPosition(q);
         trajectoryPoint1DMessage.setVelocity(0.0);
      }

      return jointspaceTrajectoryMessage;
   }
}
