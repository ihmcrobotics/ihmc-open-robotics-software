package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ArmActionExecutor extends ActionNodeExecutor<ArmActionState, ArmActionDefinition>
{
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();
   private final RigidBodyTransform chestToPelvisZeroAngles = new RigidBodyTransform();
   private final FramePose3D chestInPelvis = new FramePose3D();
   private final FramePose3D goalChestFrame = new FramePose3D();
   private final double[] desiredJointAngles;
   private final ArmJointName[] armJointNames;
   private final ScrewPrimitiveExecutor screwPrimitive;

   public ArmActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new ArmActionState(id, rootNode.getState()), rootNode);

      desiredJointAngles = new double[state.getNumberOfJoints()];
      armJointNames = robotModel.getJointMap().getArmJointNames(definition.getSide());

      for (RobotSide side : RobotSide.values)
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel.getJointMap(), syncedRobot.getFullRobotModel()));

      FramePose3D chestAfterJointToPelvis = new FramePose3D();
      chestAfterJointToPelvis.setToZero(syncedRobot.getReferenceFrames().getChestFrame());
      chestAfterJointToPelvis.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
      chestAfterJointToPelvis.get(chestToPelvisZeroAngles);

      screwPrimitive = new ScrewPrimitiveExecutor(this);
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      if (definition.getDefinedInJointspace())
         updateJointspacePreview();
      else if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE)
         screwPrimitive.updatePreview();
      else
         updateSinglePosePreview();
   }

   private void updateJointspacePreview()
   {
      state.setCanExecute(true);
      state.setSolutionQuality(0.0);
      for (int i = 0; i < definition.getJointAngles().getLength(); i++)
         state.getPreviewJointAngles().accessValue()[i] = definition.getJointAngles().getValueReadOnly(i);
   }

   private void updateSinglePosePreview()
   {
      updateGoalChestTransform();
      state.setCanExecute(state.getPalmFrame().isChildOfWorld());

      if (state.getPalmFrame().isChildOfWorld() && state.getIsNextForExecution())
         solveSinglePoseIK();
   }

   private void updateGoalChestTransform()
   {
      if (!state.getIsNextForExecution())
         return;

      SpineActionState concurrentSpineAction = null;
      PelvisActionState concurrentPelvisAction = null;

      for (int i = state.getExecuteAfterLeafIndex() + 1; i < state.getLeafIndex(); i++)
      {
         if (rootNode.getState().getOrderedLeaves().get(i) instanceof SpineActionState spineAction)
            concurrentSpineAction = spineAction;
         if (rootNode.getState().getOrderedLeaves().get(i) instanceof PelvisActionState pelvisAction)
            concurrentPelvisAction = pelvisAction;
      }

      for (LeafNodeExecutor<?, ?> currentlyExecutingLeaf : rootNode.getCurrentlyExecutingLeaves())
      {
         if (currentlyExecutingLeaf.getState() instanceof SpineActionState spineAction)
            concurrentSpineAction = spineAction;
         if (currentlyExecutingLeaf.getState() instanceof PelvisActionState pelvisAction)
            concurrentPelvisAction = pelvisAction;
      }

      if (concurrentSpineAction == null && concurrentPelvisAction == null)
         state.getGoalChestToWorldTransform().accessValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
      else if (concurrentPelvisAction == null)
      {
         concurrentSpineAction.update();
         state.getGoalChestToWorldTransform().accessValue().set(concurrentSpineAction.getChestFrame().getReferenceFrame().getTransformToRoot());
      }
      else if (concurrentSpineAction == null)
         state.getGoalChestToWorldTransform().accessValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
      else
      {
         concurrentSpineAction.update();
         concurrentPelvisAction.update();

         ReferenceFrame chestActionFrame = concurrentSpineAction.getChestFrame().getReferenceFrame();
         ReferenceFrame pelvisActionFrame = concurrentPelvisAction.getPelvisFrame().getReferenceFrame();

         chestInPelvis.setToZero(chestActionFrame);
         chestInPelvis.changeFrame(pelvisActionFrame);

         goalChestFrame.setToZero(pelvisActionFrame);
         goalChestFrame.getRotation().append(chestInPelvis.getRotation());
         goalChestFrame.prependTranslation(chestToPelvisZeroAngles.getTranslation());
         goalChestFrame.changeFrame(ReferenceFrame.getWorldFrame());
         goalChestFrame.get(state.getGoalChestToWorldTransform().accessValue());
      }
      state.getGoalChestFrame().update();
   }

   private void solveSinglePoseIK()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
      armIKSolver.copySourceToWork();
      armIKSolver.update(state.getGoalChestFrame(), state.getPalmFrame().getReferenceFrame());
      armIKSolver.solve();

      state.setSolutionQuality(armIKSolver.getQuality());
      for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         state.getPreviewJointAngles().accessValue()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      if (definition.getDefinedInJointspace())
         triggerJointspaceExecution();
      else if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE)
         screwPrimitive.triggerExecution();
      else
         triggerSinglePoseExecution();
   }

   private void triggerJointspaceExecution()
   {
      state.setNominalExecutionDuration(definition.getTrajectoryDuration());
      trackingCalculator.reset();
      publishJointspaceCommand(buildJointspaceTrajectoryMessage());
   }

   private void triggerSinglePoseExecution()
   {
      state.setNominalExecutionDuration(definition.getTrajectoryDuration());
      trackingCalculator.reset();

      if (!state.getPalmFrame().isChildOfWorld())
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
         return;
      }

      solveSinglePoseIK();
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = buildJointspaceTrajectoryMessage();

      if (definition.getJointspaceOnly())
         publishJointspaceCommand(jointspaceTrajectoryMessage);
      else
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

   private void publishJointspaceCommand(JointspaceTrajectoryMessage jointspaceTrajectoryMessage)
   {
      ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
      armTrajectoryMessage.setRobotSide(definition.getSide().toByte());
      armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
      armTrajectoryMessage.setForceExecution(true);
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

      if (trackingCalculator.getHitTimeLimit(state.getLogger()))
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         if (!definition.getDefinedInJointspace() && definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SINGLE_POSE)
            state.getLogger().error("%s  %s"
                    .formatted("Position error: %.3f / %.3f".formatted(trackingCalculator.getPositionError(), definition.getPositionErrorTolerance()),
                               "Orientation error: %.3f / %.3f".formatted(trackingCalculator.getOrientationError(), definition.getOrientationErrorTolerance())));
         return;
      }

      if (definition.getDefinedInJointspace())
         updateJointspaceExecution();
      else if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE)
         screwPrimitive.updateExecution();
      else
         updateSinglePoseExecution();
   }

   private void updateJointspaceExecution()
   {
      trackingCalculator.resetJointspaceError();

      for (int i = 0; i < desiredJointAngles.length; i++)
      {
         double desired = desiredJointAngles[i];
         double current = syncedRobot.getFullRobotModel().getArmJoint(definition.getSide(), armJointNames[i]).getQ();
         trackingCalculator.addJointData(desired, current);
      }
      trackingCalculator.factorInJointspaceErrors(definition.getPositionErrorTolerance());

      if (trackingCalculator.getTimeIsUp())
      {
         state.setIsExecuting(false);
         if (!trackingCalculator.isWithinPositionTolerance())
         {
            state.getLogger().error("Total jointspace error: %.3f deg".formatted(Math.toDegrees(trackingCalculator.getTotalAbsoluteJointspaceError())));
            state.setFailed(true);
         }
      }
   }

   private void updateSinglePoseExecution()
   {
      if (!state.getPalmFrame().isChildOfWorld())
         return;

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
            disengageHoldPoseInWorld();
      }
   }

   private void disengageHoldPoseInWorld()
   {
      state.getLogger().info("Disengaging holding hand in taskspace");
      publishJointspaceCommand(buildJointspaceTrajectoryMessage());
   }

   private JointspaceTrajectoryMessage buildJointspaceTrajectoryMessage()
   {
      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      if (definition.getDefinedInJointspace())
         for (int i = 0; i < desiredJointAngles.length; i++)
            desiredJointAngles[i] = definition.getJointAngles().getValueReadOnly(i);
      else
         for (int i = 0; i < desiredJointAngles.length; i++)
            desiredJointAngles[i] = armIKSolvers.get(definition.getSide()).getSolutionOneDoFJoints()[i].getQ();

      for (double q : desiredJointAngles)
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

   SideDependentList<ArmIKSolver> getArmIKSolvers()
   {
      return armIKSolvers;
   }

   TrajectoryTrackingErrorCalculator getTrackingCalculator()
   {
      return trackingCalculator;
   }

   ROS2SyncedRobotModel getSyncedRobot()
   {
      return syncedRobot;
   }

   ROS2ControllerHelper getRos2ControllerHelper()
   {
      return ros2ControllerHelper;
   }
}
