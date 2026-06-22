package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ScrewPrimitiveExecutor
{
   private final ArmActionExecutor parent;
   private final ScrewPrimitiveState screwState;
   private final FramePose3D workPose = new FramePose3D();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final FramePose3D currentPose = new FramePose3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final RecyclingArrayList<FrameVector3D> linearVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> angularVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final TDoubleArrayList trajectoryTimes = new TDoubleArrayList();
   private final MultipleWaypointsPoseTrajectoryGenerator poseTrajectoryGenerator
         = new MultipleWaypointsPoseTrajectoryGenerator("", ScrewPrimitiveState.TRAJECTORY_SIZE_LIMIT, new YoRegistry("Dummy"));
   private final ArmTrajectoryMessage jointspaceOnlyTrajectoryMessage = new ArmTrajectoryMessage();
   private final HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
   private final MutableReferenceFrame currentPoseFrame = new MutableReferenceFrame();
   private final ScrewTrajectoryData trajectoryData = new ScrewTrajectoryData();

   private static final class ScrewTrajectoryData
   {
      int numberOfPoints;
      double rotationRadius;
      double signedTotalRotation;
      double signedTotalTranslation;
      double signedRadialDistance;
      double totalLinearDistanceOfHand;
      double movementDuration;
      double segmentDuration;
      double tangentialVelocity;
      double axialVelocity;
      double rotationalVelocity;
   }

   public ScrewPrimitiveExecutor(ArmActionExecutor parent)
   {
      this.parent = parent;
      screwState = parent.getState().getScrewPrimitive();
   }

   public void updatePreview()
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();
      boolean definitionInvalid = definition.getRotation() == 0.0 && definition.getTranslation() == 0.0;
      state.setCanExecute(screwState.getScrewFrame().isChildOfWorld() && !definitionInvalid);

      if (screwState.getScrewFrame().isChildOfWorld() && !definitionInvalid)
      {
         BehaviorTreeRootNodeState actionSequence = parent.getRootNode().getState();
         if (actionSequence.getExecutionNextIndex() > state.getLeafIndex())
            return;

         ReferenceFrame initialHandFrame = resolveInitialHandFrame(actionSequence);
         if (initialHandFrame == null)
            return;

         if (!buildPreviewTrajectory(initialHandFrame))
            return;

         if (!computeTrajectoryData(trajectoryData))
            return;

         updatePreviewMetrics(trajectoryData);

         if (state.getIsNextForExecution())
            buildTrajectory(trajectoryData, true);
      }
   }

   public void triggerExecution()
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();

      if (Double.isNaN(screwState.getPreviewTrajectoryLinearVelocity().getValue())
       || Double.isNaN(screwState.getPreviewTrajectoryAngularVelocity().getValue())
       || (definition.getRotation() == 0.0 && definition.getTranslation() == 0.0))
      {
         state.setFailed(true);
         state.getLogger().error("Cannot execute screw primitive with velocities:   Velocity %.2f m/s  %.2f %s/s"
                              .formatted(screwState.getPreviewTrajectoryLinearVelocity().getValue(),
                                         screwState.getPreviewTrajectoryAngularVelocity().getValue(),
                                         EuclidCoreMissingTools.DEGREE_SYMBOL));
         return;
      }

      if (!screwState.getScrewFrame().isChildOfWorld())
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
         return;
      }

      if (!computeTrajectoryData(trajectoryData))
      {
         state.setFailed(true);
         return;
      }

      buildTrajectory(trajectoryData, false);

      state.getCommandedTrajectory().accessValue().clear();

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = prepareJointspaceTrajectoryMessage();
      SE3TrajectoryMessage taskspaceTrajectoryMessage = prepareTaskspaceTrajectoryMessage();

      ArmIKSolver armIKSolver = parent.getArmIKSolvers().get(definition.getSide());
      armIKSolver.copySourceToWork();

      for (int i = 0; i < trajectoryData.numberOfPoints; i++)
      {
         Pose3DReadOnly desiredPose = screwState.getPreviewTrajectory().getValueReadOnly(i);
         angularVelocity.setIncludingFrame(angularVelocities.get(i));
         linearVelocity.setIncludingFrame(linearVelocities.get(i));
         double waypointTime = trajectoryTimes.get(i);

         currentPoseFrame.getTransformToParent().set(desiredPose);
         currentPoseFrame.getReferenceFrame().update();

         SE3TrajectoryPointMessage se3TrajectoryPointMessage = taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().add();
         se3TrajectoryPointMessage.setTime(waypointTime);
         se3TrajectoryPointMessage.getPosition().set(desiredPose.getTranslation());
         se3TrajectoryPointMessage.getOrientation().set(desiredPose.getOrientation());
         se3TrajectoryPointMessage.getLinearVelocity().set(linearVelocity);
         se3TrajectoryPointMessage.getAngularVelocity().set(angularVelocity);

         armIKSolver.update(parent.getSyncedRobot().getReferenceFrames().getChestFrame(), currentPoseFrame.getReferenceFrame());
         armIKSolver.solve(angularVelocity, linearVelocity);

         if (armIKSolver.getQuality() > ArmIKSolver.GOOD_QUALITY_MAX)
            state.getLogger().warn("Bad quality: {} (i == {})", armIKSolver.getQuality(), i);

         for (int j = 0; j < state.getNumberOfJoints(); j++)
         {
            OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(j);

            TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
            trajectoryPoint1DMessage.setTime(waypointTime);
            trajectoryPoint1DMessage.setPosition(armIKSolver.getSolutionOneDoFJoints()[j].getQ());
            trajectoryPoint1DMessage.setVelocity(armIKSolver.getSolutionOneDoFJoints()[j].getQd());
         }

         LogTools.info("Adding point time: %.2f  nextPose: %s %s  linearVel: %s  angularVel: %s"
                 .formatted(waypointTime,
                            desiredPose.getPosition(),
                            new YawPitchRoll(desiredPose.getOrientation()),
                            linearVelocity,
                            angularVelocity));

         state.getCommandedTrajectory().addTrajectoryPoint(desiredPose, waypointTime);
      }

      for (int i = 1; i < trajectoryData.numberOfPoints - 1; i++)
      {
         for (int jointIdx = 0; jointIdx < state.getNumberOfJoints(); jointIdx++)
         {
            OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(jointIdx);
            TrajectoryPoint1DMessage previousPoint = oneDoFJointTrajectoryMessage.getTrajectoryPoints().get(i - 1);
            TrajectoryPoint1DMessage currentPoint = oneDoFJointTrajectoryMessage.getTrajectoryPoints().get(i);
            TrajectoryPoint1DMessage nextPoint = oneDoFJointTrajectoryMessage.getTrajectoryPoints().get(i + 1);
            double duration = nextPoint.getTime() - previousPoint.getTime();
            double displacement = AngleTools.computeAngleDifferenceMinusPiToPi(nextPoint.getPosition(), previousPoint.getPosition());
            currentPoint.setVelocity(displacement / duration);
         }
      }

      if (definition.getJointspaceOnly())
      {
         state.getLogger().info("Commanding %.3f s jointspace only trajectory with %d points"
                                .formatted(trajectoryData.movementDuration, trajectoryData.numberOfPoints));
         jointspaceOnlyTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
         parent.getRos2ControllerHelper().publishToController(jointspaceOnlyTrajectoryMessage);
      }
      else
      {
         state.getLogger().info("Commanding %.3f s hybrid trajectory with %d points"
                                .formatted(trajectoryData.movementDuration, trajectoryData.numberOfPoints));
         parent.getRos2ControllerHelper().publishToController(handHybridTrajectoryMessage);
      }

      parent.getTrackingCalculator().reset();
      state.setNominalExecutionDuration(trajectoryData.movementDuration);
   }

   public void updateExecution()
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();

      if (!screwState.getScrewFrame().isChildOfWorld())
         return;

      if (state.getCommandedTrajectory().isEmpty())
      {
         state.getLogger().error("Commanded trajectory is empty.");
         state.setIsExecuting(false);
         state.setFailed(true);
         return;
      }

      SE3TrajectoryPointReadOnly lastTrajectoryPose = state.getCommandedTrajectory().getLastValueReadOnly();
      desiredHandControlPose.set(lastTrajectoryPose.getPosition(), lastTrajectoryPose.getOrientation());
      syncedHandControlPose.setFromReferenceFrame(parent.getSyncedRobot().getFullRobotModel().getHandControlFrame(definition.getSide()));

      parent.getTrackingCalculator().computePoseTrackingData(desiredHandControlPose, syncedHandControlPose);
      parent.getTrackingCalculator().factorInR3Errors(definition.getPositionErrorTolerance());
      parent.getTrackingCalculator().factoryInSO3Errors(definition.getOrientationErrorTolerance());

      boolean meetsDesiredCompletionCriteria = parent.getTrackingCalculator().isWithinPositionTolerance();
      meetsDesiredCompletionCriteria &= parent.getTrackingCalculator().getTimeIsUp();

      state.getCurrentPose().accessValue().set(syncedHandControlPose);
      state.setPositionDistanceToGoalTolerance(definition.getPositionErrorTolerance());
      state.setOrientationDistanceToGoalTolerance(definition.getOrientationErrorTolerance());

      if (meetsDesiredCompletionCriteria)
         state.setIsExecuting(false);
   }

   private ReferenceFrame resolveInitialHandFrame(BehaviorTreeRootNodeState actionSequence)
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();

      if (state.getIsNextForExecution())
         return parent.getSyncedRobot().getReferenceFrames().getHandFrame(definition.getSide());

      ArmActionState previousHandPose = actionSequence.findNextPreviousLeaf(ArmActionState.class, state.getLeafIndex(), definition.getSide());
      if (previousHandPose != null && previousHandPose.getPalmFrame().isChildOfWorld())
         return previousHandPose.getPalmFrame().getReferenceFrame();

      return null;
   }

   private boolean buildPreviewTrajectory(ReferenceFrame initialHandFrame)
   {
      ArmActionDefinition definition = parent.getDefinition();
      RecyclingArrayList<Pose3D> trajectoryPoses = screwState.getPreviewTrajectory().accessValue();
      trajectoryPoses.clear();
      Pose3D firstPose = trajectoryPoses.add();
      workPose.setToZero(initialHandFrame);
      workPose.changeFrame(ReferenceFrame.getWorldFrame());
      firstPose.set(workPose);

      double rotationPerPoint = Math.toRadians(10);
      double translationPerPoint = 0.05;
      int segments = (int) Math.ceil(Math.abs(definition.getRotation()) / rotationPerPoint
                                   + Math.abs(definition.getTranslation()) / translationPerPoint);

      if (segments <= 0)
         return false;

      double rotationPerSegment = definition.getRotation() / segments;
      double translationPerSegment = definition.getTranslation() / segments;

      if (segments > ScrewPrimitiveState.TRAJECTORY_SIZE_LIMIT - 1)
         segments = ScrewPrimitiveState.TRAJECTORY_SIZE_LIMIT - 1;

      for (int i = 0; i < segments; i++)
      {
         Pose3D previousPose = trajectoryPoses.getLast();
         Pose3D currentPose = trajectoryPoses.add();

         workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), previousPose);
         workPose.changeFrame(screwState.getScrewFrame().getReferenceFrame());

         workPose.prependRollRotation(rotationPerSegment);
         workPose.prependTranslation(translationPerSegment, 0.0, 0.0);

         workPose.changeFrame(ReferenceFrame.getWorldFrame());
         currentPose.set(workPose);
      }

      return true;
   }

   private boolean computeTrajectoryData(ScrewTrajectoryData data)
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();
      int previewSize = screwState.getPreviewTrajectory().getSize();
      if (previewSize < 2)
      {
         state.getLogger().error("Cannot execute screw primitive. Preview trajectory has %d point(s).".formatted(previewSize));
         return false;
      }

      data.numberOfPoints = previewSize;

      syncedHandControlPose.setFromReferenceFrame(parent.getSyncedRobot().getFullRobotModel().getHandControlFrame(definition.getSide()));
      syncedHandControlPose.changeFrame(screwState.getScrewFrame().getReferenceFrame());
      data.rotationRadius = EuclidCoreTools.norm(syncedHandControlPose.getY(), syncedHandControlPose.getZ());
      syncedHandControlPose.changeFrame(ReferenceFrame.getWorldFrame());

      data.signedTotalRotation = definition.getRotation();
      data.signedTotalTranslation = definition.getTranslation();
      data.signedRadialDistance = data.signedTotalRotation * data.rotationRadius;
      data.totalLinearDistanceOfHand = EuclidCoreTools.norm(data.signedRadialDistance, data.signedTotalTranslation);

      double durationForRotation = Math.abs(data.signedTotalRotation) / definition.getMaxAngularVelocity();
      double durationForTranslation = data.totalLinearDistanceOfHand / definition.getMaxLinearVelocity();
      double baseMovementDuration = Math.max(durationForRotation, durationForTranslation);
      if (!Double.isFinite(baseMovementDuration) || baseMovementDuration <= 0.0)
      {
         state.getLogger().error("Cannot execute screw primitive. Invalid movement duration: %.3f".formatted(baseMovementDuration));
         return false;
      }

      data.segmentDuration = baseMovementDuration / (data.numberOfPoints - 1);
      if (!Double.isFinite(data.segmentDuration) || data.segmentDuration <= 0.0)
      {
         state.getLogger().error("Cannot execute screw primitive. Invalid segment duration: %.6f".formatted(data.segmentDuration));
         return false;
      }

      data.tangentialVelocity = data.signedRadialDistance / baseMovementDuration;
      data.axialVelocity = data.signedTotalTranslation / baseMovementDuration;
      data.rotationalVelocity = data.signedTotalRotation / baseMovementDuration;
      data.movementDuration = baseMovementDuration + 2.0 * data.segmentDuration;

      return true;
   }

   private void updatePreviewMetrics(ScrewTrajectoryData data)
   {
      screwState.getPreviewTrajectoryDuration().setValue(data.movementDuration);
      screwState.getPreviewTrajectoryLinearVelocity().setValue(data.totalLinearDistanceOfHand / data.movementDuration);
      screwState.getPreviewTrajectoryAngularVelocity().setValue(data.rotationalVelocity);
   }

   private void computeTrajectoryTimesAndVelocities(ScrewTrajectoryData data)
   {
      ReferenceFrame screwAxisFrame = screwState.getScrewFrame().getReferenceFrame();

      angularVelocities.clear();
      linearVelocities.clear();
      trajectoryTimes.resetQuick();

      angularVelocities.add().setToZero();
      linearVelocities.add().setToZero();
      trajectoryTimes.add(0.0);

      double time = 2.0 * data.segmentDuration;
      for (int i = 1; i < data.numberOfPoints - 1; i++)
      {
         Pose3DReadOnly waypointPose = screwState.getPreviewTrajectory().getValueReadOnly(i);

         workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), waypointPose);
         workPose.changeFrame(screwAxisFrame);

         Vector3DReadOnly rotationAxis = Axis3D.X;
         Vector3D radialPosition = new Vector3D(workPose.getPosition());
         radialPosition.setX(0.0);
         Vector3D tangentVector = new Vector3D();
         tangentVector.cross(rotationAxis, radialPosition);
         tangentVector.normalize();

         FrameVector3D linearVelocity = linearVelocities.add();
         linearVelocity.setToZero(screwAxisFrame);
         linearVelocity.setAndScale(data.tangentialVelocity, tangentVector);
         linearVelocity.setX(data.axialVelocity);
         linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

         FrameVector3D angularVelocity = angularVelocities.add();
         angularVelocity.setToZero(screwAxisFrame);
         angularVelocity.setX(data.rotationalVelocity);
         angularVelocity.changeFrame(ReferenceFrame.getWorldFrame());

         trajectoryTimes.add(time);
         time += data.segmentDuration;
      }

      angularVelocities.add().setToZero();
      linearVelocities.add().setToZero();
      trajectoryTimes.add(data.movementDuration);
   }

   private void buildTrajectory(ScrewTrajectoryData data, boolean forPreview)
   {
      computeTrajectoryTimesAndVelocities(data);
      if (forPreview)
         updatePreviewForOperator(data);
   }

   private void updatePreviewForOperator(ScrewTrajectoryData data)
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmIKSolver armIKSolver = parent.getArmIKSolvers().get(definition.getSide());
      armIKSolver.copySourceToWork();

      poseTrajectoryGenerator.clear(ReferenceFrame.getWorldFrame());
      for (int i = 0; i < data.numberOfPoints; i++)
      {
         currentPose.set(screwState.getPreviewTrajectory().getValueReadOnly(i));
         poseTrajectoryGenerator.appendPoseWaypoint(trajectoryTimes.get(i), currentPose, linearVelocities.get(i), angularVelocities.get(i));
      }

      poseTrajectoryGenerator.initialize();
      poseTrajectoryGenerator.compute(data.movementDuration * screwState.getPreviewRequestedTime().getValue());

      currentPoseFrame.getTransformToParent().set(poseTrajectoryGenerator.getPose());
      currentPoseFrame.getReferenceFrame().update();
      linearVelocity.set(poseTrajectoryGenerator.getVelocity());
      angularVelocity.set(poseTrajectoryGenerator.getAngularVelocity());

      armIKSolver.update(parent.getSyncedRobot().getReferenceFrames().getChestFrame(), currentPoseFrame.getReferenceFrame());
      armIKSolver.solve(angularVelocity, linearVelocity);

      screwState.getPreviewSolutionQuality().setValue(armIKSolver.getQuality());
      for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         screwState.getScrewPreviewJointAngles().setValue(i, armIKSolver.getSolutionOneDoFJoints()[i].getQ());
   }

   private JointspaceTrajectoryMessage prepareJointspaceTrajectoryMessage()
   {
      ArmActionDefinition definition = parent.getDefinition();
      ArmActionState state = parent.getState();

      jointspaceOnlyTrajectoryMessage.setRobotSide(definition.getSide().toByte());
      jointspaceOnlyTrajectoryMessage.setForceExecution(true);
      handHybridTrajectoryMessage.setRobotSide(definition.getSide().toByte());
      handHybridTrajectoryMessage.setForceExecution(true);

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = handHybridTrajectoryMessage.getJointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      jointspaceTrajectoryMessage.getJointTrajectoryMessages().clear();

      for (int jointNumber = 0; jointNumber < state.getNumberOfJoints(); jointNumber++)
      {
         OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
         oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
         oneDoFJointTrajectoryMessage.setWeight(definition.getJointspaceWeight());
      }

      return jointspaceTrajectoryMessage;
   }

   private SE3TrajectoryMessage prepareTaskspaceTrajectoryMessage()
   {
      ArmActionDefinition definition = parent.getDefinition();
      SE3TrajectoryMessage taskspaceTrajectoryMessage = handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage();
      taskspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      taskspaceTrajectoryMessage.getLinearWeightMatrix().setXWeight(definition.getLinearPositionWeight());
      taskspaceTrajectoryMessage.getLinearWeightMatrix().setYWeight(definition.getLinearPositionWeight());
      taskspaceTrajectoryMessage.getLinearWeightMatrix().setZWeight(definition.getLinearPositionWeight());
      taskspaceTrajectoryMessage.getAngularWeightMatrix().setXWeight(definition.getAngularPositionWeight());
      taskspaceTrajectoryMessage.getAngularWeightMatrix().setYWeight(definition.getAngularPositionWeight());
      taskspaceTrajectoryMessage.getAngularWeightMatrix().setZWeight(definition.getAngularPositionWeight());
      taskspaceTrajectoryMessage.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      taskspaceTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().clear();
      return taskspaceTrajectoryMessage;
   }
}
