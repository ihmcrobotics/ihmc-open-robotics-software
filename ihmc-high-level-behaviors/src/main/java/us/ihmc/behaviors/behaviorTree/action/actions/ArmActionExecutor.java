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
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

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
   private final FramePose3D workPose = new FramePose3D();
   private final MultipleWaypointsPoseTrajectoryGenerator poseTrajectoryGenerator
         = new MultipleWaypointsPoseTrajectoryGenerator("", ArmActionState.TRAJECTORY_SIZE_LIMIT, new YoRegistry("Dummy"));
   private final ArmTrajectoryMessage jointspaceOnlyTrajectoryMessage = new ArmTrajectoryMessage();
   private final HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
   private final MutableReferenceFrame currentPoseFrame = new MutableReferenceFrame();
   private final FramePose3D currentPose = new FramePose3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final RecyclingArrayList<FrameVector3D> linearVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> angularVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final TDoubleArrayList trajectoryTimes = new TDoubleArrayList();
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
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      if (definition.getDefinedInJointspace())
      {
         updateJointspacePreview();
      }
      else if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE)
      {
         updateScrewPreview();
      }
      else
      {
         updateSinglePosePreview();
      }
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

   private void updateScrewPreview()
   {
      boolean definitionInvalid = definition.getRotation() == 0.0 && definition.getTranslation() == 0.0;
      state.setCanExecute(state.getScrewFrame().isChildOfWorld() && !definitionInvalid);

      if (state.getScrewFrame().isChildOfWorld() && !definitionInvalid)
      {
         BehaviorTreeRootNodeState actionSequence = rootNode.getState();
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
      {
         state.getGoalChestToWorldTransform().accessValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
      }
      else if (concurrentPelvisAction == null)
      {
         concurrentSpineAction.update();
         state.getGoalChestToWorldTransform().accessValue().set(concurrentSpineAction.getChestFrame().getReferenceFrame().getTransformToRoot());
      }
      else if (concurrentSpineAction == null)
      {
         state.getGoalChestToWorldTransform().accessValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
      }
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

   private ReferenceFrame resolveInitialHandFrame(BehaviorTreeRootNodeState actionSequence)
   {
      if (state.getIsNextForExecution())
         return syncedRobot.getReferenceFrames().getHandFrame(definition.getSide());

      ArmActionState previousHandPose = actionSequence.findNextPreviousLeaf(ArmActionState.class, state.getLeafIndex(), definition.getSide());
      if (previousHandPose != null && previousHandPose.getPalmFrame().isChildOfWorld())
         return previousHandPose.getPalmFrame().getReferenceFrame();

      return null;
   }

   private boolean buildPreviewTrajectory(ReferenceFrame initialHandFrame)
   {
      RecyclingArrayList<Pose3D> trajectoryPoses = state.getPreviewTrajectory().accessValue();
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

      if (segments > ArmActionState.TRAJECTORY_SIZE_LIMIT - 1)
         segments = ArmActionState.TRAJECTORY_SIZE_LIMIT - 1;

      for (int i = 0; i < segments; i++)
      {
         Pose3D previousPose = trajectoryPoses.getLast();
         Pose3D currentPose = trajectoryPoses.add();

         workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), previousPose);
         workPose.changeFrame(state.getScrewFrame().getReferenceFrame());

         workPose.prependRollRotation(rotationPerSegment);
         workPose.prependTranslation(translationPerSegment, 0.0, 0.0);

         workPose.changeFrame(ReferenceFrame.getWorldFrame());
         currentPose.set(workPose);
      }

      return true;
   }

   private boolean computeTrajectoryData(ScrewTrajectoryData data)
   {
      int previewSize = state.getPreviewTrajectory().getSize();
      if (previewSize < 2)
      {
         state.getLogger().error("Cannot execute screw primitive. Preview trajectory has %d point(s).".formatted(previewSize));
         return false;
      }

      data.numberOfPoints = previewSize;

      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));
      syncedHandControlPose.changeFrame(state.getScrewFrame().getReferenceFrame());
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
      state.getPreviewTrajectoryDuration().setValue(data.movementDuration);
      state.getPreviewTrajectoryLinearVelocity().setValue(data.totalLinearDistanceOfHand / data.movementDuration);
      state.getPreviewTrajectoryAngularVelocity().setValue(data.rotationalVelocity);
   }

   private void computeTrajectoryTimesAndVelocities(ScrewTrajectoryData data)
   {
      ReferenceFrame screwAxisFrame = state.getScrewFrame().getReferenceFrame();

      angularVelocities.clear();
      linearVelocities.clear();
      trajectoryTimes.resetQuick();

      angularVelocities.add().setToZero();
      linearVelocities.add().setToZero();
      trajectoryTimes.add(0.0);

      double time = 2.0 * data.segmentDuration;
      for (int i = 1; i < data.numberOfPoints - 1; i++)
      {
         Pose3DReadOnly waypointPose = state.getPreviewTrajectory().getValueReadOnly(i);

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
      ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
      armIKSolver.copySourceToWork();

      poseTrajectoryGenerator.clear(ReferenceFrame.getWorldFrame());
      for (int i = 0; i < data.numberOfPoints; i++)
      {
         currentPose.set(state.getPreviewTrajectory().getValueReadOnly(i));
         poseTrajectoryGenerator.appendPoseWaypoint(trajectoryTimes.get(i), currentPose, linearVelocities.get(i), angularVelocities.get(i));
      }

      poseTrajectoryGenerator.initialize();
      poseTrajectoryGenerator.compute(data.movementDuration * state.getPreviewRequestedTime().getValue());

      currentPoseFrame.getTransformToParent().set(poseTrajectoryGenerator.getPose());
      currentPoseFrame.getReferenceFrame().update();
      linearVelocity.set(poseTrajectoryGenerator.getVelocity());
      angularVelocity.set(poseTrajectoryGenerator.getAngularVelocity());

      armIKSolver.update(syncedRobot.getReferenceFrames().getChestFrame(), currentPoseFrame.getReferenceFrame());
      armIKSolver.solve(angularVelocity, linearVelocity);

      state.getPreviewSolutionQuality().setValue(armIKSolver.getQuality());
      for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         state.getScrewPreviewJointAngles().setValue(i, armIKSolver.getSolutionOneDoFJoints()[i].getQ());
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
      {
         triggerJointspaceExecution();
      }
      else if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE)
      {
         triggerScrewExecution();
      }
      else
      {
         triggerSinglePoseExecution();
      }
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
      {
         publishJointspaceCommand(jointspaceTrajectoryMessage);
      }
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

   private void triggerScrewExecution()
   {
      if (Double.isNaN(state.getPreviewTrajectoryLinearVelocity().getValue())
       || Double.isNaN(state.getPreviewTrajectoryAngularVelocity().getValue())
       || (definition.getRotation() == 0.0 && definition.getTranslation() == 0.0))
      {
         state.setFailed(true);
         state.getLogger().error("Cannot execute screw primitive with velocities:   Velocity %.2f m/s  %.2f %s/s"
                              .formatted(state.getPreviewTrajectoryLinearVelocity().getValue(),
                                         state.getPreviewTrajectoryAngularVelocity().getValue(),
                                         EuclidCoreMissingTools.DEGREE_SYMBOL));
         return;
      }

      if (!state.getScrewFrame().isChildOfWorld())
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

      ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
      armIKSolver.copySourceToWork();

      for (int i = 0; i < trajectoryData.numberOfPoints; i++)
      {
         Pose3DReadOnly desiredPose = state.getPreviewTrajectory().getValueReadOnly(i);
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

         armIKSolver.update(syncedRobot.getReferenceFrames().getChestFrame(), currentPoseFrame.getReferenceFrame());
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
         ros2ControllerHelper.publishToController(jointspaceOnlyTrajectoryMessage);
      }
      else
      {
         state.getLogger().info("Commanding %.3f s hybrid trajectory with %d points"
                                .formatted(trajectoryData.movementDuration, trajectoryData.numberOfPoints));
         ros2ControllerHelper.publishToController(handHybridTrajectoryMessage);
      }

      trackingCalculator.reset();
      state.setNominalExecutionDuration(trajectoryData.movementDuration);
   }

   private JointspaceTrajectoryMessage prepareJointspaceTrajectoryMessage()
   {
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
         {
            state.getLogger().error("%s  %s"
                    .formatted("Position error: %.3f / %.3f".formatted(trackingCalculator.getPositionError(), definition.getPositionErrorTolerance()),
                               "Orientation error: %.3f / %.3f".formatted(trackingCalculator.getOrientationError(), definition.getOrientationErrorTolerance())));
         }
         return;
      }

      if (definition.getDefinedInJointspace())
      {
         updateJointspaceExecution();
      }
      else if (definition.getTaskspaceTrajectoryMode() == ArmActionTaskspaceTrajectoryMode.SCREW_PRIMITIVE)
      {
         updateScrewExecution();
      }
      else
      {
         updateSinglePoseExecution();
      }
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

   private void updateScrewExecution()
   {
      if (!state.getScrewFrame().isChildOfWorld())
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
         state.setIsExecuting(false);
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
}
