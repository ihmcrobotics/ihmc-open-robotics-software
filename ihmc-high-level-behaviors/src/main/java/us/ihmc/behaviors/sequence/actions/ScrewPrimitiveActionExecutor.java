package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TDoubleArrayList;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
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
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ScrewPrimitiveActionExecutor extends ActionNodeExecutor<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   private final ScrewPrimitiveActionState state;
   private final ScrewPrimitiveActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final TaskspaceTrajectoryTrackingErrorCalculator trackingCalculator = new TaskspaceTrajectoryTrackingErrorCalculator();
   private final FramePose3D workPose = new FramePose3D();
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final MultipleWaypointsPoseTrajectoryGenerator poseTrajectoryGenerator
         = new MultipleWaypointsPoseTrajectoryGenerator("", ScrewPrimitiveActionState.TRAJECTORY_SIZE_LIMIT, new YoRegistry("Dummy"));
   private final ArmTrajectoryMessage jointspaceOnlyTrajectoryMessage = new ArmTrajectoryMessage();
   private final HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
   private final MutableReferenceFrame currentPoseFrame = new MutableReferenceFrame();
   private final FramePose3D currentPose = new FramePose3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final RecyclingArrayList<FrameVector3D> linearVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> angularVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final TDoubleArrayList trajectoryTimes = new TDoubleArrayList();
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
   private final int numberOfJoints = ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS;
   private int numberOfPoints;
   private double rotationRadius;
   private double signedTotalRotation;
   private double signedTotalTranslation;
   private double signedRadialDistance;
   private double totalLinearDistanceOfHand;
   private double durationForRotation;
   private double durationForTranslation;
   private double movementDuration;
   private double segmentDuration;
   private double tangentialVelocity;
   private double axialVelocity;
   private double rotationalVelocity;

   public ScrewPrimitiveActionExecutor(long id,
                                       CRDTInfo crdtInfo,
                                       WorkspaceResourceDirectory saveFileDirectory,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       ReferenceFrameLibrary referenceFrameLibrary,
                                       DRCRobotModel robotModel,
                                       ROS2SyncedRobotModel syncedRobot)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      state = getState();
      definition = getDefinition();

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(state.getScrewFrame().isChildOfWorld());

      if (state.getScrewFrame().isChildOfWorld())
      {
         if (getParent().getState() instanceof ActionSequenceState parent)
         {
            if (parent.getExecutionNextIndex() <= state.getActionIndex())
            {
               ReferenceFrame initialHandFrame = null;

               if (state.getIsNextForExecution())
               {
                  initialHandFrame = syncedRobot.getReferenceFrames().getHandFrame(definition.getSide());
               }
               else
               {
                  HandPoseActionState previousHandPose = parent.findNextPreviousAction(HandPoseActionState.class,
                                                                                       state.getActionIndex(),
                                                                                       definition.getSide());
                  if (previousHandPose != null && previousHandPose.getPalmFrame().isChildOfWorld())
                  {
                     initialHandFrame = previousHandPose.getPalmFrame().getReferenceFrame();
                  }
               }

               if (initialHandFrame != null)
               {
                  RecyclingArrayList<Pose3D> trajectoryPoses = state.getPreviewTrajectory().getValue();
                  trajectoryPoses.clear();
                  Pose3D firstPose = trajectoryPoses.add();
                  workPose.setToZero(initialHandFrame);
                  workPose.changeFrame(ReferenceFrame.getWorldFrame());
                  firstPose.set(workPose);

                  // These contants could be adjusted
                  double rotationPerPoint = Math.toRadians(10);
                  double translationPerPoint = 0.05;
                  int segments = (int) Math.ceil(Math.abs(definition.getRotation()) / rotationPerPoint
                                               + Math.abs(definition.getTranslation()) / translationPerPoint);

                  double rotationPerSegment = definition.getRotation() / segments;
                  double translationPerSegment = definition.getTranslation() / segments;

                  if (segments > ScrewPrimitiveActionState.TRAJECTORY_SIZE_LIMIT - 1)
                  {
                     segments = ScrewPrimitiveActionState.TRAJECTORY_SIZE_LIMIT - 1; // We have to fit within the message size limit
                  }

                  // Generate the trajectory poses
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

                  numberOfPoints = state.getPreviewTrajectory().getSize();

                  syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));
                  syncedHandControlPose.changeFrame(state.getScrewFrame().getReferenceFrame());
                  // This is always the radial distance.
                  rotationRadius = EuclidCoreTools.norm(syncedHandControlPose.getY(), syncedHandControlPose.getZ());
                  syncedHandControlPose.changeFrame(ReferenceFrame.getWorldFrame());

                  signedTotalRotation = definition.getRotation();
                  signedTotalTranslation = definition.getTranslation();
                  // This is the distance the hand must travel along the screw portion
                  signedRadialDistance = signedTotalRotation * rotationRadius;
                  totalLinearDistanceOfHand = EuclidCoreTools.norm(signedRadialDistance, signedTotalTranslation);

                  // Computing the movement duration, which is clamped by the max movement speed
                  durationForRotation = Math.abs(signedTotalRotation) / definition.getMaxAngularVelocity();
                  durationForTranslation = totalLinearDistanceOfHand / definition.getMaxLinearVelocity();
                  movementDuration = Math.max(durationForRotation, durationForTranslation);
                  segmentDuration = movementDuration / (numberOfPoints - 1);

                  // The way the screw frame is defined, x is always the axis of rotation and translation.
                  // This means that the tangential velocity is normal to the x axis and the vector yz
                  tangentialVelocity = signedRadialDistance / movementDuration;
                  axialVelocity = signedTotalTranslation / movementDuration;
                  rotationalVelocity = signedTotalRotation / movementDuration;

                  movementDuration += 2.0 * segmentDuration;

                  state.getPreviewTrajectoryDuration().setValue(movementDuration);
                  state.getPreviewTrajectoryLinearVelocity().setValue(totalLinearDistanceOfHand / movementDuration);
                  state.getPreviewTrajectoryAngularVelocity().setValue(rotationalVelocity);

                  if (state.getIsNextForExecution())
                  {
                     // These calculations are used for both preview and execution
                     calculateTrajectoryTimesAndVelocities();

                     // Calculate preview for operator
                     ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
                     armIKSolver.copySourceToWork(); // Initialize the command, since we're not going to be far.

                     poseTrajectoryGenerator.clear(ReferenceFrame.getWorldFrame());
                     for (int i = 0; i < numberOfPoints; i++)
                     {
                        currentPose.set(state.getPreviewTrajectory().getValueReadOnly(i));
                        poseTrajectoryGenerator.appendPoseWaypoint(trajectoryTimes.get(i), currentPose, linearVelocities.get(i), angularVelocities.get(i));
                     }

                     poseTrajectoryGenerator.initialize();
                     poseTrajectoryGenerator.compute(movementDuration * state.getPreviewRequestedTime().getValue());

                     currentPoseFrame.getTransformToParent().set(poseTrajectoryGenerator.getPose());
                     currentPoseFrame.getReferenceFrame().update();
                     linearVelocity.set(poseTrajectoryGenerator.getVelocity());
                     angularVelocity.set(poseTrajectoryGenerator.getAngularVelocity());

                     armIKSolver.update(syncedRobot.getReferenceFrames().getChestFrame(), currentPoseFrame.getReferenceFrame());
                     armIKSolver.solve(angularVelocity, linearVelocity);

                     // Send the solution back to the UI so the user knows what's gonna happen with the arm.
                     state.getPreviewSolutionQuality().setValue(armIKSolver.getQuality());
                     for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
                     {
                        state.getPreviewJointAngles().getValue()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
                     }
                  }
               }
            }
         }
      }
   }

   private void calculateTrajectoryTimesAndVelocities()
   {
      ReferenceFrame screwAxisFrame = state.getScrewFrame().getReferenceFrame();

      angularVelocities.clear();
      linearVelocities.clear();
      trajectoryTimes.resetQuick();

      // Set the initial velocity and time as zero
      angularVelocities.add().setToZero();
      linearVelocities.add().setToZero();
      trajectoryTimes.add(0.0);

      double time = 2.0 * segmentDuration; // Make the first segment twice as long to allow smooth acceleration
      for (int i = 1; i < numberOfPoints - 1; i++)
      {
         Pose3DReadOnly waypointPose = state.getPreviewTrajectory().getValueReadOnly(i);

         // Get those pose in the screw axis frame
         workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), waypointPose);
         workPose.changeFrame(screwAxisFrame);

         // Get the vector that is tangent to the rotation
         Vector3DReadOnly rotationAxis = Axis3D.X;
         Vector3D radialPosition = new Vector3D(workPose.getPosition());
         radialPosition.setX(0.0);
         Vector3D tangentVector = new Vector3D();
         tangentVector.cross(rotationAxis, radialPosition);
         tangentVector.normalize();

         FrameVector3D linearVelocity = linearVelocities.add();
         linearVelocity.setToZero(screwAxisFrame);
         // Set the tangent velocity in the linear velocity
         linearVelocity.setAndScale(tangentialVelocity, tangentVector);
         // Set the axial velocity
         linearVelocity.setX(axialVelocity);
         linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

         // Set the angular velocity
         FrameVector3D angularVelocity = angularVelocities.add();
         angularVelocity.setToZero(screwAxisFrame);
         angularVelocity.setX(rotationalVelocity);
         angularVelocity.changeFrame(ReferenceFrame.getWorldFrame());

         // Add the time into the array
         trajectoryTimes.add(time);
         time += segmentDuration;
      }

      // Set the final velocity as zeros and time as the duration
      angularVelocities.add().setToZero();
      linearVelocities.add().setToZero();
      trajectoryTimes.add(movementDuration); // Makes the last segment twice as long to allow for smooth deceleration
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      // Fail if invalid
      if (Double.isNaN(state.getPreviewTrajectoryLinearVelocity().getValue())
       || Double.isNaN(state.getPreviewTrajectoryAngularVelocity().getValue()))
      {
         state.setFailed(true);
         LogTools.error("Cannot execute screw primitive with velocities:   Velocity %.2f m/s  %.2f %s/s"
                              .formatted(state.getPreviewTrajectoryLinearVelocity().getValue(),
                                         state.getPreviewTrajectoryAngularVelocity().getValue(),
                                         EuclidCoreMissingTools.DEGREE_SYMBOL));
         return;
      }

      // Compute smooth trajectory and publish command
      if (state.getScrewFrame().isChildOfWorld())
      {
         state.getCommandedTrajectory().getValue().clear();

         jointspaceOnlyTrajectoryMessage.setRobotSide(definition.getSide().toByte());
         jointspaceOnlyTrajectoryMessage.setForceExecution(true);
         handHybridTrajectoryMessage.setRobotSide(definition.getSide().toByte());
         handHybridTrajectoryMessage.setForceExecution(true);

         JointspaceTrajectoryMessage jointspaceTrajectoryMessage = handHybridTrajectoryMessage.getJointspaceTrajectoryMessage();
         jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().clear();

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

         // Start setting up the joint space trajectory
         for (int jointNumber = 0; jointNumber < numberOfJoints; jointNumber++)
         {
            OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
            oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
            oneDoFJointTrajectoryMessage.setWeight(definition.getJointspaceWeight());
         }

         ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
         armIKSolver.copySourceToWork(); // Initialize the command, since we're not going to be far.

         for (int i = 0; i < numberOfPoints; i++)
         {
            // For the first point, they are the same -- the initial hand pose. For the last point, they're the same.
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
               LogTools.warn("Bad quality: {} (i == {})", armIKSolver.getQuality(), i);

            for (int j = 0; j < numberOfJoints; j++)
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

         // Perform smoothing from the jointspace velocities
         for (int i = 1; i < numberOfPoints - 1; i++)
         {
            for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
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
            LogTools.info("Commanding %.3f s jointspace only trajectory with %d points".formatted(movementDuration, numberOfPoints));
            jointspaceOnlyTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
            ros2ControllerHelper.publishToController(jointspaceOnlyTrajectoryMessage);
         }
         else
         {
            LogTools.info("Commanding %.3f s hybrid trajectory with %d points".formatted(movementDuration, numberOfPoints));
            ros2ControllerHelper.publishToController(handHybridTrajectoryMessage);
         }

         trackingCalculator.reset();

         state.setNominalExecutionDuration(movementDuration);
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

      if (state.getScrewFrame().isChildOfWorld())
      {
         SE3TrajectoryPointReadOnly lastTrajectoryPose = state.getCommandedTrajectory().getLastValueReadOnly();
         desiredHandControlPose.set(lastTrajectoryPose.getPosition(), lastTrajectoryPose.getOrientation());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));

         trackingCalculator.computePoseTrackingData(desiredHandControlPose, syncedHandControlPose);
         trackingCalculator.factorInR3Errors(definition.getPositionErrorTolerance());
         trackingCalculator.factoryInSO3Errors(definition.getOrientationErrorTolerance());

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();

         state.getCurrentPose().getValue().set(syncedHandControlPose);
         state.setPositionDistanceToGoalTolerance(definition.getPositionErrorTolerance());
         state.setOrientationDistanceToGoalTolerance(definition.getOrientationErrorTolerance());
         state.getForce().getValue().set(syncedRobot.getHandWrenchCalculators().get(definition.getSide()).getFilteredWrench().getLinearPart());
         state.getTorque().getValue().set(syncedRobot.getHandWrenchCalculators().get(definition.getSide()).getFilteredWrench().getAngularPart());

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);
         }
      }
   }
}
