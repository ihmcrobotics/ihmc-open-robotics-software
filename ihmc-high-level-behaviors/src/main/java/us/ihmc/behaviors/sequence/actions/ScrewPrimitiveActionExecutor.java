package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.*;
import gnu.trove.list.array.TDoubleArrayList;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.linsol.svd.SolvePseudoInverseSvd_DDRM;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.TrajectoryTrackingErrorCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MotionQPInputCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.robotics.functionApproximation.DampedLeastSquaresSolver;
import us.ihmc.robotics.math.YoSolvePseudoInverseSVDWithDampedLeastSquaresNearSingularities;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointReadOnly;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionExecutor extends ActionNodeExecutor<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();
   private final FramePose3D workPose = new FramePose3D();
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();

   private final MutableReferenceFrame currentPoseFrame = new MutableReferenceFrame();
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final RecyclingArrayList<FrameVector3D> linearVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final RecyclingArrayList<FrameVector3D> angularVelocities = new RecyclingArrayList<>(FrameVector3D::new);
   private final TDoubleArrayList trajectoryTimes = new TDoubleArrayList();

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

      getState().setCanExecute(getState().getScrewFrame().isChildOfWorld());

      if (getState().getScrewFrame().isChildOfWorld())
      {
         if (getParent().getState() instanceof ActionSequenceState parent)
         {
            if (parent.getExecutionNextIndex() <= getState().getActionIndex())
            {
               ReferenceFrame initialHandFrame = null;

               if (getState().getIsNextForExecution())
               {
                  initialHandFrame = syncedRobot.getReferenceFrames().getHandFrame(getDefinition().getSide());
               }
               else
               {
                  HandPoseActionState previousHandPose = parent.findNextPreviousAction(HandPoseActionState.class,
                                                                                       getState().getActionIndex(),
                                                                                       getDefinition().getSide());
                  if (previousHandPose != null && previousHandPose.getPalmFrame().isChildOfWorld())
                  {
                     initialHandFrame = previousHandPose.getPalmFrame().getReferenceFrame();
                  }
               }

               if (initialHandFrame != null)
               {
                  RecyclingArrayList<Pose3D> trajectoryPoses = getState().getTrajectory().getValue();
                  trajectoryPoses.clear();
                  Pose3D firstPose = trajectoryPoses.add();
                  workPose.setToZero(initialHandFrame);
                  workPose.changeFrame(ReferenceFrame.getWorldFrame());
                  firstPose.set(workPose);

                  // These contants could be adjusted
                  double rotationPerPoint = Math.toRadians(10);
                  double translationPerPoint = 0.05;
                  int segments = (int) Math.ceil(Math.abs(getDefinition().getRotation()) / rotationPerPoint
                                               + Math.abs(getDefinition().getTranslation()) / translationPerPoint);

                  double rotationPerSegment = getDefinition().getRotation() / segments;
                  double translationPerSegment = getDefinition().getTranslation() / segments;

                  if (segments > ScrewPrimitiveActionState.TRAJECTORY_SIZE_LIMIT - 1)
                  {
                     segments = ScrewPrimitiveActionState.TRAJECTORY_SIZE_LIMIT - 1; // We have to fit within the message size limit
                  }

                  for (int i = 0; i < segments; i++)
                  {
                     Pose3D previousPose = trajectoryPoses.getLast();
                     Pose3D nextPose = trajectoryPoses.add();

                     workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), previousPose);
                     workPose.changeFrame(getState().getScrewFrame().getReferenceFrame());

                     workPose.prependRollRotation(rotationPerSegment);
                     workPose.prependTranslation(translationPerSegment, 0.0, 0.0);

                     workPose.changeFrame(ReferenceFrame.getWorldFrame());
                     nextPose.set(workPose);
                  }
               }
            }
         }
      }
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      LogTools.info("Triggered screw execution.");

      if (getState().getScrewFrame().isChildOfWorld())
      {
         getState().getDesiredTrajectory().getValue().clear();

         ReferenceFrame affordanceFrame = getState().getScrewFrame().getReferenceFrame();

         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));

         handHybridTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         handHybridTrajectoryMessage.setForceExecution(true);

         JointspaceTrajectoryMessage jointspaceTrajectoryMessage = handHybridTrajectoryMessage.getJointspaceTrajectoryMessage();
         jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().clear();

         SE3TrajectoryMessage taskspaceTrajectoryMessage = handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage();
         taskspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         taskspaceTrajectoryMessage.getLinearWeightMatrix().setXWeight(getDefinition().getLinearPositionWeight());
         taskspaceTrajectoryMessage.getLinearWeightMatrix().setYWeight(getDefinition().getLinearPositionWeight());
         taskspaceTrajectoryMessage.getLinearWeightMatrix().setZWeight(getDefinition().getLinearPositionWeight());
         taskspaceTrajectoryMessage.getAngularWeightMatrix().setXWeight(getDefinition().getAngularPositionWeight());
         taskspaceTrajectoryMessage.getAngularWeightMatrix().setYWeight(getDefinition().getAngularPositionWeight());
         taskspaceTrajectoryMessage.getAngularWeightMatrix().setZWeight(getDefinition().getAngularPositionWeight());
         taskspaceTrajectoryMessage.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         taskspaceTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().clear();

         int numberOfPoints = getState().getTrajectory().getSize();

         syncedHandControlPose.changeFrame(affordanceFrame);

         double totalRotationInRadians = getDefinition().getRotation();
         double rotationRadius = EuclidCoreTools.norm(syncedHandControlPose.getY(), syncedHandControlPose.getZ()); // this is always the radial distance.
         double totalTranslation = getDefinition().getTranslation();
         // this is the distance the hand must travel along the screw portion
         double radialDistance = totalRotationInRadians * rotationRadius;
         double totalLinearDistanceOfHand = EuclidCoreTools.norm(radialDistance, totalTranslation);

         // computing the movement duration, which is clamped by the max movement speed
         double durationForRotation = totalRotationInRadians / getDefinition().getMaxAngularVelocity();
         double durationForTranslation = totalLinearDistanceOfHand / getDefinition().getMaxLinearVelocity();
         double movementDuration = Math.max(durationForRotation, durationForTranslation);
         double segmentDuration = movementDuration / (numberOfPoints - 1);
         movementDuration += 2.0 * segmentDuration;

         // the way the screw frame is defined, x is always the axis of rotation and translation. This means that the tangential velocity is normal to the x axis and the vector yz
         double tangentialVelocity = radialDistance / movementDuration;
         double axialVelocity = totalTranslation / movementDuration;
         double rotationalVelocity = totalRotationInRadians / movementDuration;

         angularVelocities.clear();
         linearVelocities.clear();
         trajectoryTimes.reset();
         // set the initial velocity and time as zero
         angularVelocities.add().setToZero();
         linearVelocities.add().setToZero();
         trajectoryTimes.add(0.0);

         double time = 2.0 * segmentDuration; // make the first segment twice as long to allow smooth acceleration
         for (int i = 1; i < numberOfPoints - 1; i++)
         {
            Pose3DReadOnly waypointPose = getState().getTrajectory().getValueReadOnly(i);

            // get those pose in the frame of the affordance
            workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), waypointPose);
            workPose.changeFrame(affordanceFrame);

            // get the vector that is tangent to the rotation
            Vector3DReadOnly rotationAxis = Axis3D.X;
            Vector3D radialPosition = new Vector3D(workPose.getPosition());
            radialPosition.setX(0.0);
            Vector3D tangentVector = new Vector3D();
            tangentVector.cross(rotationAxis, radialPosition);
            tangentVector.normalize();

            FrameVector3D linearVelocity = linearVelocities.add();
            linearVelocity.setToZero(affordanceFrame);
            // set the tangent velocity in the linear velocity
            linearVelocity.setAndScale(tangentialVelocity, tangentVector);
            // set the axial velocity
            linearVelocity.setX(axialVelocity);
            linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

            // set the angular velocity
            FrameVector3D angularVelocity = angularVelocities.add();
            angularVelocity.setToZero(affordanceFrame);
            angularVelocity.setX(rotationalVelocity);
            angularVelocity.changeFrame(ReferenceFrame.getWorldFrame());

            // add the time into the array
            trajectoryTimes.add(time);
            time += segmentDuration;
         }

         // set the final velocity as zeros and time as the duration
         angularVelocities.add().setToZero();
         linearVelocities.add().setToZero();
         trajectoryTimes.add(movementDuration); // makes the last segment twice as long to allow for smooth deceleration

         // start setting up the joint space trajectory
         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());
         int numberOfJoints = armIKSolver.getSolutionOneDoFJoints().length;
         for (int jointNumber = 0; jointNumber < numberOfJoints; jointNumber++)
         {
            OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = jointspaceTrajectoryMessage.getJointTrajectoryMessages().add();
            oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
            oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use Default weight
         }

         // initialize the command, since we're not going to be far.
         armIKSolver.copySourceToWork();

         for (int i = 0; i < numberOfPoints; i++)
         {
            // For the first point, they are the same -- the initial hand pose. For the last point, they're the same.
            Pose3DReadOnly currentPose = getState().getTrajectory().getValueReadOnly(i);
            angularVelocity.setIncludingFrame(angularVelocities.get(i));
            linearVelocity.setIncludingFrame(linearVelocities.get(i));
            double waypointTime = trajectoryTimes.get(i);

            currentPoseFrame.getTransformToParent().set(currentPose);
            currentPoseFrame.getReferenceFrame().update();

            SE3TrajectoryPointMessage se3TrajectoryPointMessage = taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().add();
            se3TrajectoryPointMessage.setTime(waypointTime);
            se3TrajectoryPointMessage.getPosition().set(currentPose.getTranslation());
            se3TrajectoryPointMessage.getOrientation().set(currentPose.getOrientation());
            se3TrajectoryPointMessage.getLinearVelocity().set(linearVelocity);
            se3TrajectoryPointMessage.getAngularVelocity().set(angularVelocity);

            angularVelocity.changeFrame(currentPoseFrame.getReferenceFrame());
            linearVelocity.changeFrame(currentPoseFrame.getReferenceFrame());

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

            angularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
            linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

            LogTools.info("Adding point time: %.2f  nextPose: %s %s  linearVel: %s  angularVel: %s"
                    .formatted(time,
                               currentPose.getPosition(),
                               new YawPitchRoll(currentPose.getOrientation()),
                               linearVelocity,
                               angularVelocity));

            getState().getDesiredTrajectory().addTrajectoryPoint(currentPose, time);
         }
         ros2ControllerHelper.publishToController(handHybridTrajectoryMessage);

         trackingCalculator.reset();

         getState().setNominalExecutionDuration(movementDuration);
         LogTools.info("Sending trajectory %f long with %d points", movementDuration, numberOfPoints);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(getState().getNominalExecutionDuration());
      getState().setElapsedExecutionTime(trackingCalculator.getElapsedTime());

      if (trackingCalculator.getHitTimeLimit())
      {
         getState().setIsExecuting(false);
         getState().setFailed(true);
         LogTools.error("Task execution timed out. Publishing stop all trajectories message.");
         ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
         return;
      }

      if (getState().getScrewFrame().isChildOfWorld())
      {
         SE3TrajectoryPointReadOnly lastTrajectoryPose = getState().getDesiredTrajectory().getLastValueReadOnly();
         desiredHandControlPose.set(lastTrajectoryPose.getPosition(), lastTrajectoryPose.getOrientation());
         syncedHandControlPose.setToZero(ReferenceFrame.getWorldFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));

         trackingCalculator.computePoseTrackingData(desiredHandControlPose, syncedHandControlPose);
         trackingCalculator.factorInR3Errors(POSITION_TOLERANCE);
         trackingCalculator.factoryInSO3Errors(ORIENTATION_TOLERANCE);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();

         getState().getCurrentPose().getValue().set(syncedHandControlPose);
         getState().setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         getState().setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
         getState().getForce().getValue().set(syncedRobot.getHandWrenchCalculators().get(getDefinition().getSide()).getFilteredWrench().getLinearPart());
         getState().getTorque().getValue().set(syncedRobot.getHandWrenchCalculators().get(getDefinition().getSide()).getFilteredWrench().getAngularPart());

         if (meetsDesiredCompletionCriteria)
         {
            getState().setIsExecuting(false);
         }
      }
   }
}
