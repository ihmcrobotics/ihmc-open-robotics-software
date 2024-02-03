package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.*;
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
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
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
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final Quaternion localRotationQuaternion = new Quaternion();
   private final Vector3D worldRotationVector = new Vector3D();;
   private final Vector3D localRotationVectorEnd = new Vector3D();
   private final FramePoint3D frameRotationVectorEnd = new FramePoint3D();
   private final MutableReferenceFrame previousPoseFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame currentPoseFrame = new MutableReferenceFrame();
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();

   private final PoseReferenceFrame handFrame = new PoseReferenceFrame("screwHandControlFrame", ReferenceFrame.getWorldFrame());
   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private final DampedLeastSquaresSolver leastSquaresSolver = new DampedLeastSquaresSolver(7);

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
      // This is the damping factor for the damped pseudo-inverse
      leastSquaresSolver.setAlpha(1e-4);

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
                  double rotationPerPoint = Math.toRadians(8.6);
                  double translationPerPoint = 0.01;
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
                     Pose3D lastPose = trajectoryPoses.getLast();
                     Pose3D nextPose = trajectoryPoses.add();

                     workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), lastPose);
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

      if (getState().getScrewFrame().isChildOfWorld())
      {
         getState().getDesiredTrajectory().getValue().clear();

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
         double time = 0.0;

         for (int i = 0; i < numberOfPoints; i++)
         {
            boolean isLastPoint = i == numberOfPoints - 1;

            // For the first point, they are the same -- the initial hand pose. For the last point, they're the same.
            Pose3DReadOnly previousPose = getState().getTrajectory().getValueReadOnly(i == 0 ? i : i - 1);
            Pose3DReadOnly currentPose = getState().getTrajectory().getValueReadOnly(i);
            Pose3DReadOnly nextPose = getState().getTrajectory().getValueReadOnly(isLastPoint ? i : i + 1);

            previousPoseFrame.getTransformToParent().set(previousPose);
            previousPoseFrame.getReferenceFrame().update();

            currentPoseFrame.getTransformToParent().set(currentPose);
            currentPoseFrame.getReferenceFrame().update();

            double deltaTime = 0.0;
            if (i > 0)
            {
               // Find whether linear or angular would take longer at max speed and slow the other down
               double linearDistanceFromPrevious = currentPose.getPosition().distance(previousPose.getPosition());
               double angularDistanceFromPrevious = currentPose.getOrientation().distance(previousPose.getOrientation());
               double linearDistanceToNext = currentPose.getPosition().distance(nextPose.getPosition());
               double angularDistanceToNext = currentPose.getOrientation().distance(nextPose.getOrientation());

               double timeForLinear = (linearDistanceFromPrevious + linearDistanceToNext) / (2.0 * getDefinition().getMaxLinearVelocity());
               double timeForAngular = (angularDistanceFromPrevious + angularDistanceToNext) / (2.0 * getDefinition().getMaxAngularVelocity());
               deltaTime = Math.max(timeForLinear, timeForAngular); // Take longest
               time += deltaTime;

               linearVelocity.sub(nextPose.getPosition(), previousPose.getPosition());
               linearVelocity.scale(1.0 / deltaTime);

               localRotationQuaternion.difference(nextPose.getOrientation(), currentPose.getOrientation());
               localRotationQuaternion.getRotationVector(localRotationVectorEnd);
               frameRotationVectorEnd.setIncludingFrame(currentPoseFrame.getReferenceFrame(), localRotationVectorEnd);
               frameRotationVectorEnd.changeFrame(ReferenceFrame.getWorldFrame());
               worldRotationVector.sub(frameRotationVectorEnd, currentPose.getTranslation());

               angularVelocity.set(worldRotationVector);
               angularVelocity.normalize();
               angularVelocity.scale(1.0 / deltaTime);
            }

            if (i == 0 || isLastPoint)
            {
               linearVelocity.setToZero();
               angularVelocity.setToZero();
            }

            ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());
            if (i == 0)
               armIKSolver.copySourceToWork();
            armIKSolver.update(syncedRobot.getReferenceFrames().getChestFrame(), currentPoseFrame.getReferenceFrame());
            armIKSolver.solve();

            if (armIKSolver.getQuality() > ArmIKSolver.GOOD_QUALITY_MAX)
               LogTools.warn("Bad quality: {} (i == {})", armIKSolver.getQuality(), i);

            // Compute the velocities of the arm joints using the hand spatial velocities
            // TODO this only needs to be done if i is not the boundary conditions
            armIKSolver.getControlFrame(handFrame);
            jacobianCalculator.clear();
            jacobianCalculator.setKinematicChain(armIKSolver.getWorkingChest(), armIKSolver.getWorkingHand());
            jacobianCalculator.setJacobianFrame(handFrame);
            jacobianCalculator.reset();

            int joints = armIKSolver.getSolutionOneDoFJoints().length;
            DMatrixRMaj velocityObjective = new DMatrixRMaj(6, 1);
            DMatrixRMaj jointVelocities = new DMatrixRMaj(joints, 1);
            angularVelocity.changeFrame(handFrame);
            linearVelocity.changeFrame(handFrame);
            angularVelocity.get(velocityObjective);
            linearVelocity.get(3, velocityObjective);
            angularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
            linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

            leastSquaresSolver.setA(jacobianCalculator.getJacobianMatrix());
            leastSquaresSolver.solve(velocityObjective, jointVelocities);

            for (int j = 0; j < joints; j++)
            {
               OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = i == 0 ? jointspaceTrajectoryMessage.getJointTrajectoryMessages().add()
                                                                                  : jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(j);
               if (i == 0)
                  oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
               oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

               TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
               trajectoryPoint1DMessage.setTime(time);
               trajectoryPoint1DMessage.setPosition(armIKSolver.getSolutionOneDoFJoints()[j].getQ());
               if (i == 0 || isLastPoint)
               {
                  trajectoryPoint1DMessage.setVelocity(0.0);
               }
               else
               {
                  trajectoryPoint1DMessage.setVelocity(jointVelocities.get(i, 0));
               }
            }

            SE3TrajectoryPointMessage se3TrajectoryPointMessage = taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().add();
            se3TrajectoryPointMessage.setTime(time);
            se3TrajectoryPointMessage.getPosition().set(currentPose.getTranslation());
            se3TrajectoryPointMessage.getOrientation().set(currentPose.getOrientation());
            se3TrajectoryPointMessage.getLinearVelocity().set(linearVelocity);
            se3TrajectoryPointMessage.getAngularVelocity().set(angularVelocity);

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

         SE3TrajectoryPointReadOnly lastTrajectoryPose = getState().getDesiredTrajectory().getLastValueReadOnly();
         getState().setNominalExecutionDuration(lastTrajectoryPose.getTime());
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
