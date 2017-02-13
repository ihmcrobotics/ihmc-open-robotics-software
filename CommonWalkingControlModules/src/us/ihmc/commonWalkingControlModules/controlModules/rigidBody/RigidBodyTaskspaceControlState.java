package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.tools.io.printing.PrintTools;

public class RigidBodyTaskspaceControlState extends RigidBodyControlState
{
   private static final int maxPoints = 1000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final String warningPrefix;

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Twist.SIZE, Twist.SIZE);

   private final YoOrientationPIDGainsInterface orientationGains;
   private final YoPositionPIDGainsInterface positionGains;
   private final YoFrameVector yoAngularWeight;
   private final YoFrameVector yoLinearWeight;
   private final Vector3d angularWeight = new Vector3d();
   private final Vector3d linearWeight = new Vector3d();

   private final BooleanYoVariable trajectoryStopped;
   private final BooleanYoVariable trackingOrientation;
   private final BooleanYoVariable trackingPosition;

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardLinearAcceleration = new FrameVector(worldFrame);
   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(worldFrame);

   private final RecyclingArrayList<FrameSE3TrajectoryPoint> pointQueue = new RecyclingArrayList<>(maxPoints, FrameSE3TrajectoryPoint.class);

   public RigidBodyTaskspaceControlState(RigidBody bodyToControl, RigidBody rootBody, RigidBody elevator, YoOrientationPIDGainsInterface orientationGains,
         YoPositionPIDGainsInterface positionGains, Map<BaseForControl, ReferenceFrame> controlFrameMap, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE);
      this.orientationGains = orientationGains;
      this.positionGains = positionGains;

      String bodyName = bodyToControl.getName();
      registry = new YoVariableRegistry(bodyName + "TaskspaceControlModule");

      String prefix = bodyName + "Taskspace";
      warningPrefix = getClass().getSimpleName() + " for " + bodyName + ": ";
      trajectoryStopped = new BooleanYoVariable(prefix + "TrajectoryStopped", registry);
      trackingOrientation = new BooleanYoVariable(prefix + "TrackingOrientation", registry);
      trackingPosition = new BooleanYoVariable(prefix + "TrackingPosition", registry);

      spatialFeedbackControlCommand.set(elevator, bodyToControl);
      spatialFeedbackControlCommand.setPrimaryBase(rootBody);
      spatialFeedbackControlCommand.setSelectionMatrixToIdentity();
      privilegedConfigurationCommand.applyPrivilegedConfigurationToSubChain(rootBody, bodyToControl);

      yoAngularWeight = new YoFrameVector(prefix + "AngularWeight", null, registry);
      yoLinearWeight = new YoFrameVector(prefix + "LinearWeight", null, registry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(prefix, true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(prefix, true, worldFrame, registry);

      for (ReferenceFrame frameToRegister : controlFrameMap.values())
      {
         positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
      }

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3d angularWeight, Vector3d linearWeight)
   {
      if (angularWeight != null)
         yoAngularWeight.set(angularWeight);
      else
         yoAngularWeight.setToNaN();

      if (linearWeight != null)
         yoLinearWeight.set(linearWeight);
      else
         yoLinearWeight.setToNaN();
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      trajectoryStopped.set(command.isStopAllTrajectory());
   }

   @Override
   public void doAction()
   {
      positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      spatialFeedbackControlCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      spatialFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      if (orientationGains != null)
         spatialFeedbackControlCommand.setGains(orientationGains);
      if (positionGains != null)
         spatialFeedbackControlCommand.setGains(positionGains);
      yoAngularWeight.get(angularWeight);
      yoLinearWeight.get(linearWeight);
      spatialFeedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);
   }

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
      trackingOrientation.set(false);
      trackingPosition.set(false);
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> command, FrameOrientation initialOrientation)
   {
      if (!checkOrientationGainsAndWeights())
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking position. Can not queue orientation trajectory.");
         return false;
      }

      if (override)
         clear();

      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!queuePoint(command.getTrajectoryPoint(i)))
            break;
      }

      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      for (int i = 0; i < 3; i++)
         MatrixTools.removeRow(selectionMatrix, 0);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);

      trajectoryStopped.set(false);
      trackingOrientation.set(true);
      trackingPosition.set(false);
      return true;
   }

   public boolean handlePoseTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command, FramePose initialPose)
   {
      if (!checkPoseGainsAndWeights())
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (!trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override)
         clear();

      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!queuePoint(command.getTrajectoryPoint(i)))
            break;
      }

      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);

      trajectoryStopped.set(false);
      trackingOrientation.set(true);
      trackingPosition.set(true);
      return true;
   }

   public void getDesiredPose(FramePose desiredPoseToPack)
   {
      orientationTrajectoryGenerator.getOrientation(desiredOrientation);
      positionTrajectoryGenerator.getPosition(desiredPosition);
      desiredPoseToPack.setPoseIncludingFrame(desiredPosition, desiredOrientation);
   }

   public void getDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(desiredOrientationToPack);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedConfigurationCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

   private boolean queuePoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      pointQueue.add().setIncludingFrame(trajectoryPoint);
      return true;
   }

   private boolean queuePoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      FrameSE3TrajectoryPoint point = pointQueue.add();
      trajectoryPoint.getOrientation(desiredOrientation);
      trajectoryPoint.getAngularVelocity(desiredAngularVelocity);
      point.setToZero(trajectoryPoint.getReferenceFrame());
      point.setOrientation(desiredOrientation);
      point.setAngularVelocity(desiredAngularVelocity);
      point.setTime(trajectoryPoint.getTime());
      return true;
   }

   private boolean atCapacityLimit()
   {
      if (pointQueue.size() == maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + maxPoints + " discarding additional waypoints.");
         return true;
      }
      return false;
   }

   private void clear()
   {
      orientationTrajectoryGenerator.clear();
      positionTrajectoryGenerator.clear();
      pointQueue.clear();
   }

   private boolean checkPoseGainsAndWeights()
   {
      return checkOrientationGainsAndWeights() && checkPositionGainsAndWeights();
   }

   private boolean checkOrientationGainsAndWeights()
   {
      boolean success = true;
      if (yoAngularWeight.containsNaN())
      {
         PrintTools.warn(warningPrefix + "Orientation weights are NaN.");
         success = false;
      }
      if (orientationGains == null)
      {
         PrintTools.warn(warningPrefix + "Orientation gains are null.");
         success = false;
      }
      return success;
   }

   private boolean checkPositionGainsAndWeights()
   {
      boolean success = true;
      if (yoLinearWeight.containsNaN())
      {
         PrintTools.warn(warningPrefix + "Position weights are NaN.");
         success = false;
      }
      if (positionGains == null)
      {
         PrintTools.warn(warningPrefix + "Position gains are null.");
         success = false;
      }
      return success;
   }

}
