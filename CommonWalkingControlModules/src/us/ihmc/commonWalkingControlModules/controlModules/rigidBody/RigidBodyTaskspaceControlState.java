package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
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
   public static final int maxPoints = 200;
   public static final int maxPointsInGenerator = 5;
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
   private final BooleanYoVariable trajectoryDone;
   private final BooleanYoVariable trackingOrientation;
   private final BooleanYoVariable trackingPosition;

   private final IntegerYoVariable numberOfPointsInQueue;
   private final IntegerYoVariable numberOfPointsInGenerator;
   private final IntegerYoVariable numberOfPoints;

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardLinearAcceleration = new FrameVector(worldFrame);
   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(worldFrame);

   private final RecyclingArrayList<FrameSE3TrajectoryPoint> pointQueue = new RecyclingArrayList<>(maxPoints, FrameSE3TrajectoryPoint.class);
   private final FrameSE3TrajectoryPoint lastPointAdded = new FrameSE3TrajectoryPoint();
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable trajectoryStartTime;
   private final LongYoVariable lastCommandId;

   private final ReferenceFrame rootFrame;

   public RigidBodyTaskspaceControlState(RigidBody bodyToControl, RigidBody rootBody, RigidBody elevator, YoOrientationPIDGainsInterface orientationGains,
         YoPositionPIDGainsInterface positionGains, Map<BaseForControl, ReferenceFrame> controlFrameMap, ReferenceFrame rootFrame, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE);
      this.orientationGains = orientationGains;
      this.positionGains = positionGains;
      this.yoTime = yoTime;
      this.rootFrame = rootFrame;

      String bodyName = bodyToControl.getName();
      registry = new YoVariableRegistry(bodyName + "TaskspaceControlModule");

      String prefix = bodyName + "Taskspace";
      warningPrefix = getClass().getSimpleName() + " for " + bodyName + ": ";
      trajectoryStopped = new BooleanYoVariable(prefix + "TrajectoryStopped", registry);
      trajectoryDone = new BooleanYoVariable(prefix + "TrajectoryDone", registry);
      trackingOrientation = new BooleanYoVariable(prefix + "TrackingOrientation", registry);
      trackingPosition = new BooleanYoVariable(prefix + "TrackingPosition", registry);
      trajectoryStartTime = new DoubleYoVariable(prefix + "TrajectoryStartTime", registry);

      numberOfPointsInQueue = new IntegerYoVariable(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new IntegerYoVariable(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new IntegerYoVariable(prefix + "NumberOfPoints", registry);

      lastCommandId = new LongYoVariable(prefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      spatialFeedbackControlCommand.set(elevator, bodyToControl);
      spatialFeedbackControlCommand.setPrimaryBase(rootBody);
      spatialFeedbackControlCommand.setSelectionMatrixToIdentity();
      privilegedConfigurationCommand.applyPrivilegedConfigurationToSubChain(rootBody, bodyToControl);

      yoAngularWeight = new YoFrameVector(prefix + "AngularWeight", null, registry);
      yoLinearWeight = new YoFrameVector(prefix + "LinearWeight", null, registry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry);

      for (ReferenceFrame frameToRegister : controlFrameMap.values())
      {
         positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
      }
      positionTrajectoryGenerator.registerNewTrajectoryFrame(rootFrame);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(rootFrame);

      pointQueue.clear();
      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3d angularWeight, Vector3d linearWeight)
   {
      if (angularWeight != null)
         yoAngularWeight.set(angularWeight);
      else
         yoAngularWeight.setToZero();

      if (linearWeight != null)
         yoLinearWeight.set(linearWeight);
      else
         yoLinearWeight.setToZero();
   }

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      trajectoryStopped.set(command.isStopAllTrajectory());
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = yoTime.getDoubleValue() - trajectoryStartTime.getDoubleValue();
      if (!trajectoryDone.getBooleanValue() && orientationTrajectoryGenerator.isDone())
         fillAndReinitializeTrajectories();

      if (!trajectoryStopped.getBooleanValue())
      {
         positionTrajectoryGenerator.compute(timeInTrajectory);
         orientationTrajectoryGenerator.compute(timeInTrajectory);
      }

      positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);

      if (trajectoryStopped.getBooleanValue())
      {
         desiredLinearVelocity.setToZero(worldFrame);
         feedForwardLinearAcceleration.setToZero(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);
         feedForwardAngularAcceleration.setToZero(worldFrame);
      }

      spatialFeedbackControlCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      spatialFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      if (orientationGains != null)
         spatialFeedbackControlCommand.setGains(orientationGains);
      if (positionGains != null)
         spatialFeedbackControlCommand.setGains(positionGains);
      yoAngularWeight.get(angularWeight);
      yoLinearWeight.get(linearWeight);
      spatialFeedbackControlCommand.setSelectionMatrix(selectionMatrix);
      spatialFeedbackControlCommand.setWeightsForSolver(angularWeight, linearWeight);

      numberOfPointsInQueue.set(pointQueue.size());
      numberOfPointsInGenerator.set(orientationTrajectoryGenerator.getCurrentNumberOfWaypoints());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());
   }

   private void fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         trajectoryDone.set(true);
         positionTrajectoryGenerator.changeFrame(rootFrame);
         orientationTrajectoryGenerator.changeFrame(rootFrame);
         return;
      }

      if (!orientationTrajectoryGenerator.isEmpty())
      {
         positionTrajectoryGenerator.clear(worldFrame);
         orientationTrajectoryGenerator.clear(worldFrame);
         lastPointAdded.changeFrame(worldFrame);
         positionTrajectoryGenerator.appendWaypoint(lastPointAdded);
         orientationTrajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      positionTrajectoryGenerator.changeFrame(worldFrame);
      orientationTrajectoryGenerator.changeFrame(worldFrame);

      int currentNumberOfWaypoints = orientationTrajectoryGenerator.getCurrentNumberOfWaypoints();
      int pointsToAdd = maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         FrameSE3TrajectoryPoint pointToAdd = pointQueue.get(0);
         lastPointAdded.setIncludingFrame(pointToAdd); // TODO: get from generators
         positionTrajectoryGenerator.appendWaypoint(pointToAdd);
         orientationTrajectoryGenerator.appendWaypoint(pointToAdd);
         pointQueue.remove(0); // TODO: replace with queue
      }

      lastPointAdded.changeFrame(rootFrame);
      positionTrajectoryGenerator.changeFrame(rootFrame);
      orientationTrajectoryGenerator.changeFrame(rootFrame);
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
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

      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
   }

   public void holdOrientation(FrameOrientation initialOrientation)
   {
      overrideTrajectory(Packet.INVALID_MESSAGE_ID);
      queueInitialPoint(initialOrientation);

      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      for (int i = 0; i < 3; i++)
         MatrixTools.removeRow(selectionMatrix, 3);

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
      trackingOrientation.set(true);
      trackingPosition.set(false);
   }

   public void holdPose(FramePose initialPose)
   {
      overrideTrajectory(Packet.INVALID_MESSAGE_ID);
      queueInitialPoint(initialPose);

      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
      trackingOrientation.set(true);
      trackingPosition.set(true);
   }

   public boolean handleOrientationTrajectoryCommand(SO3TrajectoryControllerCommand<?, ?> command, FrameOrientation initialOrientation)
   {
      if (!checkOrientationGainsAndWeights())
         return false;

      if (command.getCommandId() == Packet.INVALID_MESSAGE_ID)
      {
         PrintTools.warn(warningPrefix + "Recieved packet with invalid ID.");
         return false;
      }
      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking pose. Can not queue orientation trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         overrideTrajectory(command.getCommandId());
         if (command.getTrajectoryPoint(0).getTime() > 0.0)
            queueInitialPoint(initialOrientation);
      }
      else
      {
         if (command.getPreviousCommandId() != lastCommandId.getLongValue())
         {
            PrintTools.warn(warningPrefix + "Unexpected command ID.");
            return false;
         }
         lastCommandId.set(command.getCommandId());
         command.addTimeOffset(getLastTrajectoryPointTime());
      }

      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);
      for (int i = 0; i < 3; i++)
         MatrixTools.removeRow(selectionMatrix, 3);

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
      trackingOrientation.set(true);
      trackingPosition.set(false);
      return true;
   }

   public boolean handlePoseTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command, FramePose initialPose)
   {
      if (!checkPoseGainsAndWeights())
         return false;

      if (command.getCommandId() == Packet.INVALID_MESSAGE_ID)
      {
         PrintTools.warn(warningPrefix + "Recieved packet with invalid ID.");
         return false;
      }

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (!trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         overrideTrajectory(command.getCommandId());
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
            queueInitialPoint(initialPose);
      }
      else
      {
         if (command.getPreviousCommandId() != lastCommandId.getLongValue())
         {
            PrintTools.warn(warningPrefix + "Unexpected command ID.");
            return false;
         }
         lastCommandId.set(command.getCommandId());
         command.addTimeOffset(getLastTrajectoryPointTime());
      }

      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      selectionMatrix.reshape(Twist.SIZE, Twist.SIZE);
      CommonOps.setIdentity(selectionMatrix);

      trajectoryStopped.set(false);
      trajectoryDone.set(false);
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

   private boolean checkTime(double time)
   {
      boolean timeValid = time > getLastTrajectoryPointTime();
      if (!timeValid)
         PrintTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
      return timeValid;
   }

   private double getLastTrajectoryPointTime()
   {
      if (isEmpty())
      {
         return Double.NEGATIVE_INFINITY;
      }
      else if (pointQueue.isEmpty())
      {
         return orientationTrajectoryGenerator.getLastWaypointTime();
      }
      else
      {
         return pointQueue.getLast().getTime();
      }
   }

   private boolean isEmpty()
   {
      return pointQueue.isEmpty() && orientationTrajectoryGenerator.isDone();
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

   private void queueInitialPoint(FramePose initialPose)
   {
      FrameSE3TrajectoryPoint point = pointQueue.add();
      point.setToZero(initialPose.getReferenceFrame());
      point.setTime(0.0);
      initialPose.getPoseIncludingFrame(desiredPosition, desiredOrientation);
      point.setPosition(desiredPosition);
      point.setOrientation(desiredOrientation);
   }

   private void queueInitialPoint(FrameOrientation initialOrientation)
   {
      FrameSE3TrajectoryPoint point = pointQueue.add();
      point.setToZero(initialOrientation.getReferenceFrame());
      point.setTime(0.0);
      point.setOrientation(initialOrientation);
   }

   private boolean atCapacityLimit()
   {
      if (pointQueue.size() >= maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + maxPoints + " can not execute trajectory.");
         return true;
      }
      return false;
   }

   private void overrideTrajectory(long commandId)
   {
      orientationTrajectoryGenerator.clear();
      positionTrajectoryGenerator.clear();
      pointQueue.clear();
      trajectoryStartTime.set(yoTime.getDoubleValue());
      lastCommandId.set(commandId);
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
