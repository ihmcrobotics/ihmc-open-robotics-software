package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
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

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Twist.SIZE, Twist.SIZE);

   private final YoOrientationPIDGainsInterface orientationGains;
   private final YoPositionPIDGainsInterface positionGains;
   private final YoFrameVector yoAngularWeight;
   private final YoFrameVector yoLinearWeight;
   private final Vector3D angularWeight = new Vector3D();
   private final Vector3D linearWeight = new Vector3D();

   private final BooleanYoVariable trackingOrientation;
   private final BooleanYoVariable trackingPosition;

   private final BooleanYoVariable hasOrientaionGains;
   private final BooleanYoVariable hasAngularWeight;
   private final BooleanYoVariable hasPositionGains;
   private final BooleanYoVariable hasLinearWeight;

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

   private final RecyclingArrayDeque<FrameSE3TrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(maxPoints, FrameSE3TrajectoryPoint.class);
   private final FrameSE3TrajectoryPoint lastPointAdded = new FrameSE3TrajectoryPoint();

   private final ReferenceFrame baseFrame;
   private ReferenceFrame trajectoryFrame;

   public RigidBodyTaskspaceControlState(String bodyName, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
         Map<BaseForControl, ReferenceFrame> controlFrameMap, ReferenceFrame baseFrame, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyName, yoTime);
      this.baseFrame = baseFrame;
      this.trajectoryFrame = baseFrame;

      String prefix = bodyName + "Taskspace";
      trackingOrientation = new BooleanYoVariable(prefix + "TrackingOrientation", registry);
      trackingPosition = new BooleanYoVariable(prefix + "TrackingPosition", registry);

      orientationGains = new YoSymmetricSE3PIDGains(prefix + "OrientationGains", registry);
      positionGains = new YoSymmetricSE3PIDGains(prefix + "PositionGains", registry);

      numberOfPointsInQueue = new IntegerYoVariable(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new IntegerYoVariable(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new IntegerYoVariable(prefix + "NumberOfPoints", registry);

      spatialFeedbackControlCommand.set(elevator, bodyToControl);
      spatialFeedbackControlCommand.setPrimaryBase(baseBody);
      spatialFeedbackControlCommand.setSelectionMatrixToIdentity();

      yoAngularWeight = new YoFrameVector(prefix + "AngularWeight", null, registry);
      yoLinearWeight = new YoFrameVector(prefix + "LinearWeight", null, registry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(bodyName, maxPointsInGenerator, true, worldFrame, registry);

      if (controlFrameMap != null)
      {
         for (ReferenceFrame frameToRegister : controlFrameMap.values())
         {
            positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
            orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         }
      }

      positionTrajectoryGenerator.registerNewTrajectoryFrame(baseFrame);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(baseFrame);

      hasOrientaionGains = new BooleanYoVariable(prefix + "HasOrientaionGains", registry);
      hasAngularWeight = new BooleanYoVariable(prefix + "HasAngularWeights", registry);
      hasPositionGains = new BooleanYoVariable(prefix + "HasPositionGains", registry);
      hasLinearWeight = new BooleanYoVariable(prefix + "HasLinearWeights", registry);

      pointQueue.clear();
      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3D angularWeight, Vector3D linearWeight)
   {
      if (angularWeight != null)
      {
         yoAngularWeight.set(angularWeight);
         hasAngularWeight.set(true);
      }
      else
      {
         yoAngularWeight.setToZero();
         hasAngularWeight.set(false);
      }

      if (linearWeight != null)
      {
         yoLinearWeight.set(linearWeight);
         hasLinearWeight.set(true);
      }
      else
      {
         yoLinearWeight.setToZero();
         hasLinearWeight.set(false);
      }
   }

   public void setWeight(double weight)
   {
      hasAngularWeight.set(true);
      yoAngularWeight.set(weight, weight, weight);
      hasLinearWeight.set(true);
      yoLinearWeight.set(weight, weight, weight);
   }

   public void setGains(YoOrientationPIDGainsInterface orientationGains, YoPositionPIDGainsInterface positionGains)
   {
      if (orientationGains != null)
      {
         this.orientationGains.set(orientationGains);
         hasOrientaionGains.set(true);
      }
      else
      {
         hasOrientaionGains.set(false);
      }

      if (positionGains != null)
      {
         this.positionGains.set(positionGains);
         hasPositionGains.set(true);
      }
      else
      {
         hasPositionGains.set(false);
      }
   }

   @Override
   public void doAction()
   {
      double timeInTrajectory = getTimeInTrajectory();
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
         desiredLinearVelocity.setToZero(baseFrame);
         feedForwardLinearAcceleration.setToZero(baseFrame);
         desiredAngularVelocity.setToZero(baseFrame);
         feedForwardAngularAcceleration.setToZero(baseFrame);
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
//         positionTrajectoryGenerator.changeFrame(trajectoryFrame);
//         orientationTrajectoryGenerator.changeFrame(trajectoryFrame);
         return;
      }

      if (!orientationTrajectoryGenerator.isEmpty())
      {
         positionTrajectoryGenerator.clear(trajectoryFrame);
         orientationTrajectoryGenerator.clear(trajectoryFrame);
         lastPointAdded.changeFrame(trajectoryFrame);
         positionTrajectoryGenerator.appendWaypoint(lastPointAdded);
         orientationTrajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      positionTrajectoryGenerator.changeFrame(trajectoryFrame);
      orientationTrajectoryGenerator.changeFrame(trajectoryFrame);

      int currentNumberOfWaypoints = orientationTrajectoryGenerator.getCurrentNumberOfWaypoints();
      int pointsToAdd = maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         FrameSE3TrajectoryPoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd); // TODO: get from generators
         positionTrajectoryGenerator.appendWaypoint(pointToAdd);
         orientationTrajectoryGenerator.appendWaypoint(pointToAdd);
      }

      lastPointAdded.changeFrame(trajectoryFrame);
//      positionTrajectoryGenerator.changeFrame(trajectoryFrame);
//      orientationTrajectoryGenerator.changeFrame(trajectoryFrame);
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
      overrideTrajectory();
      resetLastCommandId();
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
      overrideTrajectory();
      resetLastCommandId();
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

      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking pose. Can not queue orientation trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         overrideTrajectory();
         trajectoryFrame = command.getTrajectoryFrame();
         if (command.getTrajectoryPoint(0).getTime() > 0.0)
            queueInitialPoint(initialOrientation);
      }
      else if(command.getTrajectoryFrame() != trajectoryFrame)
      {
         PrintTools.warn(warningPrefix + "Was executing in ." + trajectoryFrame.getName() + " can't switch to " + command.getTrajectoryFrame() + " without override");
         return false;
      }
      
      command.getTrajectoryPointList().changeFrame(trajectoryFrame);
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

      trackingOrientation.set(true);
      trackingPosition.set(false);
      return true;
   }

   public boolean handlePoseTrajectoryCommand(SE3TrajectoryControllerCommand<?, ?> command, FramePose initialPose)
   {
      if (!checkPoseGainsAndWeights())
         return false;

      if (!handleCommandInternal(command))
         return false;

      boolean override = command.getExecutionMode() == ExecutionMode.OVERRIDE;
      if (!override && (!trackingPosition.getBooleanValue() && trackingOrientation.getBooleanValue()))
      {
         PrintTools.warn(warningPrefix + "Was tracking orientation only. Can not queue pose trajectory.");
         return false;
      }

      if (override || isEmpty())
      {
         overrideTrajectory();
         if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
            queueInitialPoint(initialPose);
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

   @Override
   public double getLastTrajectoryPointTime()
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
         return pointQueue.peekLast().getTime();
      }
   }

   @Override
   public boolean isEmpty()
   {
      return pointQueue.isEmpty() && orientationTrajectoryGenerator.isDone();
   }

   private boolean checkTime(double time)
   {
      boolean timeValid = time > getLastTrajectoryPointTime();
      if (!timeValid)
         PrintTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
      return timeValid;
   }

   private boolean queuePoint(FrameSE3TrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      pointQueue.addLast().setIncludingFrame(trajectoryPoint);
      return true;
   }

   private boolean queuePoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      if (atCapacityLimit())
         return false;

      FrameSE3TrajectoryPoint point = pointQueue.addLast();
      desiredOrientation.setToNaN(trajectoryPoint.getReferenceFrame());
      trajectoryPoint.getOrientation(desiredOrientation);
      desiredAngularVelocity.setToNaN(trajectoryPoint.getReferenceFrame());
      trajectoryPoint.getAngularVelocity(desiredAngularVelocity);
      point.setToZero(trajectoryPoint.getReferenceFrame());
      point.setOrientation(desiredOrientation);
      point.setAngularVelocity(desiredAngularVelocity);
      point.setTime(trajectoryPoint.getTime());
      return true;
   }

   private void queueInitialPoint(FramePose initialPose)
   {
      FrameSE3TrajectoryPoint point = pointQueue.addLast();
      point.setToZero(initialPose.getReferenceFrame());
      point.setTime(0.0);
      initialPose.getPoseIncludingFrame(desiredPosition, desiredOrientation);
      point.setPosition(desiredPosition);
      point.setOrientation(desiredOrientation);
      point.changeFrame(trajectoryFrame);
   }

   private void queueInitialPoint(FrameOrientation initialOrientation)
   {
      FrameSE3TrajectoryPoint point = pointQueue.addLast();
      point.setToZero(initialOrientation.getReferenceFrame());
      point.setTime(0.0);
      point.setOrientation(initialOrientation);
      point.changeFrame(trajectoryFrame);
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

   private void overrideTrajectory()
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
      if (!hasAngularWeight.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing angular weight.");
         success = false;
      }
      if (!hasOrientaionGains.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing orientation gains.");
         success = false;
      }
      return success;
   }

   private boolean checkPositionGainsAndWeights()
   {
      boolean success = true;
      if (!hasLinearWeight.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing linear weight.");
         success = false;
      }
      if (!hasPositionGains.getBooleanValue())
      {
         PrintTools.warn(warningPrefix + "Missing position gains.");
         success = false;
      }
      return success;
   }

}
