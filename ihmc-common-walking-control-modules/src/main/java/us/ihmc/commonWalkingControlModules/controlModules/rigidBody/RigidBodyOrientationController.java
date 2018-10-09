package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class RigidBodyOrientationController extends RigidBodyControlState
{
   private final OrientationFeedbackControlCommand feedbackControlCommand = new OrientationFeedbackControlCommand();

   private final YoInteger numberOfPointsInQueue;
   private final YoInteger numberOfPointsInGenerator;
   private final YoInteger numberOfPoints;
   private final MultipleWaypointsOrientationTrajectoryGenerator trajectoryGenerator;
   private final FrameSO3TrajectoryPoint lastPointAdded = new FrameSO3TrajectoryPoint();
   private final RecyclingArrayDeque<FrameSO3TrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyTaskspaceControlState.maxPoints,
                                                                                                     FrameSO3TrajectoryPoint.class,
                                                                                                     FrameSO3TrajectoryPoint::set);

   private final YoBoolean usingWeightFromMessage;
   private final WeightMatrix3D defaultWeightMatrix = new WeightMatrix3D();
   private final WeightMatrix3D messageWeightMatrix = new WeightMatrix3D();
   private final YoFrameVector3D currentWeight;

   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private Vector3DReadOnly defaultWeight;
   private PID3DGainsReadOnly gains;

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAcceleration = new FrameVector3D();

   private final BooleanProvider useBaseFrameForControl;

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame baseFrame;

   public RigidBodyOrientationController(String postfix, RigidBody bodyToControl, RigidBody baseBody, RigidBody elevator,
                                         Collection<ReferenceFrame> trajectoryFrames, ReferenceFrame controlFrame, ReferenceFrame baseFrame, YoDouble yoTime,
                                         BooleanProvider useBaseFrameForControl, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.TASKSPACE, bodyToControl.getName() + postfix, yoTime, parentRegistry);

      String bodyName = bodyToControl.getName() + postfix;
      String prefix = bodyName + "TaskspaceOrientation";

      numberOfPointsInQueue = new YoInteger(prefix + "NumberOfPointsInQueue", registry);
      numberOfPointsInGenerator = new YoInteger(prefix + "NumberOfPointsInGenerator", registry);
      numberOfPoints = new YoInteger(prefix + "NumberOfPoints", registry);
      trajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(bodyName, RigidBodyTaskspaceControlState.maxPointsInGenerator, true,
                                                                                ReferenceFrame.getWorldFrame(), registry);
      if (trajectoryFrames != null)
      {
         trajectoryFrames.forEach(frame -> trajectoryGenerator.registerNewTrajectoryFrame(frame));
      }
      trajectoryGenerator.registerNewTrajectoryFrame(baseFrame);

      usingWeightFromMessage = new YoBoolean(prefix + "UsingWeightFromMessage", registry);
      currentWeight = new YoFrameVector3D(prefix + "CurrentWeight", null, registry);

      feedbackControlCommand.set(elevator, bodyToControl);
      feedbackControlCommand.setPrimaryBase(baseBody);

      this.bodyFrame = bodyToControl.getBodyFixedFrame();
      this.baseFrame = baseFrame;
      this.useBaseFrameForControl = useBaseFrameForControl;
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      this.gains = gains;
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      this.defaultWeight = weights;
   }

   public void holdCurrent()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      desiredOrientation.setToZero(bodyFrame);
      queueInitialPoint(desiredOrientation);

      trajectoryDone.set(false);
   }

   public void holdCurrentDesired()
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      trajectoryGenerator.getOrientation(desiredOrientation);
      queueInitialPoint(desiredOrientation);

      trajectoryDone.set(false);
   }

   public void goToOrientationFromCurrent(FrameQuaternionReadOnly orientation, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      trajectoryGenerator.changeFrame(baseFrame);
      desiredOrientation.setToZero(bodyFrame);
      queueInitialPoint(desiredOrientation);

      FrameSO3TrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(baseFrame);
      trajectoryPoint.setTime(trajectoryTime);

      desiredOrientation.setIncludingFrame(orientation);
      desiredOrientation.changeFrame(baseFrame);
      trajectoryPoint.setOrientation(desiredOrientation);

      trajectoryDone.set(false);
   }

   public void goToOrientation(FrameQuaternionReadOnly orientation, FrameQuaternionReadOnly initialOrientation, double trajectoryTime)
   {
      clear();
      setTrajectoryStartTimeToCurrentTime();
      trajectoryGenerator.changeFrame(baseFrame);
      desiredOrientation.setIncludingFrame(initialOrientation);
      queueInitialPoint(desiredOrientation);

      FrameSO3TrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(baseFrame);
      trajectoryPoint.setTime(trajectoryTime);

      desiredOrientation.setIncludingFrame(orientation);
      desiredOrientation.changeFrame(baseFrame);
      trajectoryPoint.setOrientation(desiredOrientation);

      trajectoryDone.set(false);
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      double timeInTrajectory = getTimeInTrajectory();

      if (!trajectoryDone.getValue())
      {
         if (trajectoryGenerator.isDone() || trajectoryGenerator.getLastWaypointTime() <= timeInTrajectory)
         {
            fillAndReinitializeTrajectories();
         }
      }

      trajectoryGenerator.compute(timeInTrajectory);
      trajectoryGenerator.getAngularData(desiredOrientation, desiredVelocity, feedForwardAcceleration);

      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      desiredVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardAcceleration.changeFrame(ReferenceFrame.getWorldFrame());

      feedbackControlCommand.set(desiredOrientation, desiredVelocity);
      feedbackControlCommand.setFeedForwardAction(feedForwardAcceleration);
      feedbackControlCommand.setGains(gains);

      // This will improve the tracking with respect to moving trajectory frames.
      if (useBaseFrameForControl.getValue())
      {
         feedbackControlCommand.setControlBaseFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
      }
      else
      {
         feedbackControlCommand.resetControlBaseFrame();
      }

      // Update the QP weight and selection YoVariables:
      defaultWeightMatrix.set(defaultWeight);
      defaultWeightMatrix.setWeightFrame(null);
      WeightMatrix3D weightMatrix = usingWeightFromMessage.getValue() ? messageWeightMatrix : defaultWeightMatrix;
      currentWeight.set(weightMatrix);

      feedbackControlCommand.setWeightMatrix(weightMatrix);
      feedbackControlCommand.setSelectionMatrix(selectionMatrix);

      numberOfPointsInQueue.set(pointQueue.size());
      numberOfPointsInGenerator.set(trajectoryGenerator.getCurrentNumberOfWaypoints());
      numberOfPoints.set(numberOfPointsInQueue.getIntegerValue() + numberOfPointsInGenerator.getIntegerValue());
   }

   private void fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         trajectoryDone.set(true);
         return;
      }

      if (!trajectoryGenerator.isEmpty())
      {
         trajectoryGenerator.clear();
         lastPointAdded.changeFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
         trajectoryGenerator.appendWaypoint(lastPointAdded);
      }

      int currentNumberOfWaypoints = trajectoryGenerator.getCurrentNumberOfWaypoints();
      int pointsToAdd = RigidBodyTaskspaceControlState.maxPointsInGenerator - currentNumberOfWaypoints;
      for (int pointIdx = 0; pointIdx < pointsToAdd; pointIdx++)
      {
         if (pointQueue.isEmpty())
            break;

         FrameSO3TrajectoryPoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd);
         trajectoryGenerator.appendWaypoint(pointToAdd);
      }

      trajectoryGenerator.initialize();
   }

   public boolean handleSO3TrajectoryCommand(SO3TrajectoryControllerCommand command, FrameQuaternionReadOnly initialOrientation)
   {
      if (!handleCommandInternal(command))
      {
         return false;
      }

      if (command.getExecutionMode() == ExecutionMode.OVERRIDE || isEmpty())
      {
         clear();
         trajectoryGenerator.changeFrame(command.getTrajectoryFrame());
         selectionMatrix.set(command.getSelectionMatrix());

         if (command.getTrajectoryPoint(0).getTime() > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
         {
            desiredOrientation.setIncludingFrame(initialOrientation);
            queueInitialPoint(desiredOrientation);
         }

         messageWeightMatrix.set(command.getWeightMatrix());
         boolean messageHasValidWeights = true;
         for (int i = 0; i < 3; i++)
         {
            double weight = messageWeightMatrix.getElement(i);
            messageHasValidWeights = messageHasValidWeights && (Double.isNaN(weight) || weight < 0.0);
         }
         usingWeightFromMessage.set(messageHasValidWeights);
      }
      else if (command.getTrajectoryFrame() != trajectoryGenerator.getCurrentTrajectoryFrame())
      {
         PrintTools.warn(warningPrefix + "Was executing in " + trajectoryGenerator.getCurrentTrajectoryFrame() + " can not switch to "
               + command.getTrajectoryFrame() + " without override.");
         return false;
      }
      else if (!selectionMatrix.equals(command.getSelectionMatrix()))
      {
         PrintTools.warn(warningPrefix + "Received a change of selection matrix without an override. Was\n" + selectionMatrix + "\nRequested\n"
               + command.getSelectionMatrix());
         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
      }

      return true;
   }

   private void queueInitialPoint(FrameQuaternion initialOrientation)
   {
      initialOrientation.changeFrame(trajectoryGenerator.getCurrentTrajectoryFrame());
      FrameSO3TrajectoryPoint initialPoint = pointQueue.addLast();
      initialPoint.setToZero(trajectoryGenerator.getCurrentTrajectoryFrame());
      initialPoint.setTime(0.0);
      initialPoint.setOrientation(initialOrientation);
   }

   private boolean queuePoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      if (pointQueue.size() >= RigidBodyTaskspaceControlState.maxPoints)
      {
         PrintTools.info(warningPrefix + "Reached maximum capacity of " + RigidBodyTaskspaceControlState.maxPoints + " can not execute trajectory.");
         return false;
      }

      pointQueue.addLast().setIncludingFrame(trajectoryPoint);
      return true;
   }

   private boolean checkTime(double time)
   {
      if (time <= getLastTrajectoryPointTime())
      {
         PrintTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
         return false;
      }
      return true;
   }

   @Override
   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public void onExit()
   {
      clear();
   }

   @Override
   public boolean isEmpty()
   {
      if (!pointQueue.isEmpty())
      {
         return false;
      }
      return trajectoryGenerator.isDone();
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
         return trajectoryGenerator.getLastWaypointTime();
      }
      else
      {
         return pointQueue.peekLast().getTime();
      }
   }

   public void clear()
   {
      selectionMatrix.resetSelection();
      trajectoryGenerator.clear(baseFrame);
      pointQueue.clear();
      numberOfPointsInQueue.set(0);
      numberOfPointsInGenerator.set(0);
      numberOfPoints.set(0);
      usingWeightFromMessage.set(false);
      resetLastCommandId();
   }
}
