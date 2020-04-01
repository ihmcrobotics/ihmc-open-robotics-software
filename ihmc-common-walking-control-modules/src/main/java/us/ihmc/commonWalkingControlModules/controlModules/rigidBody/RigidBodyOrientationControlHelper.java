package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class RigidBodyOrientationControlHelper
{
   private final OrientationFeedbackControlCommand feedbackControlCommand = new OrientationFeedbackControlCommand();

   private final MultipleWaypointsOrientationTrajectoryGenerator trajectoryGenerator;
   private final FrameSO3TrajectoryPoint lastPointAdded = new FrameSO3TrajectoryPoint();
   private final RecyclingArrayDeque<FrameSO3TrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyTaskspaceControlState.maxPoints,
                                                                                                     FrameSO3TrajectoryPoint.class,
                                                                                                     FrameSO3TrajectoryPoint::set);
   /**
    * Used when streaming to account for time variations occurring during the transport of the message
    * over the network.
    */
   private final YoDouble streamTimestampOffset;

   private boolean messageWeightValid = false;
   private final BooleanProvider useWeightFromMessage;

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

   private final FixedFrameQuaternionBasics previousControlFrameOrientation;
   private final FixedFrameQuaternionBasics controlFrameOrientation;
   private final ReferenceFrame defaultControlFrame;

   private final Quaternion integratedOrientation = new Quaternion();
   private final Vector3D integratedRotationVector = new Vector3D();

   private final ReferenceFrame baseFrame;
   private final ReferenceFrame bodyFrame;

   private final String warningPrefix;

   private final DoubleProvider time;

   public RigidBodyOrientationControlHelper(String warningPrefix, RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, RigidBodyBasics elevator,
                                            ReferenceFrame controlFrame, ReferenceFrame baseFrame, BooleanProvider useBaseFrameForControl,
                                            BooleanProvider useWeightFromMessage, DoubleProvider time, YoVariableRegistry registry)
   {
      this.warningPrefix = warningPrefix;
      this.useBaseFrameForControl = useBaseFrameForControl;
      this.useWeightFromMessage = useWeightFromMessage;
      this.baseFrame = baseFrame;
      this.time = time;

      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "TaskspaceOrientation";

      trajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(bodyName,
                                                                                RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                                ReferenceFrame.getWorldFrame(),
                                                                                registry);
      trajectoryGenerator.clear(baseFrame);

      currentWeight = new YoFrameVector3D(prefix + "CurrentWeight", null, registry);

      feedbackControlCommand.set(elevator, bodyToControl);
      feedbackControlCommand.setPrimaryBase(baseBody);

      defaultControlFrame = controlFrame;
      bodyFrame = bodyToControl.getBodyFixedFrame();
      controlFrameOrientation = new FrameQuaternion(bodyFrame);
      previousControlFrameOrientation = new FrameQuaternion(bodyFrame);
      setDefaultControlFrame();

      streamTimestampOffset = new YoDouble(prefix + "StreamTimestampOffset", registry);
      streamTimestampOffset.setToNaN();
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      this.gains = gains;
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      this.defaultWeight = weights;
   }

   private void setDefaultControlFrame()
   {
      controlFrameOrientation.setFromReferenceFrame(defaultControlFrame);
      feedbackControlCommand.setBodyFixedOrientationToControl(controlFrameOrientation);
   }

   private void setControlFrameOrientation(Orientation3DReadOnly controlFrameOrientationInBodyFrame)
   {
      controlFrameOrientation.set(controlFrameOrientationInBodyFrame);
      feedbackControlCommand.setBodyFixedOrientationToControl(controlFrameOrientation);
   }

   public static void modifyControlFrame(FrameQuaternionBasics desiredOrientationToModify, QuaternionReadOnly previousControlFrameOrientation,
                                         QuaternionReadOnly newControlFrameOrientation)
   {
      desiredOrientationToModify.multiplyConjugateOther(previousControlFrameOrientation);
      desiredOrientationToModify.multiply(newControlFrameOrientation);
   }

   public void holdCurrent()
   {
      clear();
      desiredOrientation.setIncludingFrame(controlFrameOrientation);
      queueInitialPoint(desiredOrientation);
   }

   public void holdCurrentDesired()
   {
      getDesiredOrientation(desiredOrientation);
      previousControlFrameOrientation.set(controlFrameOrientation);
      clear();
      modifyControlFrame(desiredOrientation, previousControlFrameOrientation, controlFrameOrientation);
      queueInitialPoint(desiredOrientation);
   }

   public void goToOrientationFromCurrent(FrameQuaternionReadOnly orientation, double trajectoryTime)
   {
      holdCurrent();

      FrameSO3TrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredOrientation.setIncludingFrame(orientation);
      desiredOrientation.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setOrientation(desiredOrientation);
   }

   public void goToOrientation(FrameQuaternionReadOnly orientation, double trajectoryTime)
   {
      holdCurrentDesired();

      FrameSO3TrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredOrientation.setIncludingFrame(orientation);
      desiredOrientation.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setOrientation(desiredOrientation);
   }

   public void getDesiredOrientation(FrameQuaternion orientationToPack)
   {
      if (trajectoryGenerator.isEmpty())
      {
         orientationToPack.setIncludingFrame(controlFrameOrientation);
         orientationToPack.changeFrame(trajectoryGenerator.getReferenceFrame());
      }
      else
      {
         trajectoryGenerator.getOrientation(orientationToPack);
      }
   }

   public boolean doAction(double timeInTrajectory)
   {
      boolean done = false;
      if (trajectoryGenerator.isDone() || trajectoryGenerator.getLastWaypointTime() <= timeInTrajectory)
      {
         done = fillAndReinitializeTrajectories();
      }

      if (done)
         streamTimestampOffset.setToNaN();

      trajectoryGenerator.compute(timeInTrajectory);
      trajectoryGenerator.getAngularData(desiredOrientation, desiredVelocity, feedForwardAcceleration);

      desiredOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      desiredVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardAcceleration.changeFrame(ReferenceFrame.getWorldFrame());

      feedbackControlCommand.setInverseDynamics(desiredOrientation, desiredVelocity, feedForwardAcceleration);
      feedbackControlCommand.setGains(gains);

      // This will improve the tracking with respect to moving trajectory frames.
      if (useBaseFrameForControl.getValue())
      {
         feedbackControlCommand.setControlBaseFrame(trajectoryGenerator.getReferenceFrame());
      }
      else
      {
         feedbackControlCommand.resetControlBaseFrame();
      }

      // Update the QP weight and selection YoVariables:
      defaultWeightMatrix.set(defaultWeight);
      defaultWeightMatrix.setWeightFrame(null);
      WeightMatrix3D weightMatrix = useWeightFromMessage.getValue() ? messageWeightMatrix : defaultWeightMatrix;
      currentWeight.set(weightMatrix);

      feedbackControlCommand.setWeightMatrix(weightMatrix);
      feedbackControlCommand.setSelectionMatrix(selectionMatrix);

      return done;
   }

   private boolean fillAndReinitializeTrajectories()
   {
      if (pointQueue.isEmpty())
      {
         return true;
      }

      if (!trajectoryGenerator.isEmpty())
      {
         trajectoryGenerator.clear();
         lastPointAdded.changeFrame(trajectoryGenerator.getReferenceFrame());
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
      return false;
   }

   public boolean handleTrajectoryCommand(SO3TrajectoryControllerCommand command)
   {
      // The timestamp is being cleared in the clear method, need to save it to use it further down.
      double previousStreamTimestampOffset = streamTimestampOffset.getValue();

      // Both OVERRIDE and STREAM clear the current trajectory.
      if (command.getExecutionMode() != ExecutionMode.QUEUE || isEmpty())
      {
         // Record the current desired orientation and the control frame orientation.
         getDesiredOrientation(desiredOrientation);
         previousControlFrameOrientation.set(controlFrameOrientation);

         clear();

         // Set the new control frame and move the desired orientation to be for that frame.
         if (command.useCustomControlFrame())
         {
            setControlFrameOrientation(command.getControlFramePose().getRotation());
         }
         modifyControlFrame(desiredOrientation, previousControlFrameOrientation, controlFrameOrientation);

         trajectoryGenerator.changeFrame(command.getTrajectoryFrame());
         selectionMatrix.set(command.getSelectionMatrix());

         if (command.getTrajectoryPoint(0).getTime() > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
         {
            queueInitialPoint(desiredOrientation);
         }

         messageWeightMatrix.set(command.getWeightMatrix());
         boolean messageHasValidWeights = true;
         for (int i = 0; i < 3; i++)
         {
            double weight = messageWeightMatrix.getElement(i);
            messageHasValidWeights = messageHasValidWeights && !(Double.isNaN(weight) || weight < 0.0);
         }
         messageWeightValid = messageHasValidWeights;
      }
      else if (command.getTrajectoryFrame() != trajectoryGenerator.getReferenceFrame())
      {
         LogTools.warn(warningPrefix + "Was executing in " + trajectoryGenerator.getReferenceFrame() + " can not switch to " + command.getTrajectoryFrame()
               + " without override.");
         return false;
      }
      else if (!selectionMatrix.equals(command.getSelectionMatrix()))
      {
         LogTools.warn(warningPrefix + "Received a change of selection matrix without an override. Was\n" + selectionMatrix + "\nRequested\n"
               + command.getSelectionMatrix());
         return false;
      }

      FrameSO3TrajectoryPointList trajectoryPoints = command.getTrajectoryPointList();
      trajectoryPoints.changeFrame(trajectoryGenerator.getReferenceFrame());

      if (command.getExecutionMode() == ExecutionMode.STREAM)
      {
         double timeOffset = 0.0;

         if (command.getTimestamp() <= 0)
         {
            streamTimestampOffset.setToNaN();
         }
         else
         {
            double senderTime = Conversions.nanosecondsToSeconds(command.getTimestamp());
            timeOffset = time.getValue() - senderTime;
            if (Double.isNaN(previousStreamTimestampOffset))
            {
               streamTimestampOffset.set(timeOffset);
            }
            else
            {
               /*
                * Update to the smallest time offset, which is closer to the true offset between the sender CPU and
                * control CPU. If the change in offset is too large though, we always set the streamTimestampOffset
                * for safety.
                */
               if (Math.abs(timeOffset - previousStreamTimestampOffset) > 0.5)
                  streamTimestampOffset.set(timeOffset);
               else
                  streamTimestampOffset.set(Math.min(timeOffset, previousStreamTimestampOffset));
            }
         }

         if (trajectoryPoints.getNumberOfTrajectoryPoints() != 1)
         {
            LogTools.warn("When streaming, trajectories should contain only 1 trajectory point, was: " + trajectoryPoints.getNumberOfTrajectoryPoints());
            return false;
         }

         FrameSO3TrajectoryPoint trajectoryPoint = trajectoryPoints.getTrajectoryPoint(0);

         if (trajectoryPoint.getTime() != 0.0)
         {
            LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + trajectoryPoint.getTime());
            return false;
         }

         FrameSO3TrajectoryPoint initialPoint = addPoint();

         if (initialPoint == null)
            return false;

         initialPoint.setIncludingFrame(trajectoryPoint);
         if (!streamTimestampOffset.isNaN())
            initialPoint.setTime(streamTimestampOffset.getValue() - timeOffset);

         FrameSO3TrajectoryPoint integratedPoint = addPoint();

         if (integratedPoint == null)
            return false;

         integratedPoint.setIncludingFrame(initialPoint);
         integratedRotationVector.setAndScale(command.getStreamIntegrationDuration(), integratedPoint.getAngularVelocity());
         integratedOrientation.setRotationVector(integratedRotationVector);
         integratedOrientation.append(integratedPoint.getOrientation());
         integratedPoint.setOrientation(integratedOrientation);
         integratedPoint.setTime(command.getStreamIntegrationDuration() + initialPoint.getTime());
      }
      else
      {
         for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
         {
            if (!checkTime(command.getTrajectoryPoint(i).getTime()))
               return false;
            if (!queuePoint(command.getTrajectoryPoint(i)))
               return false;
         }
      }

      return true;
   }

   public boolean isMessageWeightValid()
   {
      return messageWeightValid;
   }

   public int getNumberOfPointsInQueue()
   {
      return pointQueue.size();
   }

   public int getNumberOfPointsInGenerator()
   {
      return trajectoryGenerator.getCurrentNumberOfWaypoints();
   }

   private void queueInitialPoint(FrameQuaternion initialOrientation)
   {
      initialOrientation.changeFrame(trajectoryGenerator.getReferenceFrame());
      FrameSO3TrajectoryPoint initialPoint = pointQueue.addLast();
      initialPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      initialPoint.setTime(0.0);
      initialPoint.setOrientation(initialOrientation);
   }

   private FrameSO3TrajectoryPoint addPoint()
   {
      if (pointQueue.size() >= RigidBodyTaskspaceControlState.maxPoints)
      {
         LogTools.warn(warningPrefix + "Reached maximum capacity of " + RigidBodyTaskspaceControlState.maxPoints + " can not execute trajectory.");
         return null;
      }

      return pointQueue.addLast();
   }

   private boolean queuePoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      FrameSO3TrajectoryPoint point = addPoint();
      if (point == null)
         return false;

      point.setIncludingFrame(trajectoryPoint);
      return true;
   }

   private boolean checkTime(double time)
   {
      if (time <= getLastTrajectoryPointTime())
      {
         LogTools.warn(warningPrefix + "Time in trajectory must be strictly increasing.");
         return false;
      }
      return true;
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   public void onExit()
   {
      clear();
   }

   public boolean isEmpty()
   {
      if (!pointQueue.isEmpty())
      {
         return false;
      }
      return trajectoryGenerator.isDone();
   }

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
      setDefaultControlFrame();
      pointQueue.clear();
      streamTimestampOffset.setToNaN();
   }

   public void disable()
   {
      clear();
      holdCurrentDesired();
      selectionMatrix.clearSelection();
   }
}
