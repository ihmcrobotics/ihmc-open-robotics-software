package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class RigidBodyOrientationControlHelper
{
   private final OrientationFeedbackControlCommand feedbackControlCommand = new OrientationFeedbackControlCommand();

   private final MultipleWaypointsOrientationTrajectoryGenerator trajectoryGenerator;
   private final FrameSO3TrajectoryPoint lastPointAdded = new FrameSO3TrajectoryPoint();
   private final RecyclingArrayDeque<FrameSO3TrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyTaskspaceControlState.maxPoints,
                                                                                                     FrameSO3TrajectoryPoint.class,
                                                                                                     FrameSO3TrajectoryPoint::set);

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

   private final ReferenceFrame baseFrame;
   private final ReferenceFrame bodyFrame;

   private final String warningPrefix;

   public RigidBodyOrientationControlHelper(String warningPrefix, RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, RigidBodyBasics elevator,
                                            ReferenceFrame controlFrame, ReferenceFrame baseFrame, BooleanProvider useBaseFrameForControl,
                                            BooleanProvider useWeightFromMessage, YoVariableRegistry registry)
   {
      this.warningPrefix = warningPrefix;
      this.useBaseFrameForControl = useBaseFrameForControl;
      this.useWeightFromMessage = useWeightFromMessage;
      this.baseFrame = baseFrame;

      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "TaskspaceOrientation";

      trajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(bodyName, RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                                ReferenceFrame.getWorldFrame(), registry);
      trajectoryGenerator.clear(baseFrame);

      currentWeight = new YoFrameVector3D(prefix + "CurrentWeight", null, registry);

      feedbackControlCommand.set(elevator, bodyToControl);
      feedbackControlCommand.setPrimaryBase(baseBody);

      defaultControlFrame = controlFrame;
      bodyFrame = bodyToControl.getBodyFixedFrame();
      controlFrameOrientation = new FrameQuaternion(bodyFrame);
      previousControlFrameOrientation = new FrameQuaternion(bodyFrame);
      setDefaultControlFrame();
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
      if (command.getExecutionMode() == ExecutionMode.OVERRIDE || isEmpty())
      {
         // Record the current desired orientation and the control frame orientation.
         getDesiredOrientation(desiredOrientation);
         previousControlFrameOrientation.set(controlFrameOrientation);

         clear();

         // Set the new control frame and move the desired orientation to be for that frame.
         if (command.useCustomControlFrame())
         {
            setControlFrameOrientation(command.getControlFramePose().getRotationMatrix());
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
         LogTools.warn(warningPrefix + "Was executing in " + trajectoryGenerator.getReferenceFrame() + " can not switch to "
               + command.getTrajectoryFrame() + " without override.");
         return false;
      }
      else if (!selectionMatrix.equals(command.getSelectionMatrix()))
      {
         LogTools.warn(warningPrefix + "Received a change of selection matrix without an override. Was\n" + selectionMatrix + "\nRequested\n"
               + command.getSelectionMatrix());
         return false;
      }

      command.getTrajectoryPointList().changeFrame(trajectoryGenerator.getReferenceFrame());
      for (int i = 0; i < command.getNumberOfTrajectoryPoints(); i++)
      {
         if (!checkTime(command.getTrajectoryPoint(i).getTime()))
            return false;
         if (!queuePoint(command.getTrajectoryPoint(i)))
            return false;
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

   private boolean queuePoint(FrameSO3TrajectoryPoint trajectoryPoint)
   {
      if (pointQueue.size() >= RigidBodyTaskspaceControlState.maxPoints)
      {
         LogTools.warn(warningPrefix + "Reached maximum capacity of " + RigidBodyTaskspaceControlState.maxPoints + " can not execute trajectory.");
         return false;
      }

      pointQueue.addLast().setIncludingFrame(trajectoryPoint);
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
   }

   public void disable()
   {
      clear();
      holdCurrentDesired();
      selectionMatrix.clearSelection();
   }
}
