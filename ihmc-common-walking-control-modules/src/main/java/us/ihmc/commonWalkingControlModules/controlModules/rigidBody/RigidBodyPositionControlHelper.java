package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EuclideanTrajectoryControllerCommand;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class RigidBodyPositionControlHelper
{
   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private final MultipleWaypointsPositionTrajectoryGenerator trajectoryGenerator;
   private final FrameEuclideanTrajectoryPoint lastPointAdded = new FrameEuclideanTrajectoryPoint();
   private final RecyclingArrayDeque<FrameEuclideanTrajectoryPoint> pointQueue = new RecyclingArrayDeque<>(RigidBodyTaskspaceControlState.maxPoints,
                                                                                                           FrameEuclideanTrajectoryPoint.class,
                                                                                                           FrameEuclideanTrajectoryPoint::set);

   private boolean messageWeightValid = false;
   private final BooleanProvider useWeightFromMessage;

   private final WeightMatrix3D defaultWeightMatrix = new WeightMatrix3D();
   private final WeightMatrix3D messageWeightMatrix = new WeightMatrix3D();
   private final YoFrameVector3D currentWeight;

   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   private Vector3DReadOnly defaultWeight;
   private PID3DGainsReadOnly gains;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAcceleration = new FrameVector3D();

   private final BooleanProvider useBaseFrameForControl;

   private final RigidBodyTransform previousControlFramePose = new RigidBodyTransform();
   private final RigidBodyTransform controlFramePose = new RigidBodyTransform();
   private final ReferenceFrame defaultControlFrame;

   private final Point3D integratedPosition = new Point3D();

   private final ReferenceFrame baseFrame;
   private final ReferenceFrame bodyFrame;

   private final FramePoint3D currentPosition = new FramePoint3D();
   private final YoFramePoint3D yoCurrentPosition;
   private final YoFramePoint3D yoDesiredPosition;
   private final List<YoGraphicPosition> graphics = new ArrayList<>();

   private final String warningPrefix;

   public RigidBodyPositionControlHelper(String warningPrefix, RigidBodyBasics bodyToControl, RigidBodyBasics baseBody, RigidBodyBasics elevator,
                                         ReferenceFrame controlFrame, ReferenceFrame baseFrame, BooleanProvider useBaseFrameForControl,
                                         BooleanProvider useWeightFromMessage, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.warningPrefix = warningPrefix;
      this.useBaseFrameForControl = useBaseFrameForControl;
      this.useWeightFromMessage = useWeightFromMessage;
      this.baseFrame = baseFrame;

      String bodyName = bodyToControl.getName();
      String prefix = bodyName + "TaskspacePosition";

      trajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(bodyName,
                                                                             RigidBodyTaskspaceControlState.maxPointsInGenerator,
                                                                             ReferenceFrame.getWorldFrame(),
                                                                             registry);
      trajectoryGenerator.clear(baseFrame);

      currentWeight = new YoFrameVector3D(prefix + "CurrentWeight", null, registry);

      feedbackControlCommand.set(elevator, bodyToControl);
      feedbackControlCommand.setPrimaryBase(baseBody);

      defaultControlFrame = controlFrame;
      bodyFrame = bodyToControl.getBodyFixedFrame();
      setDefaultControlFrame();

      if (graphicsListRegistry != null)
      {
         yoCurrentPosition = new YoFramePoint3D(prefix + "Current", ReferenceFrame.getWorldFrame(), registry);
         yoDesiredPosition = new YoFramePoint3D(prefix + "Desired", ReferenceFrame.getWorldFrame(), registry);
         setupViz(graphicsListRegistry, bodyName);
      }
      else
      {
         yoCurrentPosition = null;
         yoDesiredPosition = null;
      }
   }

   public void setGains(PID3DGainsReadOnly gains)
   {
      this.gains = gains;
   }

   public void setWeights(Vector3DReadOnly weights)
   {
      this.defaultWeight = weights;
   }

   private void setupViz(YoGraphicsListRegistry graphicsListRegistry, String bodyName)
   {
      String listName = getClass().getSimpleName();

      YoGraphicPosition controlPoint = new YoGraphicPosition(bodyName + "Current", yoCurrentPosition, 0.005, YoAppearance.Red());
      graphicsListRegistry.registerYoGraphic(listName, controlPoint);
      graphics.add(controlPoint);

      YoGraphicPosition desiredPoint = new YoGraphicPosition(bodyName + "Desired", yoDesiredPosition, 0.005, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic(listName, desiredPoint);
      graphics.add(desiredPoint);
   }

   public List<YoGraphicPosition> getGraphics()
   {
      return graphics;
   }

   private void setDefaultControlFrame()
   {
      defaultControlFrame.getTransformToDesiredFrame(controlFramePose, bodyFrame);
      currentPosition.setIncludingFrame(bodyFrame, controlFramePose.getTranslation());
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
   }

   private void setControlFramePose(RigidBodyTransform controlFramePoseInBody)
   {
      controlFramePose.set(controlFramePoseInBody);
      currentPosition.setIncludingFrame(bodyFrame, controlFramePose.getTranslation());
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);
   }

   public static void modifyControlFrame(FixedFramePoint3DBasics desiredPositionToModify, FrameQuaternionBasics desiredOrientationOfPreviousControlFrame,
                                         RigidBodyTransform previousControlFramePose, RigidBodyTransform newControlFramePose)
   {
      // We may skip this if there is no change in control frame.
      if (previousControlFramePose.equals(newControlFramePose))
      {
         return;
      }
      else if (desiredOrientationOfPreviousControlFrame == null)
      {
         throw new RuntimeException("Changing the control frame requires a desired orientation. Bodies that are position controlled do not support control frame changes.");
      }

      desiredOrientationOfPreviousControlFrame.changeFrame(desiredPositionToModify.getReferenceFrame());
      previousControlFramePose.invert();
      previousControlFramePose.multiply(newControlFramePose);
      QuaternionTools.addTransform(desiredOrientationOfPreviousControlFrame, previousControlFramePose.getTranslation(), desiredPositionToModify);
   }

   public void holdCurrent()
   {
      clear();
      desiredPosition.setIncludingFrame(bodyFrame, controlFramePose.getTranslation());
      queueInitialPoint(desiredPosition);
   }

   public void holdCurrentDesired(FrameQuaternionBasics currentDesiredOrientation)
   {
      getDesiredPosition(desiredPosition);
      previousControlFramePose.set(controlFramePose);
      clear();
      modifyControlFrame(desiredPosition, currentDesiredOrientation, previousControlFramePose, controlFramePose);
      queueInitialPoint(desiredPosition);
   }

   public void goToPositionFromCurrent(FramePoint3DReadOnly position, double trajectoryTime)
   {
      holdCurrent();

      FrameEuclideanTrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredPosition.setIncludingFrame(position);
      desiredPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setPosition(desiredPosition);
   }

   public void goToPosition(FramePoint3DReadOnly position, FrameQuaternionBasics currentDesiredOrientation, double trajectoryTime)
   {
      holdCurrentDesired(currentDesiredOrientation);

      FrameEuclideanTrajectoryPoint trajectoryPoint = pointQueue.addLast();
      trajectoryPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setTime(trajectoryTime);

      desiredPosition.setIncludingFrame(position);
      desiredPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      trajectoryPoint.setPosition(desiredPosition);
   }

   public void getDesiredPosition(FramePoint3D positionToPack)
   {
      if (trajectoryGenerator.isEmpty())
      {
         positionToPack.setIncludingFrame(bodyFrame, controlFramePose.getTranslation());
         positionToPack.changeFrame(trajectoryGenerator.getReferenceFrame());
      }
      else
      {
         trajectoryGenerator.getPosition(positionToPack);
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
      trajectoryGenerator.getLinearData(desiredPosition, desiredVelocity, feedForwardAcceleration);

      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      desiredVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardAcceleration.changeFrame(ReferenceFrame.getWorldFrame());

      feedbackControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, feedForwardAcceleration);
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

      if (yoCurrentPosition != null && yoDesiredPosition != null)
      {
         currentPosition.setIncludingFrame(bodyFrame, controlFramePose.getTranslation());
         yoCurrentPosition.setMatchingFrame(currentPosition);
         yoDesiredPosition.setMatchingFrame(desiredPosition);
      }

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

         FrameEuclideanTrajectoryPoint pointToAdd = pointQueue.pollFirst();
         lastPointAdded.setIncludingFrame(pointToAdd);
         trajectoryGenerator.appendWaypoint(pointToAdd);
      }

      trajectoryGenerator.initialize();
      return false;
   }

   public boolean handleTrajectoryCommand(EuclideanTrajectoryControllerCommand command, FrameQuaternionBasics currentDesiredOrientation)
   {
      // Both OVERRIDE and STREAM clear the current trajectory.
      if (command.getExecutionMode() != ExecutionMode.QUEUE || isEmpty())
      {
         // Record the current desired position and the control frame pose.
         getDesiredPosition(desiredPosition);
         previousControlFramePose.set(controlFramePose);

         clear();

         // Set the new control frame and move the desired position to be for that frame.
         if (command.useCustomControlFrame())
         {
            setControlFramePose(command.getControlFramePose());
         }
         modifyControlFrame(desiredPosition, currentDesiredOrientation, previousControlFramePose, controlFramePose);

         trajectoryGenerator.changeFrame(command.getTrajectoryFrame());
         selectionMatrix.set(command.getSelectionMatrix());

         if (command.getTrajectoryPoint(0).getTime() > RigidBodyTaskspaceControlState.timeEpsilonForInitialPoint)
         {
            queueInitialPoint(desiredPosition);
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

      FrameEuclideanTrajectoryPointList trajectoryPoints = command.getTrajectoryPointList();
      trajectoryPoints.changeFrame(trajectoryGenerator.getReferenceFrame());

      if (command.getExecutionMode() == ExecutionMode.STREAM)
      {
         if (trajectoryPoints.getNumberOfTrajectoryPoints() != 1)
         {
            LogTools.warn("When streaming, trajectories should contain only 1 trajectory point, was: " + trajectoryPoints.getNumberOfTrajectoryPoints());
            return false;
         }

         FrameEuclideanTrajectoryPoint trajectoryPoint = trajectoryPoints.getTrajectoryPoint(0);

         if (trajectoryPoint.getTime() != 0.0)
         {
            LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + trajectoryPoint.getTime());
            return false;
         }

         if (!queuePoint(trajectoryPoint))
            return false;

         FrameEuclideanTrajectoryPoint integratedPoint = addPoint();

         if (integratedPoint == null)
            return false;

         integratedPoint.set(trajectoryPoint);
         integratedPosition.scaleAdd(command.getStreamIntegrationDuration(), integratedPoint.getLinearVelocity(), integratedPoint.getPosition());
         integratedPoint.setPosition(integratedPosition);
         integratedPoint.setTime(command.getStreamIntegrationDuration());
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

   private void queueInitialPoint(FramePoint3D initialPosition)
   {
      initialPosition.changeFrame(trajectoryGenerator.getReferenceFrame());
      FrameEuclideanTrajectoryPoint initialPoint = pointQueue.addLast();
      initialPoint.setToZero(trajectoryGenerator.getReferenceFrame());
      initialPoint.setTime(0.0);
      initialPoint.setPosition(initialPosition);
   }

   private FrameEuclideanTrajectoryPoint addPoint()
   {
      if (pointQueue.size() >= RigidBodyTaskspaceControlState.maxPoints)
      {
         LogTools.warn(warningPrefix + "Reached maximum capacity of " + RigidBodyTaskspaceControlState.maxPoints + " can not execute trajectory.");
         return null;
      }

      return pointQueue.addLast();
   }

   private boolean queuePoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      FrameEuclideanTrajectoryPoint point = addPoint();
      if (point == null)
         return false;

      point.set(trajectoryPoint);
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

   public PointFeedbackControlCommand getFeedbackControlCommand()
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
      holdCurrent();
      selectionMatrix.clearSelection();
   }
}
