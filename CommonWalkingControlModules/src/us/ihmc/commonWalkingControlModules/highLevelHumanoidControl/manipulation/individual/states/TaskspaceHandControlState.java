package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import java.util.Map;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.communication.controllerAPI.command.CommandArrayDeque;
import us.ihmc.communication.packets.Packet;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage.BaseForControl;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.FormattingTools;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * @author twan
 *         Date: 5/9/13
 */
public class TaskspaceHandControlState extends HandControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name;
   private final YoVariableRegistry registry;

   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();

   // viz stuff:
   private final YoFramePose yoDesiredPose;

   // temp stuff:
   private final FramePose desiredPose = new FramePose();
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredLinearVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardLinearAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(worldFrame);
   private final FrameSE3TrajectoryPoint initialTrajectoryPoint = new FrameSE3TrajectoryPoint();

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final FramePose controlFramePose = new FramePose();
   private final PoseReferenceFrame controlFrame;
   private final ReferenceFrame endEffectorFrame;
   private final ReferenceFrame chestFrame;

   private final YoSE3PIDGainsInterface gains;
   private final YoFrameVector yoAngularWeight;
   private final YoFrameVector yoLinearWeight;
   private final Vector3d angularWeight = new Vector3d();
   private final Vector3d linearWeight = new Vector3d();

   private final Map<BaseForControl, ReferenceFrame> baseForControlToReferenceFrameMap;

   private final BooleanYoVariable abortTaskspaceControlState;
   private final LongYoVariable lastCommandId;

   private final BooleanYoVariable isReadyToHandleQueuedCommands;
   private final LongYoVariable numberOfQueuedCommands;
   private final CommandArrayDeque<HandTrajectoryCommand> commandQueue = new CommandArrayDeque<>(HandTrajectoryCommand.class);

   public TaskspaceHandControlState(String namePrefix, RigidBody base, RigidBody endEffector, RigidBody chest, YoSE3PIDGainsInterface gains,
         Map<BaseForControl, ReferenceFrame> baseForControlToReferenceFrameMap, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      super(HandControlMode.TASKSPACE);
      this.gains = gains;
      this.baseForControlToReferenceFrameMap = baseForControlToReferenceFrameMap;

      name = namePrefix + FormattingTools.underscoredToCamelCase(getStateEnum().toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      endEffectorFrame = endEffector.getBodyFixedFrame();
      chestFrame = chest.getBodyFixedFrame();

      yoAngularWeight = new YoFrameVector(namePrefix + "AngularTaskspaceWeight", null, registry);
      yoLinearWeight = new YoFrameVector(namePrefix + "LinearTaskspaceWeight", null, registry);
      yoAngularWeight.set(SolverWeightLevels.HAND_TASKSPACE_WEIGHT, SolverWeightLevels.HAND_TASKSPACE_WEIGHT, SolverWeightLevels.HAND_TASKSPACE_WEIGHT);
      yoLinearWeight.set(SolverWeightLevels.HAND_TASKSPACE_WEIGHT, SolverWeightLevels.HAND_TASKSPACE_WEIGHT, SolverWeightLevels.HAND_TASKSPACE_WEIGHT);
      yoAngularWeight.get(angularWeight);
      yoLinearWeight.get(linearWeight);

      spatialFeedbackControlCommand.set(base, endEffector);
      spatialFeedbackControlCommand.setPrimaryBase(chest);

      controlFrame = new PoseReferenceFrame("trackingFrame", endEffectorFrame);
      yoDesiredPose = new YoFramePose(namePrefix + "DesiredPose", worldFrame, registry);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix, true, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, true, worldFrame, registry);

      for (ReferenceFrame frameToRegister : baseForControlToReferenceFrameMap.values())
      {
         positionTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
         orientationTrajectoryGenerator.registerNewTrajectoryFrame(frameToRegister);
      }

      setupVisualization(namePrefix, yoGraphicsListRegistry);

      abortTaskspaceControlState = new BooleanYoVariable(namePrefix + "AbortTaskspaceControlState", registry);
      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new BooleanYoVariable(namePrefix + "IsReadyToHandleQueuedHandTrajectoryCommands", registry);
      numberOfQueuedCommands = new LongYoVariable(namePrefix + "NumberOfQueuedCommands", registry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualization(String namePrefix, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList list = new YoGraphicsList(name);

      YoGraphicCoordinateSystem desiredPoseViz = new YoGraphicCoordinateSystem(namePrefix + "DesiredPose", yoDesiredPose, 0.3);
      list.add(desiredPoseViz);

      yoGraphicsListRegistry.registerYoGraphicsList(list);
      list.hideYoGraphics();
   }

   public void setWeight(double weight)
   {
      yoAngularWeight.set(weight, weight, weight);
      yoLinearWeight.set(weight, weight, weight);
   }

   public void setWeights(Vector3d angularWeight, Vector3d linearWeight)
   {
      yoAngularWeight.set(angularWeight);
      yoLinearWeight.set(linearWeight);
   }

   public void holdPositionInChest(ReferenceFrame newControlFrame, boolean initializeToCurrent)
   {
      updateControlFrameAndDesireds(newControlFrame, initializeToCurrent, initialTrajectoryPoint);
      initialTrajectoryPoint.changeFrame(chestFrame);

      positionTrajectoryGenerator.clear(chestFrame);
      orientationTrajectoryGenerator.clear(chestFrame);
      positionTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      orientationTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      positionTrajectoryGenerator.initialize();
      positionTrajectoryGenerator.initialize();

      isReadyToHandleQueuedCommands.set(false);
      clearCommandQueue(INVALID_MESSAGE_ID);
   }

   public boolean handleHandTrajectoryCommand(HandTrajectoryCommand command, ReferenceFrame newControlFrame, boolean initializeToCurrent)
   {
      isReadyToHandleQueuedCommands.set(true);
      clearCommandQueue(command.getCommandId());
      initializeTrajectoryGenerators(command, newControlFrame, initializeToCurrent, 0.0);
      return true;
   }

   public boolean queueHandTrajectoryCommand(HandTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         PrintTools.warn(this, "The very first " + command.getClass().getSimpleName() + " of a series must be " + ExecutionMode.OVERRIDE + ". Aborting motion.");
         return false;
      }

      long previousCommandId = command.getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         PrintTools.warn(this, "Previous command ID mismatch: previous ID from command = " + previousCommandId
               + ", last message ID received by the controller = " + lastCommandId.getLongValue() + ". Aborting motion.");
         isReadyToHandleQueuedCommands.set(false);
         clearCommandQueue(INVALID_MESSAGE_ID);
         abortTaskspaceControlState.set(true);
         return false;
      }

      if (command.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         PrintTools.warn(this, "Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         isReadyToHandleQueuedCommands.set(false);
         clearCommandQueue(INVALID_MESSAGE_ID);
         abortTaskspaceControlState.set(true);
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getCommandId());

      return true;
   }

   @Override
   public void doAction()
   {
      positionTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.compute(getTimeInCurrentState());

      if (positionTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
      {
         double firstTrajectoryPointTime = positionTrajectoryGenerator.getLastWaypointTime();
         HandTrajectoryCommand command = commandQueue.poll();
         numberOfQueuedCommands.decrement();
         initializeTrajectoryGenerators(command, firstTrajectoryPointTime);
         positionTrajectoryGenerator.compute(getTimeInCurrentState());
         orientationTrajectoryGenerator.compute(getTimeInCurrentState());
      }

      positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      desiredPose.setPoseIncludingFrame(desiredPosition, desiredOrientation);
      yoDesiredPose.setAndMatchFrame(desiredPose);

      spatialFeedbackControlCommand.changeFrameAndSet(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      spatialFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      spatialFeedbackControlCommand.setGains(gains);
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
      abortTaskspaceControlState.set(false);
   }

   private void initializeTrajectoryGenerators(HandTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      initializeTrajectoryGenerators(command, controlFrame, false, firstTrajectoryPointTime);
   }

   private void initializeTrajectoryGenerators(HandTrajectoryCommand command, ReferenceFrame newControlFrame, boolean initializeToCurrent, double firstTrajectoryPointTime)
   {
      updateControlFrameAndDesireds(newControlFrame, initializeToCurrent, initialTrajectoryPoint);
      command.addTimeOffset(firstTrajectoryPointTime);

      ReferenceFrame trajectoryFrame = baseForControlToReferenceFrameMap.get(command.getBase());

      if (trajectoryFrame == null)
      {
         PrintTools.error(this, "The base: " + command.getBase() + " is not handled.");
         abortTaskspaceControlState.set(true);
         return;
      }

      if (command.getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         initialTrajectoryPoint.changeFrame(worldFrame);

         positionTrajectoryGenerator.clear(worldFrame);
         orientationTrajectoryGenerator.clear(worldFrame);
         positionTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
         orientationTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      }
      else
      {
         positionTrajectoryGenerator.clear(worldFrame);
         orientationTrajectoryGenerator.clear(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         positionTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
         orientationTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      positionTrajectoryGenerator.changeFrame(trajectoryFrame);
      orientationTrajectoryGenerator.changeFrame(trajectoryFrame);

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   private int queueExceedingTrajectoryPointsIfNeeded(HandTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = positionTrajectoryGenerator.getMaximumNumberOfWaypoints() - positionTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      HandTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.setPropertiesOnly(command);

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.addTrajectoryPoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private final FramePoint tempFramePoint = new FramePoint();
   private final FrameOrientation tempFrameOrientation = new FrameOrientation();

   private void updateControlFrameAndDesireds(ReferenceFrame newControlFrame, boolean initializeToCurrent, FrameSE3TrajectoryPoint trajectoryPointToPack)
   {
      trajectoryPointToPack.setToZero(newControlFrame);

      if (!initializeToCurrent)
      {
         positionTrajectoryGenerator.getPosition(tempFramePoint);
         orientationTrajectoryGenerator.getOrientation(tempFrameOrientation);
         desiredPose.setPoseIncludingFrame(tempFramePoint, tempFrameOrientation);
         changeControlFrame(controlFrame, newControlFrame, desiredPose);
         desiredPose.getPoseIncludingFrame(tempFramePoint, tempFrameOrientation);
         trajectoryPointToPack.setToZero(desiredPose.getReferenceFrame());
         trajectoryPointToPack.setPosition(tempFramePoint);
         trajectoryPointToPack.setOrientation(tempFrameOrientation);
      }

      setControlFrameFixedInEndEffector(newControlFrame);
   }

   private final RigidBodyTransform oldTrackingFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newTrackingFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewTrackingFrameToOldTrackingFrame = new RigidBodyTransform();

   private void changeControlFrame(ReferenceFrame oldControlFrame, ReferenceFrame newControlFrame, FramePose framePoseToModify)
   {
      if (oldControlFrame == newControlFrame)
         return;

      framePoseToModify.getPose(oldTrackingFrameDesiredTransform);
      newControlFrame.getTransformToDesiredFrame(transformFromNewTrackingFrameToOldTrackingFrame, oldControlFrame);
      newTrackingFrameDesiredTransform.multiply(oldTrackingFrameDesiredTransform, transformFromNewTrackingFrameToOldTrackingFrame);
      framePoseToModify.setPose(newTrackingFrameDesiredTransform);
   }

   private void setControlFrameFixedInEndEffector(ReferenceFrame controlFrame)
   {
      controlFramePose.setToZero(controlFrame);
      controlFramePose.changeFrame(endEffectorFrame);
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePose);
      this.controlFrame.setPoseAndUpdate(controlFramePose);
   }

   public ReferenceFrame getTrajectoryFrame()
   {
      return positionTrajectoryGenerator.getCurrentTrajectoryFrame();
   }

   @Override
   public boolean isDone()
   {
      boolean areTrajectoriesDone = positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
      boolean commandQueueEmpty = commandQueue.isEmpty();
      return areTrajectoriesDone && commandQueueEmpty;
   }

   @Override
   public boolean isAbortRequested()
   {
      return abortTaskspaceControlState.getBooleanValue();
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return privilegedConfigurationCommand;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }
}
