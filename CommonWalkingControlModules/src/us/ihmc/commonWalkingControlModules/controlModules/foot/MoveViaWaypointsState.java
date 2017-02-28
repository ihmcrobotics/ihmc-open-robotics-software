package us.ihmc.commonWalkingControlModules.controlModules.foot;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SettablePositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.tools.io.printing.PrintTools;

public class MoveViaWaypointsState extends AbstractUnconstrainedState
{
   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final ReferenceFrame footFrame;

   private final BooleanYoVariable isTrajectoryStopped;
   private final BooleanYoVariable isPerformingTouchdown;
   private final SettableDoubleProvider touchdownInitialTimeProvider = new SettableDoubleProvider(0.0);
   private final SettablePositionProvider currentDesiredFootPosition = new SettablePositionProvider();
   private final SoftTouchdownPositionTrajectoryGenerator positionTrajectoryForDisturbanceRecovery;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempLinearVelocity = new FrameVector();
   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameSE3TrajectoryPoint initialTrajectoryPoint = new FrameSE3TrajectoryPoint();

   private final LongYoVariable lastCommandId;

   private final BooleanYoVariable isReadyToHandleQueuedCommands;
   private final LongYoVariable numberOfQueuedCommands;
   private final RecyclingArrayDeque<FootTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(FootTrajectoryCommand.class);

   public MoveViaWaypointsState(FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, VectorProvider touchdownAccelerationProvider, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      super(ConstraintType.MOVE_VIA_WAYPOINTS, footControlHelper, gains, registry);

      String namePrefix = footControlHelper.getRobotSide().getCamelCaseNameForStartOfExpression() + "FootMoveViaWaypoints";

      footFrame = momentumBasedController.getReferenceFrames().getFootFrame(robotSide);

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix, worldFrame, registry);
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, worldFrame, registry);

      isTrajectoryStopped = new BooleanYoVariable(namePrefix + "IsTrajectoryStopped", registry);
      isPerformingTouchdown = new BooleanYoVariable(namePrefix + "IsPerformingTouchdown", registry);

      positionTrajectoryForDisturbanceRecovery = new SoftTouchdownPositionTrajectoryGenerator(namePrefix + "Touchdown", worldFrame, currentDesiredFootPosition,
            touchdownVelocityProvider, touchdownAccelerationProvider, touchdownInitialTimeProvider, registry);

      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new BooleanYoVariable(namePrefix + "IsReadyToHandleQueuedFootTrajectoryCommands", registry);
      numberOfQueuedCommands = new LongYoVariable(namePrefix + "NumberOfQueuedCommands", registry);
   }

   public void holdCurrentPosition()
   {
      initialTrajectoryPoint.setToZero(footFrame);
      initialTrajectoryPoint.changeFrame(worldFrame);

      positionTrajectoryGenerator.clear();
      orientationTrajectoryGenerator.clear();
      positionTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      orientationTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();

      isReadyToHandleQueuedCommands.set(false);
      clearCommandQueue(INVALID_MESSAGE_ID);
   }

   public void handleFootTrajectoryCommand(FootTrajectoryCommand command, boolean initializeToCurrent)
   {
      isReadyToHandleQueuedCommands.set(true);
      clearCommandQueue(command.getCommandId());
      initializeTrajectoryGenerators(command, initializeToCurrent, 0.0);
   }

   public boolean queueFootTrajectoryCommand(FootTrajectoryCommand command)
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
         return false;
      }

      if (command.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         PrintTools.warn(this, "Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         isReadyToHandleQueuedCommands.set(false);
         clearCommandQueue(INVALID_MESSAGE_ID);
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getCommandId());

      return true;
   }

   @Override
   protected void initializeTrajectory()
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      if (getPreviousState() == this)
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(false);
      isTrajectoryStopped.set(false);
      isPerformingTouchdown.set(false);
   };

   @Override
   protected void computeAndPackTrajectory()
   {
      if (isPerformingTouchdown.getBooleanValue())
      {
         positionTrajectoryForDisturbanceRecovery.compute(getTimeInCurrentState());

         positionTrajectoryForDisturbanceRecovery.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
         desiredAngularVelocity.setToZero();
         desiredAngularAcceleration.setToZero();
      }
      else if (isTrajectoryStopped.getBooleanValue())
      {
         positionTrajectoryGenerator.getPosition(desiredPosition);
         orientationTrajectoryGenerator.getOrientation(desiredOrientation);
         desiredLinearVelocity.setToZero();
         desiredAngularVelocity.setToZero();
         desiredLinearAcceleration.setToZero();
         desiredAngularAcceleration.setToZero();
      }
      else
      {
         positionTrajectoryGenerator.compute(getTimeInCurrentState());
         orientationTrajectoryGenerator.compute(getTimeInCurrentState());

         if (positionTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
         {
            double firstTrajectoryPointTime = positionTrajectoryGenerator.getLastWaypointTime();
            FootTrajectoryCommand command = commandQueue.poll();
            numberOfQueuedCommands.decrement();
            initializeTrajectoryGenerators(command, false, firstTrajectoryPointTime);
            positionTrajectoryGenerator.compute(getTimeInCurrentState());
            orientationTrajectoryGenerator.compute(getTimeInCurrentState());
         }

         positionTrajectoryGenerator.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
         orientationTrajectoryGenerator.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
      }
   }

   private void initializeTrajectoryGenerators(FootTrajectoryCommand command, boolean initializeToCurrent, double firstTrajectoryPointTime)
   {
      command.addTimeOffset(firstTrajectoryPointTime);

      if (command.getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         if (initializeToCurrent)
         {
            tempPosition.setToZero(footFrame);
            tempOrientation.setToZero(footFrame);
         }
         else
         {
            positionTrajectoryGenerator.getPosition(tempPosition);
            orientationTrajectoryGenerator.getOrientation(tempOrientation);
         }

         tempOrientation.changeFrame(worldFrame);
         tempPosition.changeFrame(worldFrame);
         tempLinearVelocity.setToZero(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         positionTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.clear();
         positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempLinearVelocity);
         orientationTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }
      else
      {
         positionTrajectoryGenerator.clear();
         orientationTrajectoryGenerator.clear();
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         positionTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
         orientationTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   private int queueExceedingTrajectoryPointsIfNeeded(FootTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = positionTrajectoryGenerator.getMaximumNumberOfWaypoints() - positionTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      FootTrajectoryCommand commandForExcedent = commandQueue.addFirst();
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

   public void requestTouchdownForDisturbanceRecovery()
   {
      if (isPerformingTouchdown.getBooleanValue())
         return;

      positionTrajectoryGenerator.getPosition(desiredPosition);
      currentDesiredFootPosition.set(desiredPosition);
      touchdownInitialTimeProvider.setValue(getTimeInCurrentState());
      positionTrajectoryForDisturbanceRecovery.initialize();

      isPerformingTouchdown.set(true);
   }

   public void requestStopTrajectory()
   {
      isTrajectoryStopped.set(true);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      isTrajectoryStopped.set(false);
      isPerformingTouchdown.set(false);
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   @Override
   public boolean isDone()
   {
      return true;
   }
}