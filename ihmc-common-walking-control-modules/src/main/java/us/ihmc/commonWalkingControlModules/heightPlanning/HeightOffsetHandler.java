package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

public class HeightOffsetHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean isTrajectoryOffsetStopped = new YoBoolean("isPelvisOffsetHeightTrajectoryStopped", registry);

   private final YoDouble offsetHeightAboveGround = new YoDouble("offsetHeightAboveGround", registry);

   private final YoDouble offsetHeightAboveGroundPrevValue = new YoDouble("offsetHeightAboveGroundPrevValue", registry);
   private final YoDouble offsetHeightAboveGroundChangedTime = new YoDouble("offsetHeightAboveGroundChangedTime", registry);
   private final YoDouble offsetHeightAboveGroundTrajectoryOutput = new YoDouble("offsetHeightAboveGroundTrajectoryOutput", registry);
   private final YoDouble offsetHeightAboveGroundTrajectoryTimeProvider = new YoDouble("offsetHeightAboveGroundTrajectoryTimeProvider", registry);
   private final MultipleWaypointsTrajectoryGenerator offsetHeightTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator("pelvisHeightOffset",
                                                                                                                                 registry);

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final YoLong numberOfQueuedCommands;
   private final RecyclingArrayDeque<PelvisHeightTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisHeightTrajectoryCommand.class,
                                                                                                             PelvisHeightTrajectoryCommand::set);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private ReferenceFrame internalReferenceFrame;

   private final DoubleProvider yoTime;

   public HeightOffsetHandler(DoubleProvider yoTime, YoRegistry parentRegistry)
   {
      this.yoTime = yoTime;

      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightAboveGroundTrajectoryTimeProvider.set(0.5);
      offsetHeightAboveGround.set(0.0);
      offsetHeightAboveGroundPrevValue.set(0.0);
      offsetHeightAboveGround.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
            double previous = offsetHeightTrajectoryGenerator.getValue();
            offsetHeightTrajectoryGenerator.clear();
            offsetHeightTrajectoryGenerator.appendWaypoint(0.0, previous, 0.0);
            offsetHeightTrajectoryGenerator.appendWaypoint(offsetHeightAboveGroundTrajectoryTimeProvider.getValue(),
                                                           offsetHeightAboveGround.getDoubleValue(),
                                                           0.0);
            offsetHeightTrajectoryGenerator.initialize();
         }
      });

      String namePrefix = "pelvisHeight";
      lastCommandId = new YoLong(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new YoBoolean(namePrefix + "IsReadyToHandleQueuedPelvisHeightTrajectoryCommands", registry);
      numberOfQueuedCommands = new YoLong(namePrefix + "NumberOfQueuedCommands", registry);

      parentRegistry.addChild(registry);
   }

   public void setReferenceFrame(ReferenceFrame internalReferenceFrame)
   {
      this.internalReferenceFrame = internalReferenceFrame;
   }

   public void reset()
   {
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);
      isReadyToHandleQueuedCommands.set(false);
      numberOfQueuedCommands.set(0);

      offsetHeightAboveGround.set(0.0, false);
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, 0.0, 0.0);
      offsetHeightTrajectoryGenerator.initialize();
   }

   public void update(double currentDesiredHeight)
   {
      if (!isTrajectoryOffsetStopped.getBooleanValue())
      {
         double deltaTime = yoTime.getValue() - offsetHeightAboveGroundChangedTime.getDoubleValue();

         if (!offsetHeightTrajectoryGenerator.isEmpty())
         {
            offsetHeightTrajectoryGenerator.compute(deltaTime);
         }

         if (offsetHeightTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
         {
            double firstTrajectoryPointTime = offsetHeightTrajectoryGenerator.getLastWaypointTime();
            PelvisHeightTrajectoryCommand command = commandQueue.poll();
            numberOfQueuedCommands.decrement();
            initializeOffsetTrajectoryGenerator(command, firstTrajectoryPointTime, currentDesiredHeight);
            offsetHeightTrajectoryGenerator.compute(deltaTime);
         }

         offsetHeightAboveGround.set(offsetHeightTrajectoryGenerator.getValue(), false);
      }
      offsetHeightAboveGroundTrajectoryOutput.set(offsetHeightTrajectoryGenerator.getValue());

      offsetHeightAboveGroundPrevValue.set(offsetHeightTrajectoryGenerator.getValue());
   }

   public void initializeToCurrent(double currentHeightOffset)
   {
      offsetHeightAboveGround.set(currentHeightOffset);
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());

      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, currentHeightOffset, 0.0);
      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   public double getOffsetHeightAboveGround()
   {
      return offsetHeightAboveGround.getDoubleValue();
   }

   public double getOffsetHeightAboveGroundTrajectoryOutput()
   {
      return offsetHeightAboveGroundTrajectoryOutput.getValue();
   }

   private final PelvisHeightTrajectoryCommand tempPelvisHeightTrajectoryCommand = new PelvisHeightTrajectoryCommand();

   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command, double currentDesiredHeight)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();

      if (!se3Trajectory.getSelectionMatrix().isLinearZSelected())
         return false; // The user does not want to control the height, do nothing.

      se3Trajectory.changeFrame(worldFrame);
      tempPelvisHeightTrajectoryCommand.set(command);
      handlePelvisHeightTrajectoryCommand(tempPelvisHeightTrajectoryCommand, currentDesiredHeight);
      return true;
   }

   public boolean handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command, double currentDesiredHeight)
   {
      EuclideanTrajectoryControllerCommand euclideanTrajectory = command.getEuclideanTrajectory();

      if (euclideanTrajectory.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(euclideanTrajectory.getCommandId());
         offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
         initializeOffsetTrajectoryGenerator(command, 0.0, currentDesiredHeight);
         return true;
      }
      else if (euclideanTrajectory.getExecutionMode() == ExecutionMode.QUEUE)
      {
         boolean success = queuePelvisHeightTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            offsetHeightTrajectoryGenerator.clear();
            offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundPrevValue.getDoubleValue(), 0.0);
            offsetHeightTrajectoryGenerator.initialize();
         }
         return success;
      }
      else if (euclideanTrajectory.getExecutionMode() == ExecutionMode.STREAM)
      {
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(euclideanTrajectory.getCommandId());
         offsetHeightAboveGroundChangedTime.set(yoTime.getValue());

         if (euclideanTrajectory.getNumberOfTrajectoryPoints() == 0 || euclideanTrajectory.getNumberOfTrajectoryPoints() > 1)
         {
            LogTools.warn(
                  "When streaming, trajectories should contain either 1 or 2 trajectory point(s), was: " + euclideanTrajectory.getNumberOfTrajectoryPoints());
            return false;
         }

         FrameEuclideanTrajectoryPoint firstPoint = euclideanTrajectory.getTrajectoryPoint(0);

         if (firstPoint.getTime() != 0.0)
         {
            LogTools.warn("When streaming, the trajectory point should have a time of zero, was: " + firstPoint.getTime());
            return false;
         }

         offsetHeightTrajectoryGenerator.clear();

         double time = firstPoint.getTime();
         double z = fromAbsoluteToOffset(firstPoint.getPositionZ(), currentDesiredHeight);
         double zDot = firstPoint.getLinearVelocityZ();
         offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);

         if (euclideanTrajectory.getNumberOfTrajectoryPoints() == 1)
         {
            time = euclideanTrajectory.getStreamIntegrationDuration();
            z += time * zDot;
            offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);
         }
         else
         {
            FrameEuclideanTrajectoryPoint secondPoint = euclideanTrajectory.getTrajectoryPoint(1);

            if (secondPoint.getTime() != euclideanTrajectory.getStreamIntegrationDuration())
            {
               LogTools.warn("When streaming, the second trajectory point should have a time equal to the integration duration, was: " + secondPoint.getTime());
               return false;
            }

            time = secondPoint.getTime();
            z = fromAbsoluteToOffset(secondPoint.getPositionZ(), currentDesiredHeight);
            zDot = secondPoint.getLinearVelocityZ();
            offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);
         }

         offsetHeightTrajectoryGenerator.initialize();
         isTrajectoryOffsetStopped.set(false);
         return true;
      }
      else
      {
         LogTools.warn("Unknown {} value: {}. Command ignored.", ExecutionMode.class.getSimpleName(), euclideanTrajectory.getExecutionMode());
         return false;
      }
   }

   private boolean queuePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         LogTools.warn("The very first {} of a series must be {}. Aborting motion.", command.getClass().getSimpleName(), ExecutionMode.OVERRIDE);
         return false;
      }

      long previousCommandId = command.getEuclideanTrajectory().getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         LogTools.warn("Previous command ID mismatch: previous ID from command = {}, last message ID received by the controller = {}. Aborting motion.",
                       previousCommandId,
                       lastCommandId.getLongValue());
         return false;
      }

      if (command.getEuclideanTrajectory().getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         LogTools.warn("Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getEuclideanTrajectory().getCommandId());

      return true;
   }

   private void initializeOffsetTrajectoryGenerator(PelvisHeightTrajectoryCommand command, double firstTrajectoryPointTime, double currentDesiredHeight)
   {
      command.getEuclideanTrajectory().addTimeOffset(firstTrajectoryPointTime);

      offsetHeightTrajectoryGenerator.clear();

      if (command.getEuclideanTrajectory().getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundPrevValue.getDoubleValue(), 0.0);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         appendTrajectoryPoint(command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex), currentDesiredHeight);
      }

      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   /**
    * Appends the given trajectory point to the trajectory generator. This method handles the
    * conversion of absolute coordinate to offset.
    */
   private void appendTrajectoryPoint(FrameEuclideanTrajectoryPoint trajectoryPoint, double currentDesiredHeight)
   {
      double time = trajectoryPoint.getTime();
      tempPoint.setIncludingFrame(trajectoryPoint.getPosition());
      tempPoint.changeFrame(worldFrame);
      double z = fromAbsoluteToOffset(tempPoint.getZ(), currentDesiredHeight);
      double zDot = trajectoryPoint.getLinearVelocityZ();
      offsetHeightTrajectoryGenerator.appendWaypoint(time, z, zDot);
   }

   private double fromAbsoluteToOffset(double zInWorld, double currentDesiredHeight)
   {
      // TODO (Sylvain) Check if that's the right way to do it
      // FIXME the given coordinate is always assumed to be in world which doesn't seem right.
      tempPoint.setIncludingFrame(worldFrame, 0.0, 0.0, zInWorld);
      tempPoint.changeFrame(internalReferenceFrame);

      double zOffset = tempPoint.getZ() - currentDesiredHeight;
      return zOffset;
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisHeightTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getEuclideanTrajectory().getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints =
            offsetHeightTrajectoryGenerator.getMaximumNumberOfWaypoints() - offsetHeightTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisHeightTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.getEuclideanTrajectory().setPropertiesOnly(command.getEuclideanTrajectory());

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.getEuclideanTrajectory().addTrajectoryPoint(command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getEuclideanTrajectory().getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.getEuclideanTrajectory().subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void goHome(double trajectoryTime)
   {
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundPrevValue.getDoubleValue(), 0.0);
      offsetHeightTrajectoryGenerator.appendWaypoint(trajectoryTime, 0.0, 0.0);
      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryOffsetStopped.set(command.isStopAllTrajectory());
      offsetHeightAboveGround.set(offsetHeightAboveGroundPrevValue.getDoubleValue());
   }

   public double getOffsetHeightAboveGroundChangedTime()
   {
      return offsetHeightAboveGroundChangedTime.getDoubleValue();
   }
}
