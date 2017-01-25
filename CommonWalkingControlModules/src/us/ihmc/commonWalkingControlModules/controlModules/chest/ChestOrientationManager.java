package us.ihmc.commonWalkingControlModules.controlModules.chest;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.command.CommandArrayDeque;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.GoHomeCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.io.printing.PrintTools;

public class ChestOrientationManager implements ChestOrientationManagerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);

   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isChestOrientationTrajectoryStopped", registry);
   private final BooleanYoVariable isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);

   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasChestOrientationManagerBeenInitialized", registry);

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame chestFrame;

   private final FrameOrientation tempOrientation = new FrameOrientation();

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameSO3TrajectoryPoint initialTrajectoryPoint = new FrameSO3TrajectoryPoint();

   private final YoFrameVector yoChestAngularWeight = new YoFrameVector("chestWeight", null, registry);
   private final Vector3d chestAngularWeight = new Vector3d();

   private final YoOrientationPIDGainsInterface gains;

   private final LongYoVariable lastCommandId;

   private final BooleanYoVariable isReadyToHandleQueuedCommands;
   private final LongYoVariable numberOfQueuedCommands;
   private final CommandArrayDeque<ChestTrajectoryCommand> commandQueue = new CommandArrayDeque<>(ChestTrajectoryCommand.class);

   private final BooleanYoVariable followChestRollSineWave = new BooleanYoVariable("followChestRollSineWave", registry);
   private final DoubleYoVariable chestRollSineFrequency = new DoubleYoVariable("chestRollSineFrequency", registry);
   private final DoubleYoVariable chestRollSineMagnitude = new DoubleYoVariable("chestRollSineMagnitude", registry);

   public ChestOrientationManager(HighLevelHumanoidControllerToolbox momentumBasedController, YoOrientationPIDGainsInterface gains, Vector3d angularWeight,
         double trajectoryTime, YoVariableRegistry parentRegistry)
   {
      this.gains = gains;
      yoTime = momentumBasedController.getYoTime();
      pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();

      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      chestFrame = chest.getBodyFixedFrame();

      yoChestAngularWeight.set(angularWeight);
      yoChestAngularWeight.get(chestAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(chestAngularWeight);
      orientationFeedbackControlCommand.set(elevator, chest);
      orientationFeedbackControlCommand.setGains(gains);

      boolean allowMultipleFrames = true;
      String namePrefix = "chest";
      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator(namePrefix, allowMultipleFrames, pelvisZUpFrame, registry);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

      lastCommandId = new LongYoVariable(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new BooleanYoVariable(namePrefix + "IsReadyToHandleQueuedChestTrajectoryCommands", registry);
      numberOfQueuedCommands = new LongYoVariable(namePrefix + "NumberOfQueuedCommands", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      if (hasBeenInitialized.getBooleanValue())
         return;

      hasBeenInitialized.set(true);

      holdCurrentOrientation();
   }

   @Override
   public void compute()
   {
      if (isTrackingOrientation.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - receivedNewChestOrientationTime.getDoubleValue();
            orientationTrajectoryGenerator.compute(deltaTime);

            if (orientationTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
            {
               double firstTrajectoryPointTime = orientationTrajectoryGenerator.getLastWaypointTime();
               ChestTrajectoryCommand command = commandQueue.poll();
               numberOfQueuedCommands.decrement();
               initializeTrajectoryGenerator(command, firstTrajectoryPointTime);
               orientationTrajectoryGenerator.compute(deltaTime);
            }
         }
         boolean isTrajectoryDone = orientationTrajectoryGenerator.isDone();

         if (isTrajectoryDone)
            orientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);

         isTrackingOrientation.set(!isTrajectoryDone);
      }


      if (followChestRollSineWave.getBooleanValue())
      {
         double roll = chestRollSineMagnitude.getDoubleValue() * Math.sin(yoTime.getDoubleValue() * chestRollSineFrequency.getDoubleValue() * 2.0 * Math.PI);
         tempOrientation.setIncludingFrame(pelvisZUpFrame, 0.0, 0.0, roll);
         tempOrientation.changeFrame(worldFrame);

         desiredOrientation.set(tempOrientation);
      }

      else
      {
         orientationTrajectoryGenerator.getOrientation(desiredOrientation);
      }

      desiredAngularVelocity.setToZero(worldFrame);
      feedForwardAngularAcceleration.setToZero(worldFrame);
      orientationFeedbackControlCommand.changeFrameAndSet(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      orientationFeedbackControlCommand.setGains(gains);
      yoChestAngularWeight.get(chestAngularWeight);
      orientationFeedbackControlCommand.setWeightsForSolver(chestAngularWeight);
   }

   @Override
   public void holdCurrentOrientation()
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());
      initialTrajectoryPoint.setToZero(chestFrame);
      initialTrajectoryPoint.changeFrame(pelvisZUpFrame);

      orientationTrajectoryGenerator.clear(pelvisZUpFrame);
      orientationTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      orientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   @Override
   public void handleChestTrajectoryCommand(ChestTrajectoryCommand command)
   {
      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(command.getCommandId());
         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());
         initializeTrajectoryGenerator(command, 0.0);
         return;
      case QUEUE:
         boolean success = queueChestTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            holdCurrentOrientation();
         }
         return;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         return;
      }
   }

   private boolean queueChestTrajectoryCommand(ChestTrajectoryCommand command)
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
         return false;
      }

      if (command.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         PrintTools.warn(this, "Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(command.getCommandId());

      return true;
   }

   private void initializeTrajectoryGenerator(ChestTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      command.addTimeOffset(firstTrajectoryPointTime);

      if (command.getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         orientationTrajectoryGenerator.getOrientation(desiredOrientation);
         desiredOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         orientationTrajectoryGenerator.clear(worldFrame);
         orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
      }
      else
      {
         orientationTrajectoryGenerator.clear(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         orientationTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      orientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      orientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   private int queueExceedingTrajectoryPointsIfNeeded(ChestTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = orientationTrajectoryGenerator.getMaximumNumberOfWaypoints() - orientationTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      ChestTrajectoryCommand commandForExcedent = commandQueue.addFirst();
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

   @Override
   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
   }

   @Override
   public void handleGoHomeCommand(GoHomeCommand command)
   {
      if (command.getRequest(BodyPart.CHEST))
         goToHomeFromCurrentDesired(command.getTrajectoryTime());
   }

   @Override
   public void goToHomeFromCurrentDesired(double trajectoryTime)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      orientationTrajectoryGenerator.getOrientation(desiredOrientation);

      desiredOrientation.changeFrame(pelvisZUpFrame);
      desiredAngularVelocity.setToZero(pelvisZUpFrame);
      orientationTrajectoryGenerator.clear(pelvisZUpFrame);
      orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);

      desiredOrientation.setToZero(pelvisZUpFrame);
      orientationTrajectoryGenerator.appendWaypoint(trajectoryTime, desiredOrientation, desiredAngularVelocity);
      orientationTrajectoryGenerator.initialize();

      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   @Override
   public void goToHomeFromCurrent(double trajectoryTime)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());
      orientationTrajectoryGenerator.clear(pelvisZUpFrame);

      desiredAngularVelocity.setToZero(pelvisZUpFrame);
      desiredOrientation.setToZero(pelvisZUpFrame);
      initialTrajectoryPoint.setToZero(chestFrame);
      initialTrajectoryPoint.changeFrame(pelvisZUpFrame);

      orientationTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      orientationTrajectoryGenerator.appendWaypoint(trajectoryTime, desiredOrientation, desiredAngularVelocity);
      orientationTrajectoryGenerator.initialize();

      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);

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
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }
}
