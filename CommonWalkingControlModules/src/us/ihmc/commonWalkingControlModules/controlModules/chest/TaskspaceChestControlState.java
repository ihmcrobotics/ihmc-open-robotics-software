package us.ihmc.commonWalkingControlModules.controlModules.chest;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Vector3D;
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
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class TaskspaceChestControlState extends ChestControlState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();
   private final YoFrameVector yoChestAngularWeight = new YoFrameVector("chestWeight", null, registry);
   private final Vector3D chestAngularWeight = new Vector3D();
   private final YoOrientationPIDGainsInterface gains;

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();

   private final MultipleWaypointsOrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final FrameSO3TrajectoryPoint initialTrajectoryPoint = new FrameSO3TrajectoryPoint();
   private final FrameSO3TrajectoryPoint tempTrajectoryPoint = new FrameSO3TrajectoryPoint();
   private final ReferenceFrame pelvisZUpFrame;

   private final RecyclingArrayDeque<ChestTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(ChestTrajectoryCommand.class);
   private final BooleanYoVariable isReadyToHandleQueuedCommands = new BooleanYoVariable("chestIsReadyToHandleQueuedChestTrajectoryCommands", registry);
   private final LongYoVariable numberOfQueuedCommands = new LongYoVariable("chestNumberOfQueuedCommands", registry);

   private final LongYoVariable lastCommandId;

   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isChestOrientationTrajectoryStopped", registry);
   private final BooleanYoVariable isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);

   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);

   private final BooleanYoVariable followChestRollSineWave = new BooleanYoVariable("followChestRollSineWave", registry);
   private final DoubleYoVariable chestRollSineFrequency = new DoubleYoVariable("chestRollSineFrequency", registry);
   private final DoubleYoVariable chestRollSineMagnitude = new DoubleYoVariable("chestRollSineMagnitude", registry);

   private final FrameOrientation tempOrientation = new FrameOrientation();

   public TaskspaceChestControlState(HighLevelHumanoidControllerToolbox humanoidControllerToolbox, YoOrientationPIDGainsInterface gains, YoVariableRegistry parentRegistry)
   {
      super(ChestControlMode.TASKSPACE);

      this.gains = gains;
      yoTime = humanoidControllerToolbox.getYoTime();

      FullRobotModel fullRobotModel = humanoidControllerToolbox.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      RigidBody elevator = fullRobotModel.getElevator();
      pelvisZUpFrame = humanoidControllerToolbox.getPelvisZUpFrame();

      orientationFeedbackControlCommand.set(elevator, chest);
      orientationFeedbackControlCommand.setGains(gains);

      orientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chest", true, pelvisZUpFrame, registry);
      orientationTrajectoryGenerator.registerNewTrajectoryFrame(worldFrame);

      lastCommandId = new LongYoVariable("chestLastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      parentRegistry.addChild(registry);
   }

   @Override
   public void doAction()
   {
      if (isTrackingOrientation.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - receivedNewChestOrientationTime.getDoubleValue();
            orientationTrajectoryGenerator.compute(deltaTime);

            if (orientationTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
            {
               orientationTrajectoryGenerator.getLastWaypoint(tempTrajectoryPoint);
               double firstTrajectoryPointTime = tempTrajectoryPoint.getTime();
               tempTrajectoryPoint.getOrientation(tempOrientation);
               ChestTrajectoryCommand command = commandQueue.poll();
               numberOfQueuedCommands.decrement();
               initializeTrajectoryGenerator(command, firstTrajectoryPointTime, tempOrientation);
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

   public void holdOrientation(FrameOrientation orientation)
   {
      desiredOrientation.setIncludingFrame(orientation);
      desiredOrientation.changeFrame(pelvisZUpFrame);
      initialTrajectoryPoint.setToZero(pelvisZUpFrame);
      initialTrajectoryPoint.setOrientation(desiredOrientation);
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      orientationTrajectoryGenerator.clear(pelvisZUpFrame);
      orientationTrajectoryGenerator.appendWaypoint(initialTrajectoryPoint);
      orientationTrajectoryGenerator.initialize();
      isTrackingOrientation.set(true);
      isTrajectoryStopped.set(false);
   }

   public void handleChestTrajectoryCommand(ChestTrajectoryCommand command, FrameOrientation currentDesired)
   {
      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(command.getCommandId());
         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());
         initializeTrajectoryGenerator(command, 0.0, currentDesired);
         return;
      case QUEUE:
         boolean success = queueChestTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            holdOrientation(currentDesired);
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

   private void initializeTrajectoryGenerator(ChestTrajectoryCommand command, double firstTrajectoryPointTime, FrameOrientation currentDesired)
   {
      command.addTimeOffset(firstTrajectoryPointTime);
      orientationTrajectoryGenerator.clear(worldFrame);

      if (command.getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
      {
         desiredOrientation.setIncludingFrame(currentDesired);
         desiredOrientation.changeFrame(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);
         orientationTrajectoryGenerator.appendWaypoint(0.0, desiredOrientation, desiredAngularVelocity);
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

   public void handleGoHomeCommand(GoHomeCommand command, FrameOrientation currentDesired)
   {
      if (command.getRequest(BodyPart.CHEST))
         goToHome(command.getTrajectoryTime(), currentDesired);
   }

   public void goToHome(double trajectoryTime, FrameOrientation currentDesired)
   {
      receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

      desiredOrientation.setIncludingFrame(currentDesired);
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

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void setWeights(Vector3D weights)
   {
      yoChestAngularWeight.set(weights);
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

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public void getDesiredOrientation(FrameOrientation desiredOrientationToPack)
   {
      orientationTrajectoryGenerator.getOrientation(desiredOrientationToPack);
   }

}
