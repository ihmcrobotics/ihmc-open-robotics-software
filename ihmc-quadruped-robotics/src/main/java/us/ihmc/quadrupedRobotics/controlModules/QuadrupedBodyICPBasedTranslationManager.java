package us.ihmc.quadrupedRobotics.controlModules;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.QuadrupedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyTaskspaceControlState;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedBodyTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.log.LogTools;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector2D;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class QuadrupedBodyICPBasedTranslationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean useFeedForward = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble supportPolygonSafeMargin = new YoDouble("supportPolygonSafeMargin", registry);
   private final YoDouble frozenOffsetDecayAlpha = new YoDouble("frozenOffsetDecayAlpha", registry);

   private final YoFramePoint2D desiredBodyPosition = new YoFramePoint2D("desiredBodyPosition", worldFrame, registry);
   private final YoFramePoint2D currentBodyPosition = new YoFramePoint2D("currentBodyPosition", worldFrame, registry);

   private final YoDouble initialBodyPositionTime = new YoDouble("initialBodyPositionTime", registry);

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final YoPIDGains bodyPositionGains = new YoPIDGains("_BodyPositionGains", registry);
   private final PIDController xController = new PIDController(bodyPositionGains, "BodyXPosition", registry);
   private final PIDController yController = new PIDController(bodyPositionGains, "BodyYPosition", registry);

   private final YoFrameVector2D desiredICPOffsetFeedback = new YoFrameVector2D("desiredICPOffsetFeedback", worldFrame, registry);
   private final YoFrameVector2D desiredICPOffsetFeedForward = new YoFrameVector2D("desiredICPOffsetFeedForward", worldFrame, registry);
   private final YoFrameVector2D desiredICPOffsetAction = new YoFrameVector2D("desiredICPOffset", worldFrame, registry);
   private final Vector2DReadOnly userOffset = new ParameterVector2D("userDesiredICPOffset", new Vector2D(), registry);

   private final YoFramePoint3D originalDCMWithNoOffset = new YoFramePoint3D("originalDCMWithNoOffset", worldFrame, registry);
   private final FramePoint3D originalDCMToModify = new FramePoint3D();


   private final YoBoolean isEnabled = new YoBoolean("isBodyTranslationManagerEnabled", registry);
   private final YoBoolean isRunning = new YoBoolean("isBodyTranslationManagerRunning", registry);
   private final YoBoolean isFrozen = new YoBoolean("isBodyTranslationManagerFrozen", registry);

   private final YoBoolean constrainAdjustedDCMToBeInSupport = new YoBoolean("constrainAdjustedDCMToBeInSupport", registry);

   private final BooleanParameter manualMode = new BooleanParameter("manualModeICPOffset", registry, false);

   private final YoDouble yoTime;
   private final double controlDT;

   private final YoBoolean isTrajectoryStopped = new YoBoolean("isBodyTranslationalTrajectoryStopped", registry);

   private final ReferenceFrame bodyZUpFrame;
   private final ReferenceFrame centerFeetZUpFrame;

   private final QuadrupedSupportPolygons quadrupedSupportPolygons;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FramePoint2D tempPosition2d = new FramePoint2D();
   private final FrameVector3D tempICPOffset = new FrameVector3D();
   private final FrameVector3D icpOffsetForFreezing = new FrameVector3D();

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final QuadrupedBodyTrajectoryCommand commandBeingProcessed = new QuadrupedBodyTrajectoryCommand();
   private final YoLong numberOfQueuedCommands;
   private final RecyclingArrayDeque<QuadrupedBodyTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(QuadrupedBodyTrajectoryCommand.class, QuadrupedBodyTrajectoryCommand::set);

   public QuadrupedBodyICPBasedTranslationManager(QuadrupedControllerToolbox controllerToolbox, double bodyTranslationICPSupportPolygonSafeMargin, YoVariableRegistry parentRegistry)
   {
      supportPolygonSafeMargin.set(bodyTranslationICPSupportPolygonSafeMargin);
      frozenOffsetDecayAlpha.set(0.998);

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      yoTime = runtimeEnvironment.getRobotTimestamp();
      controlDT = runtimeEnvironment.getControlDT();
      bodyZUpFrame = controllerToolbox.getReferenceFrames().getBodyZUpFrame();
      centerFeetZUpFrame = controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      this.quadrupedSupportPolygons = controllerToolbox.getSupportPolygons();

      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("bodyOffset", RigidBodyTaskspaceControlState.maxPointsInGenerator, centerFeetZUpFrame, registry);

      bodyPositionGains.setKp(1.5);
      bodyPositionGains.setKi(2.0);
      bodyPositionGains.setMaximumIntegralError(0.05);
      bodyPositionGains.setIntegralLeakRatio(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, controlDT));

      manualMode.addParameterChangedListener(v -> initialize());

      String namePrefix = "BodyXYTranslation";
      lastCommandId = new YoLong(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new YoBoolean(namePrefix + "IsReadyToHandleQueuedBodyTrajectoryCommands", registry);
      numberOfQueuedCommands = new YoLong(namePrefix + "NumberOfQueuedOffsetCommands", registry);

      parentRegistry.addChild(registry);
   }

   public void compute(FramePoint3DReadOnly desiredICP)
   {
      tempPosition2d.setToZero(bodyZUpFrame);
      currentBodyPosition.setMatchingFrame(tempPosition2d);

      if (isFrozen.getBooleanValue())
      {
         icpOffsetForFreezing.scale(frozenOffsetDecayAlpha.getDoubleValue());
         return;
      }

      if (!isEnabled.getBooleanValue())
      {
         desiredICPOffsetAction.setToZero();
         return;
      }

      if (manualMode.getValue())
         return;

      if (isRunning.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - initialBodyPositionTime.getDoubleValue();

            positionTrajectoryGenerator.compute(deltaTime);

            if (positionTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
            {
               double firstTrajectoryPointTime = positionTrajectoryGenerator.getLastWaypointTime();
               commandBeingProcessed.set(commandQueue.poll());
               numberOfQueuedCommands.decrement();
               initializeTrajectoryGenerator(commandBeingProcessed, firstTrajectoryPointTime);
               positionTrajectoryGenerator.compute(deltaTime);
            }
         }
         positionTrajectoryGenerator.getPosition(tempPosition);
         tempPosition.changeFrame(desiredBodyPosition.getReferenceFrame());
         desiredBodyPosition.set(tempPosition);
      }

      if (!isRunning.getBooleanValue())
      {
         desiredICPOffsetAction.setToZero();
         return;
      }

      desiredBodyPosition.checkReferenceFrameMatch(currentBodyPosition);
      double xAction = xController.compute(currentBodyPosition.getX(), desiredBodyPosition.getX(), 0.0, 0.0, controlDT);
      double yAction = yController.compute(currentBodyPosition.getY(), desiredBodyPosition.getY(), 0.0, 0.0, controlDT);

      desiredBodyPosition.checkReferenceFrameMatch(desiredICPOffsetFeedback);
      desiredICPOffsetFeedback.setX(xAction);
      desiredICPOffsetFeedback.setY(yAction);

      if (useFeedForward)
      {
         tempPosition2d.setIncludingFrame(desiredICP);
         tempPosition2d.changeFrame(desiredICPOffsetFeedForward.getReferenceFrame());
         desiredICPOffsetFeedForward.setMatchingFrame(desiredBodyPosition);
         desiredICPOffsetFeedForward.sub(tempPosition2d);
      }
      else
      {
         desiredICPOffsetFeedForward.setToZero();
      }

      desiredICPOffsetAction.set(desiredICPOffsetFeedForward);
      desiredICPOffsetAction.add(desiredICPOffsetFeedback);
   }

   public boolean isEnabled()
   {
      return isEnabled.getBooleanValue();
   }

   public void goToHome()
   {
      freeze();
   }

   public void holdCurrentPosition()
   {
      initialBodyPositionTime.set(yoTime.getDoubleValue());

      tempPosition.setToZero(bodyZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);

      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.changeFrame(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   public void handleBodyTrajectoryCommand(QuadrupedBodyTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      SelectionMatrix3D linearSelectionMatrix = se3Trajectory.getSelectionMatrix().getLinearPart();

      if (!linearSelectionMatrix.isXSelected() && !linearSelectionMatrix.isYSelected())
         return; // The user does not want to control the x and y, do nothing.

      if (se3Trajectory.getExecutionMode() == ExecutionMode.OVERRIDE)
      {
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(se3Trajectory.getCommandId());
         initialBodyPositionTime.set(yoTime.getDoubleValue());
         initializeTrajectoryGenerator(command, 0.0);
         return;
      }
      else if (se3Trajectory.getExecutionMode() == ExecutionMode.QUEUE)
      {
         boolean success = queueBodyTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            holdCurrentPosition();
         }
         return;
      }
      else
      {
         LogTools.warn("Unknown " + ExecutionMode.class.getSimpleName() + " value: " + se3Trajectory.getExecutionMode() + ". Command ignored.");
      }
   }

   private boolean queueBodyTrajectoryCommand(QuadrupedBodyTrajectoryCommand command)
   {
      if (!isReadyToHandleQueuedCommands.getBooleanValue())
      {
         LogTools.warn("The very first " + command.getClass().getSimpleName() + " of a series must be " + ExecutionMode.OVERRIDE + ". Aborting motion.");
         return false;
      }

      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      long previousCommandId = se3Trajectory.getPreviousCommandId();

      if (previousCommandId != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != INVALID_MESSAGE_ID && lastCommandId.getLongValue() != previousCommandId)
      {
         LogTools.warn("Previous command ID mismatch: previous ID from command = " + previousCommandId
               + ", last message ID received by the controller = " + lastCommandId.getLongValue() + ". Aborting motion.");
         return false;
      }

      if (se3Trajectory.getTrajectoryPoint(0).getTime() < 1.0e-5)
      {
         LogTools.warn("Time of the first trajectory point of a queued command must be greater than zero. Aborting motion.");
         return false;
      }

      commandQueue.add(command);
      numberOfQueuedCommands.increment();
      lastCommandId.set(se3Trajectory.getCommandId());

      return true;
   }

   private void initializeTrajectoryGenerator(QuadrupedBodyTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();
      se3Trajectory.addTimeOffset(firstTrajectoryPointTime);

      double currentTime = yoTime.getDoubleValue();
      double timeShift = command.isExpressedInAbsoluteTime() ? 0.0 : currentTime;

      se3Trajectory.getTrajectoryPointList().addTimeOffset(timeShift);
      command.setIsExpressedInAbsoluteTime(true);

      if (se3Trajectory.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         if (isRunning.getBooleanValue())
            positionTrajectoryGenerator.getPosition(tempPosition);
         else
            tempPosition.setToZero(centerFeetZUpFrame);
         tempPosition.changeFrame(centerFeetZUpFrame);
         tempVelocity.setToZero(worldFrame);
         tempVelocity.changeFrame(centerFeetZUpFrame);

         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(centerFeetZUpFrame);
         positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      }
      else
      {
         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingOffsetTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         positionTrajectoryGenerator.changeFrame(centerFeetZUpFrame);
         FrameSE3TrajectoryPoint trajectoryPoint = se3Trajectory.getTrajectoryPoint(trajectoryPointIndex);
         tempPosition.changeFrame(worldFrame);
         tempVelocity.changeFrame(worldFrame);
         tempPosition.set(trajectoryPoint.getPositionX(), trajectoryPoint.getPositionY(), 0.0);
         tempVelocity.set(trajectoryPoint.getLinearVelocityX(), trajectoryPoint.getLinearVelocityY(), 0.0);
         tempPosition.changeFrame(centerFeetZUpFrame);
         tempVelocity.changeFrame(centerFeetZUpFrame);
         positionTrajectoryGenerator.appendWaypoint(trajectoryPoint.getTime(), tempPosition, tempVelocity);
      }

      positionTrajectoryGenerator.initialize();

      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   private int queueExceedingOffsetTrajectoryPointsIfNeeded(QuadrupedBodyTrajectoryCommand command)
   {
      return queueExceedingTrajectoryPointsIfNeeded(command, positionTrajectoryGenerator, commandQueue, numberOfQueuedCommands);
   }


   private static int queueExceedingTrajectoryPointsIfNeeded(QuadrupedBodyTrajectoryCommand command, MultipleWaypointsPositionTrajectoryGenerator trajectoryGenerator,
                                                      RecyclingArrayDeque<QuadrupedBodyTrajectoryCommand> queue, YoLong queueCounter)
   {
      int numberOfTrajectoryPoints = command.getSE3Trajectory().getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = trajectoryGenerator.getMaximumNumberOfWaypoints() - trajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      QuadrupedBodyTrajectoryCommand commandForExcedent = queue.addFirst();
      queueCounter.increment();
      commandForExcedent.clear();
      commandForExcedent.getSE3Trajectory().setPropertiesOnly(command.getSE3Trajectory());

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.getSE3Trajectory().addTrajectoryPoint(command.getSE3Trajectory().getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getSE3Trajectory().getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.getSE3Trajectory().subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void clearCommandQueue(long lastCommandId)
   {
      commandQueue.clear();
      numberOfQueuedCommands.set(0);
      this.lastCommandId.set(lastCommandId);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryStopped.set(command.isStopAllTrajectory());
   }

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D safeSupportPolygonToConstrainICPOffset = new FrameConvexPolygon2D();


   public void addDCMOffset(FixedFramePoint3DBasics desiredICPToModify)
   {
      originalDCMWithNoOffset.setMatchingFrame(desiredICPToModify);
      originalDCMToModify.setIncludingFrame(desiredICPToModify);
      originalDCMToModify.changeFrame(centerFeetZUpFrame);

      if (!isEnabled.getBooleanValue() || (!isRunning.getBooleanValue() && !manualMode.getValue()))
      {
         desiredICPOffsetAction.setToZero();
         icpOffsetForFreezing.setToZero();
         return;
      }

      if (manualMode.getValue())
      {
         tempICPOffset.setIncludingFrame(centerFeetZUpFrame, userOffset, 0.0);
      }
      else
      {
         tempICPOffset.setIncludingFrame(desiredICPOffsetAction.getReferenceFrame(), desiredICPOffsetAction, 0.0);
         tempICPOffset.changeFrame(centerFeetZUpFrame);
      }

      if (isFrozen.getBooleanValue())
      {
         icpOffsetForFreezing.changeFrame(desiredICPOffsetAction.getReferenceFrame());
         desiredICPOffsetAction.set(icpOffsetForFreezing);
         icpOffsetForFreezing.changeFrame(desiredICPToModify.getReferenceFrame());
         desiredICPToModify.add(icpOffsetForFreezing);
      }
      else
      {
         tempICPOffset.changeFrame(desiredICPToModify.getReferenceFrame());
         desiredICPToModify.add(tempICPOffset);

         if (constrainAdjustedDCMToBeInSupport.getBooleanValue())
         {
            convexPolygonShrinker.scaleConvexPolygon(quadrupedSupportPolygons.getSupportPolygonInMidFeetZUp(), supportPolygonSafeMargin.getDoubleValue(),
                    safeSupportPolygonToConstrainICPOffset);
            tempPosition2d.setIncludingFrame(desiredICPToModify);
            safeSupportPolygonToConstrainICPOffset.orthogonalProjection(tempPosition2d);
            tempPosition2d.changeFrame(desiredICPToModify.getReferenceFrame());
            desiredICPToModify.set(tempPosition2d);
         }

         icpOffsetForFreezing.setIncludingFrame(desiredICPToModify);
         originalDCMToModify.changeFrame(desiredICPToModify.getReferenceFrame());
         icpOffsetForFreezing.sub(originalDCMToModify);
      }
   }

   public void disable()
   {
      isEnabled.set(false);
      isRunning.set(false);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);

      xController.resetIntegrator();
      yController.resetIntegrator();

      desiredICPOffsetAction.setToZero();
      desiredICPOffsetFeedForward.setToZero();
      desiredICPOffsetFeedback.setToZero();
   }

   public void enable()
   {
      if (isEnabled.getBooleanValue())
         return;
      isEnabled.set(true);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);
      initialize();
   }

   public void freeze()
   {
      isFrozen.set(true);
   }

   private void initialize()
   {
      initialBodyPositionTime.set(yoTime.getDoubleValue());
      tempPosition.setToZero(bodyZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);
      positionTrajectoryGenerator.clear(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
   }
}
