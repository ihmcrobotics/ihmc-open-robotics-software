package us.ihmc.commonWalkingControlModules.controlModules;

import static us.ihmc.communication.packets.Packet.INVALID_MESSAGE_ID;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

public class PelvisICPBasedTranslationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble supportPolygonSafeMargin = new YoDouble("supportPolygonSafeMargin", registry);
   private final YoDouble frozenOffsetDecayAlpha = new YoDouble("frozenOffsetDecayAlpha", registry);

   private final YoFramePoint2d desiredPelvisPosition = new YoFramePoint2d("desiredPelvis", worldFrame, registry);

   private final YoDouble initialPelvisPositionTime = new YoDouble("initialPelvisPositionTime", registry);

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectoryGenerator;

   private final YoFrameVector2d pelvisPositionError = new YoFrameVector2d("pelvisPositionError", worldFrame, registry);
   private final YoDouble proportionalGain = new YoDouble("pelvisPositionProportionalGain", registry);
   private final YoFrameVector2d proportionalTerm = new YoFrameVector2d("pelvisPositionProportionalTerm", worldFrame, registry);

   private final YoFrameVector2d pelvisPositionCumulatedError = new YoFrameVector2d("pelvisPositionCumulatedError", worldFrame, registry);
   private final YoDouble integralGain = new YoDouble("pelvisPositionIntegralGain", registry);
   private final YoFrameVector2d integralTerm = new YoFrameVector2d("pelvisPositionIntegralTerm", worldFrame, registry);
   private final YoDouble maximumIntegralError = new YoDouble("maximumPelvisPositionIntegralError", registry);

   private final YoFrameVector2d desiredICPOffset = new YoFrameVector2d("desiredICPOffset", worldFrame, registry);

   private final YoBoolean isEnabled = new YoBoolean("isPelvisTranslationManagerEnabled", registry);
   private final YoBoolean isRunning = new YoBoolean("isPelvisTranslationManagerRunning", registry);
   private final YoBoolean isFrozen = new YoBoolean("isPelvisTranslationManagerFrozen", registry);

   private final YoBoolean manualMode = new YoBoolean("manualModeICPOffset", registry);

   private final YoDouble yoTime;
   private final double controlDT;

   private final YoBoolean isTrajectoryStopped = new YoBoolean("isPelvisTranslationalTrajectoryStopped", registry);

   private ReferenceFrame supportFrame;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<MovingReferenceFrame> ankleZUpFrames;

   private final BipedSupportPolygons bipedSupportPolygons;
   private FrameConvexPolygon2d supportPolygon;

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();
   private final FramePoint2D tempPosition2d = new FramePoint2D();
   private final FrameVector2D tempError2d = new FrameVector2D();
   private final FrameVector2D tempICPOffset = new FrameVector2D();
   private final FrameVector2D icpOffsetForFreezing = new FrameVector2D();

   private final YoLong lastCommandId;

   private final YoBoolean isReadyToHandleQueuedCommands;
   private final YoLong numberOfQueuedCommands;
   private final RecyclingArrayDeque<PelvisTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisTrajectoryCommand.class);

   public PelvisICPBasedTranslationManager(HighLevelHumanoidControllerToolbox controllerToolbox, double pelvisTranslationICPSupportPolygonSafeMargin, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry)
   {
      supportPolygonSafeMargin.set(pelvisTranslationICPSupportPolygonSafeMargin);
      frozenOffsetDecayAlpha.set(0.998);
      
      yoTime = controllerToolbox.getYoTime();
      controlDT = controllerToolbox.getControlDT();
      pelvisZUpFrame = controllerToolbox.getPelvisZUpFrame();
      midFeetZUpFrame = controllerToolbox.getReferenceFrames().getMidFeetZUpFrame();
      ankleZUpFrames = controllerToolbox.getReferenceFrames().getAnkleZUpReferenceFrames();

      this.bipedSupportPolygons = bipedSupportPolygons;

      boolean allowMultipleFrames = true;
      positionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("pelvisOffset", allowMultipleFrames, worldFrame, registry);
      positionTrajectoryGenerator.registerNewTrajectoryFrame(midFeetZUpFrame);
      for (RobotSide robotSide : RobotSide.values)
         positionTrajectoryGenerator.registerNewTrajectoryFrame(ankleZUpFrames.get(robotSide));

      proportionalGain.set(0.5);
      integralGain.set(1.5);
      maximumIntegralError.set(0.08);

      manualMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            initialize();
         }
      });

      String namePrefix = "PelvisXYTranslation";
      lastCommandId = new YoLong(namePrefix + "LastCommandId", registry);
      lastCommandId.set(Packet.INVALID_MESSAGE_ID);

      isReadyToHandleQueuedCommands = new YoBoolean(namePrefix + "IsReadyToHandleQueuedPelvisTrajectoryCommands", registry);
      numberOfQueuedCommands = new YoLong(namePrefix + "NumberOfQueuedCommands", registry);

      parentRegistry.addChild(registry);
   }

   public void compute(RobotSide supportLeg, FramePoint2D actualICP)
   {
      if (isFrozen.getBooleanValue())
      {
         icpOffsetForFreezing.scale(frozenOffsetDecayAlpha.getDoubleValue());
         return;
      }

      if (supportLeg == null)
      {
         supportFrame = midFeetZUpFrame;
         supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      }
      else
      {
         supportFrame = ankleZUpFrames.get(supportLeg);
         supportPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
      }

      if (!isEnabled.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         return;
      }

      if (manualMode.getBooleanValue())
         return;

      if (isRunning.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - initialPelvisPositionTime.getDoubleValue();
            positionTrajectoryGenerator.compute(deltaTime);

            if (positionTrajectoryGenerator.isDone() && !commandQueue.isEmpty())
            {
               double firstTrajectoryPointTime = positionTrajectoryGenerator.getLastWaypointTime();
               PelvisTrajectoryCommand command = commandQueue.poll();
               numberOfQueuedCommands.decrement();
               initializeTrajectoryGenerator(command, firstTrajectoryPointTime);
               positionTrajectoryGenerator.compute(deltaTime);
            }
         }
         positionTrajectoryGenerator.getPosition(tempPosition);
         tempPosition.changeFrame(desiredPelvisPosition.getReferenceFrame());
         desiredPelvisPosition.setByProjectionOntoXYPlane(tempPosition);
      }

      if (!isRunning.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         return;
      }

      computeDesiredICPOffset();
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
      initialPelvisPositionTime.set(yoTime.getDoubleValue());

      tempPosition.setToZero(pelvisZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);

      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.changeFrame(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   public void handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SelectionMatrix3D linearSelectionMatrix = command.getSelectionMatrix().getLinearPart();

      if (!linearSelectionMatrix.isXSelected() && !linearSelectionMatrix.isYSelected())
         return; // The user does not want to control the x and y of the pelvis, do nothing.

      switch (command.getExecutionMode())
      {
      case OVERRIDE:
         isReadyToHandleQueuedCommands.set(true);
         clearCommandQueue(command.getCommandId());
         initialPelvisPositionTime.set(yoTime.getDoubleValue());
         initializeTrajectoryGenerator(command, 0.0);
         return;
      case QUEUE:
         boolean success = queuePelvisTrajectoryCommand(command);
         if (!success)
         {
            isReadyToHandleQueuedCommands.set(false);
            clearCommandQueue(INVALID_MESSAGE_ID);
            holdCurrentPosition();
         }
         return;
      default:
         PrintTools.warn(this, "Unknown " + ExecutionMode.class.getSimpleName() + " value: " + command.getExecutionMode() + ". Command ignored.");
         break;
      }
   }

   private boolean queuePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
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

   private void initializeTrajectoryGenerator(PelvisTrajectoryCommand command, double firstTrajectoryPointTime)
   {
      command.addTimeOffset(firstTrajectoryPointTime);

      if (command.getTrajectoryPoint(0).getTime() > 1.0e-5)
      {
         if (isRunning.getBooleanValue())
            positionTrajectoryGenerator.getPosition(tempPosition);
         else
            tempPosition.setToZero(pelvisZUpFrame);
         tempPosition.changeFrame(worldFrame);
         tempVelocity.setToZero(worldFrame);

         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
         positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      }
      else
      {
         positionTrajectoryGenerator.clear();
         positionTrajectoryGenerator.changeFrame(worldFrame);
      }

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         positionTrajectoryGenerator.appendWaypoint(command.getTrajectoryPoint(trajectoryPointIndex));
      }

      if (supportFrame != null)
         positionTrajectoryGenerator.changeFrame(supportFrame);
      else
         positionTrajectoryGenerator.changeFrame(worldFrame);

      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
      isRunning.set(true);
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints = positionTrajectoryGenerator.getMaximumNumberOfWaypoints() - positionTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisTrajectoryCommand commandForExcedent = commandQueue.addFirst();
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

   private void computeDesiredICPOffset()
   {
      pelvisPositionError.set(desiredPelvisPosition);
      tempPosition2d.setToZero(pelvisZUpFrame);
      tempPosition2d.changeFrame(worldFrame);
      pelvisPositionError.sub(tempPosition2d);

      pelvisPositionError.getFrameTuple2dIncludingFrame(tempError2d);
      tempError2d.scale(controlDT);
      pelvisPositionCumulatedError.add(tempError2d);

      double cumulativeErrorMagnitude = pelvisPositionCumulatedError.length();
      if (cumulativeErrorMagnitude > maximumIntegralError.getDoubleValue())
      {
         pelvisPositionCumulatedError.scale(maximumIntegralError.getDoubleValue() / cumulativeErrorMagnitude);
      }

      proportionalTerm.set(pelvisPositionError);
      proportionalTerm.scale(proportionalGain.getDoubleValue());

      integralTerm.set(pelvisPositionCumulatedError);
      integralTerm.scale(integralGain.getDoubleValue());

      desiredICPOffset.set(proportionalTerm);
      desiredICPOffset.add(integralTerm);
   }

   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2d safeSupportPolygonToConstrainICPOffset = new FrameConvexPolygon2d();

   private final FramePoint2D originalICPToModify = new FramePoint2D();

   public void addICPOffset(FramePoint2D desiredICPToModify, FrameVector2D desiredICPVelocityToModify)
   {
      desiredICPToModify.changeFrame(supportPolygon.getReferenceFrame());
      desiredICPVelocityToModify.changeFrame(supportPolygon.getReferenceFrame());

      originalICPToModify.setIncludingFrame(desiredICPToModify);

      if (!isEnabled.getBooleanValue() || (!isRunning.getBooleanValue() && !manualMode.getBooleanValue()))
      {
         desiredICPOffset.setToZero();
         icpOffsetForFreezing.setToZero();
         desiredICPToModify.changeFrame(worldFrame);
         desiredICPVelocityToModify.changeFrame(worldFrame);
         return;
      }

      if (manualMode.getBooleanValue())
      {
         // Ignore the desiredICPOffset frame assuming the user wants to control the ICP in the supportFrame
         tempICPOffset.setIncludingFrame(supportFrame, desiredICPOffset.getX(), desiredICPOffset.getY());
      }

      else
      {
         desiredICPOffset.getFrameTuple2dIncludingFrame(tempICPOffset);
         tempICPOffset.changeFrame(supportFrame);
      }

      if (isFrozen.getBooleanValue())
      {
         desiredICPOffset.setAndMatchFrame(icpOffsetForFreezing);
         desiredICPToModify.changeFrame(icpOffsetForFreezing.getReferenceFrame());
         desiredICPToModify.add(icpOffsetForFreezing);
      }

      else
      {
         desiredICPToModify.add(tempICPOffset);

         convexPolygonShrinker.scaleConvexPolygon(supportPolygon, supportPolygonSafeMargin.getDoubleValue(), safeSupportPolygonToConstrainICPOffset);
         safeSupportPolygonToConstrainICPOffset.orthogonalProjection(desiredICPToModify);

         icpOffsetForFreezing.setIncludingFrame(desiredICPToModify);
         icpOffsetForFreezing.sub(originalICPToModify);
      }

      desiredICPToModify.changeFrame(worldFrame);
      desiredICPVelocityToModify.changeFrame(worldFrame);
   }

   public void disable()
   {
      isEnabled.set(false);
      isRunning.set(false);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);

      pelvisPositionError.setToZero();
      pelvisPositionCumulatedError.setToZero();

      proportionalTerm.setToZero();
      integralTerm.setToZero();

      desiredICPOffset.setToZero();
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
      initialPelvisPositionTime.set(yoTime.getDoubleValue());
      tempPosition.setToZero(pelvisZUpFrame);
      tempPosition.changeFrame(worldFrame);
      tempVelocity.setToZero(worldFrame);
      positionTrajectoryGenerator.clear();
      positionTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      positionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      positionTrajectoryGenerator.initialize();
      isTrajectoryStopped.set(false);
   }
}
